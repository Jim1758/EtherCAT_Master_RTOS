#include "PositionAutoTuneJob.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <locale>
#include <limits>
#include <sstream>
#include <utility>
#ifdef _WIN32
#include <windows.h>
#include <rtapi.h>
#else
#include <cerrno>
#include <sys/stat.h>
#include <thread>
#endif

namespace PositionAutoTune {
namespace {
enum class Work : unsigned { Idle, Running, Ready };
std::atomic<Work> g_work{Work::Idle};
std::atomic<bool> g_cancel{false};
std::atomic<std::uint64_t> g_session{0ULL};
JobRequest g_request;
const CaptureSample* g_samples=nullptr;
std::size_t g_count=0U;
std::shared_ptr<JobResult> g_result;
bool g_revokeBaseline=false; // Owner handoff through worker creation / Ready.
// Files are audit records; only a successful P1 delivered in this runtime may
// authorize P2. Admission invalidation cannot fail because a directory is read-only.
std::atomic<std::uint64_t> g_cruiseEligible[kAxisCount]{};
std::atomic<std::uint64_t> g_cruiseRevision[kAxisCount]{};
std::uint64_t g_cruiseJobRevision=0ULL; // Immutable while a worker owns the job.
#ifdef _WIN32
HANDLE g_thread=nullptr;
#else
std::thread g_thread;
#endif
std::string Join(const std::string& dir,const std::string& leaf) {
    if(dir.empty()) return leaf;
    const char end=dir.back();
    return dir+(end=='/'||end=='\\'?"":"/")+leaf;
}
std::string Timestamp() {
#ifdef _WIN32
    // Same RTX-compatible wall-clock API used by HomePersistenceManager.
    SYSTEMTIME now{};GetSystemTime(&now);
    std::ostringstream out;out.imbue(std::locale::classic());out<<std::setfill('0')
        <<std::setw(4)<<now.wYear<<std::setw(2)<<now.wMonth<<std::setw(2)<<now.wDay
        <<'T'<<std::setw(2)<<now.wHour<<std::setw(2)<<now.wMinute<<std::setw(2)<<now.wSecond
        <<std::setw(3)<<now.wMilliseconds<<'Z';
    return out.str();
#else
    return std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
#endif
}
bool EnsureDirectory(const std::string& path) {
    if(path.empty()) return false;
#ifdef _WIN32
    // TUNE02 FIX2: use the same RTSS directory APIs as EDMConditionStartupIO.
    // ALREADY_EXISTS is provisional: each child-file open/write must still
    // succeed before its Saved flag is set. A regular-file collision therefore
    // reports RESULT_IO_FAILED without modifying that file.
    return CreateDirectoryA(path.c_str(),nullptr)!=0 || GetLastError()==ERROR_ALREADY_EXISTS;
#else
    if(::mkdir(path.c_str(),0700)==0) return true;
    struct stat info{};
    return errno==EEXIST && ::stat(path.c_str(),&info)==0 && S_ISDIR(info.st_mode);
#endif
}
// RTX64 supports the existing project's CreateFile/WriteFile path. Do not add
// unsupported MoveFileEx imports. A partially written baseline fails its exact
// checksum/schema validation; an older record is never silently used instead.
bool WriteChecked(const std::string& path,const std::string& text) {
#ifdef _WIN32
    HANDLE file=CreateFileA(path.c_str(),GENERIC_WRITE,FILE_SHARE_READ,nullptr,
        CREATE_ALWAYS,FILE_ATTRIBUTE_NORMAL|FILE_FLAG_WRITE_THROUGH,nullptr);
    if(file==INVALID_HANDLE_VALUE)return false;
    std::size_t offset=0U;bool good=true;
    while(offset<text.size()) {
        DWORD done=0U;
        const std::size_t left=text.size()-offset;
        const DWORD count=static_cast<DWORD>(left<65536U?left:65536U);
        if(!WriteFile(file,text.data()+offset,count,&done,nullptr)||done==0U) {good=false;break;}
        offset+=done;
    }
    if(!CloseHandle(file))good=false;
    return good;
#else
    std::ofstream out(path,std::ios::binary|std::ios::trunc);
    if(!out) return false;
    out.write(text.data(),static_cast<std::streamsize>(text.size())); out.flush();
    const bool good=out.good();out.close();
    return good&&!out.fail();
#endif
}
std::uint64_t Hash(const std::string& text) {
    std::uint64_t result=14695981039346656037ULL;
    for(unsigned char ch:text) {result^=ch;result*=1099511628211ULL;}
    return result;
}
bool Near(double a,double b) {
    return std::isfinite(a)&&std::isfinite(b)&&std::fabs(a-b)<=1e-9*(1.0+std::fabs(b));
}
bool SameGains(const CaptureGains& a,const CaptureGains& b) {
    return Near(a.kp,b.kp)&&Near(a.ki,b.ki)&&Near(a.kd,b.kd)&&Near(a.kvff,b.kvff);
}
void WriteGains(std::ostream& out,const CaptureGains& value) {
    out<<value.kp<<' '<<value.ki<<' '<<value.kd<<' '<<value.kvff<<'\n';
}
bool ReadGains(std::istream& in,CaptureGains& value) {
    return static_cast<bool>(in>>value.kp>>value.ki>>value.kd>>value.kvff)&&
        std::isfinite(value.kp)&&std::isfinite(value.ki)&&std::isfinite(value.kd)&&
        std::isfinite(value.kvff)&&value.kp>=0.0&&value.ki>=0.0&&value.kd>=0.0&&value.kvff>=0.0;
}
bool SupportedMotionGains(const CaptureGains& value) {
    return value.kp>0.0&&value.ki==0.0&&value.kd==0.0&&value.kvff>0.0;
}
bool SupportedIdleCandidate(const CaptureGains& value) {
    return value.kp>0.0&&value.ki>=0.0&&value.kd==0.0&&value.kvff==0.0;
}
void ClearComparison(JobResult& result) noexcept {
    result.motionComparison=result.idleComparison=PerformanceComparison::Unavailable;
    result.adoptionAdvice=AdoptionAdvice::Unavailable;
    result.comparisonPositionTolerance=result.comparisonTimeToleranceSeconds=0.0;
}
PerformanceComparison CompareMetrics(const double* before,const double* after,
    const double* tolerances,std::size_t count) noexcept {
    bool lower=false,higher=false;
    for(std::size_t i=0;i<count;++i) {
        if(!std::isfinite(before[i])||before[i]<0.0||!std::isfinite(after[i])||after[i]<0.0||
            !std::isfinite(tolerances[i])||tolerances[i]<=0.0) return PerformanceComparison::Unavailable;
        const double roundoff=32.0*(std::numeric_limits<double>::epsilon)()*
            (std::max)(std::fabs(before[i]),std::fabs(after[i]));
        const double tolerance=(std::max)(tolerances[i],roundoff);
        // Subtract nonnegative finite values instead of adding a tolerance
        // to a metric; that also avoids overflow for large finite magnitudes.
        lower=lower||(before[i]>after[i]&&before[i]-after[i]>tolerance);
        higher=higher||(after[i]>before[i]&&after[i]-before[i]>tolerance);
    }
    return lower?(higher?PerformanceComparison::Mixed:PerformanceComparison::Improved):
        higher?PerformanceComparison::Regressed:PerformanceComparison::Equivalent;
}
void EvaluateComparison(const JobRequest& job,JobResult& result) noexcept {
    ClearComparison(result);
    if(job.mode!=2U||!job.measurementComplete||!result.baselineMatched||result.cancelled||
        !result.comparisonValid||(!result.motionVerified&&!result.idleVerified)||
        !std::isfinite(job.capture.pulsesPerMm)||job.capture.pulsesPerMm<=0.0) return;
    const double positionTolerance=1.0/job.capture.pulsesPerMm;
    if(!std::isfinite(positionTolerance)||positionTolerance<=0.0) return;
    const auto& before=result.baselineMetrics;const auto& after=result.analysis;
    if(result.motionVerified) {
        const double b[]={before.peakErrorMm,before.rmsMovingErrorMm,before.maxOvershootMm,before.maxSettlingSeconds};
        const double a[]={after.peakErrorMm,after.rmsMovingErrorMm,after.maxOvershootMm,after.maxSettlingSeconds};
        const double t[]={positionTolerance,positionTolerance,positionTolerance,CapturePeriodSeconds};
        result.motionComparison=CompareMetrics(b,a,t,4U);
    }
    if(result.idleVerified) {
        const double b[]={before.idleTailRmsMm,before.idlePeakBiasMm,before.idleJitterMm};
        const double a[]={after.idleTailRmsMm,after.idlePeakBiasMm,after.idleJitterMm};
        const double t[]={positionTolerance,positionTolerance,positionTolerance};
        result.idleComparison=CompareMetrics(b,a,t,3U);
    }
    if(result.motionComparison==PerformanceComparison::Unavailable&&
        result.idleComparison==PerformanceComparison::Unavailable) return;
    result.comparisonPositionTolerance=positionTolerance;
    result.comparisonTimeToleranceSeconds=CapturePeriodSeconds;
    if(!result.verified||result.motionComparison==PerformanceComparison::Unavailable||
        (before.idleCandidateValid&&result.idleComparison==PerformanceComparison::Unavailable)) return;
    const bool improved=result.motionComparison==PerformanceComparison::Improved||
        (before.idleCandidateValid&&result.idleComparison==PerformanceComparison::Improved);
    const bool regressed=result.motionComparison==PerformanceComparison::Regressed||
        (before.idleCandidateValid&&result.idleComparison==PerformanceComparison::Regressed);
    const bool mixed=result.motionComparison==PerformanceComparison::Mixed||
        (before.idleCandidateValid&&result.idleComparison==PerformanceComparison::Mixed);
    result.adoptionAdvice=mixed||(improved&&regressed)?AdoptionAdvice::ReviewTradeoff:
        improved?AdoptionAdvice::RecommendCandidate:AdoptionAdvice::KeepOriginal;
}
std::string BaselinePath(const JobRequest& job) {
    // Keep the established path: a new P1 replaces/revokes the one axis record.
    // The header and identity version reject pre-TUNE03 records.
    return Join(job.outputDir,"TUNE01_"+job.config.machineId+"_A"+std::to_string(job.capture.axis)+".baseline");
}
bool SaveBaseline(const JobRequest& job,const AnalysisResult& result) {
    const bool rangeEnabled=job.config.axis[job.capture.axis].rangeEnabled;
    const bool pointToPoint=job.capture.pointToPoint;
    if(pointToPoint&&!rangeEnabled)return false;
    double originNative=0.0;
    if(rangeEnabled) {
        if(!std::isfinite(job.capture.pulsesPerMm)||job.capture.pulsesPerMm<=0.0||
            !std::isfinite(job.capture.originPulse))return false;
        originNative=job.capture.originPulse/job.capture.pulsesPerMm;
        if(!std::isfinite(originNative))return false;
    }
    const double spanNative=pointToPoint?job.capture.excursionPulse/job.capture.pulsesPerMm:0.0;
    if(pointToPoint&&(!std::isfinite(spanNative)||spanNative<=0.0||
        !std::isfinite(originNative+spanNative)||originNative+spanNative<=originNative))return false;
    std::ostringstream payload;payload.imbue(std::locale::classic());payload<<std::setprecision(17);
    payload<<(pointToPoint?"TUNE06_BASELINE_V5\n":rangeEnabled?"TUNE05_BASELINE_V4\n":"TUNE03_BASELINE_V3\n")
        <<job.config.machineId<<' '<<static_cast<unsigned>(job.config.machineProfile)
        <<' '<<job.capture.axis<<' '<<job.capture.axisType<<' '<<job.configIdentityHash<<' '<<job.capture.pulsesPerMm<<'\n';
    // Axis type/native unit and PPU remain in the checksummed identity line.
    // Range tests also bind their machine-coordinate (unwrapped rotary) start.
    if(rangeEnabled)payload<<"ORIGIN_NATIVE "<<originNative<<'\n';
    if(pointToPoint)payload<<"POINT_TO_POINT_SPAN_NATIVE "<<spanNative<<'\n';
    WriteGains(payload,result.measured);WriteGains(payload,result.suggested);
    WriteGains(payload,result.measuredIdle);
    payload<<(result.idleCandidateValid?1U:0U)<<'\n';
    WriteGains(payload,result.idleCandidateValid?result.suggestedIdle:CaptureGains{});
    payload<<result.peakErrorMm<<' '<<result.rmsMovingErrorMm<<' '<<result.rmsIdleErrorMm<<' '
        <<result.maxOvershootMm<<' '<<result.maxSettlingSeconds<<' '<<result.maxSettledJitterMm<<'\n';
    payload<<(result.idleMeasurementsValid?1U:0U)<<' '<<(result.idleQualityPassed?1U:0U)<<' '
        <<result.idleTailRmsMm<<' '<<result.idlePeakBiasMm<<' '<<result.idleJitterMm<<' '<<result.idleObservedSeconds<<' '<<result.idleRequiredObservationSeconds<<'\n';
    const std::string bytes=payload.str();
    return WriteChecked(BaselinePath(job),bytes+"CHECKSUM "+std::to_string(Hash(bytes))+"\n");
}
bool LoadBaseline(const JobRequest& job,JobResult& result) {
    ClearComparison(result);
    result.baselineAvailable=result.baselineMatched=result.candidateMatched=false;
    result.comparisonValid=result.verified=result.idleCandidateMatched=false;
    result.motionVerified=result.idleVerified=false;
    result.motionContextCompatible=result.idleContextCompatible=false;
    result.baselineMetrics=AnalysisResult{};
    result.baselineOriginChecked=result.baselineOriginMatched=false;
    result.baselineOriginNative=result.currentOriginNative=result.baselineOriginTolerance=0.0;
    std::ifstream file(BaselinePath(job),std::ios::binary);if(!file) return false;
    std::string bytes; char block[1024];
    while(file) {file.read(block,sizeof(block));bytes.append(block,static_cast<std::size_t>(file.gcount()));if(bytes.size()>8192U)return false;}
    if(!file.eof())return false;
    const auto at=bytes.rfind("CHECKSUM ");if(at==std::string::npos)return false;
    const std::string payload=bytes.substr(0,at);
    std::istringstream check(bytes.substr(at+9));std::uint64_t hash=0;
    if(!(check>>hash))return false;
    check>>std::ws;
    if(!check.eof()||hash!=Hash(payload))return false;
    std::istringstream in(payload);in.imbue(std::locale::classic());
    std::string magic,machine;unsigned profile=0,type=0;int axis=-1;std::uint64_t identity=0;double ppu=0;
    if(!(in>>magic>>machine>>profile>>axis>>type>>identity>>ppu)||
        (magic!="TUNE03_BASELINE_V3"&&magic!="TUNE05_BASELINE_V4"&&magic!="TUNE06_BASELINE_V5"))return false;
    const bool pointBound=magic=="TUNE06_BASELINE_V5";
    const bool originBound=magic=="TUNE05_BASELINE_V4"||pointBound;
    double originNative=0.0,spanNative=0.0;
    if(originBound) {
        std::string originTag;
        if(!(in>>originTag>>originNative)||originTag!="ORIGIN_NATIVE"||
            !std::isfinite(originNative)||!std::isfinite(ppu)||ppu<=0.0)return false;
    }
    if(pointBound) {
        std::string spanTag;
        if(!(in>>spanTag>>spanNative)||spanTag!="POINT_TO_POINT_SPAN_NATIVE"||
            !std::isfinite(spanNative)||spanNative<=0.0||
            !std::isfinite(originNative+spanNative)||originNative+spanNative<=originNative)return false;
    }
    AnalysisResult baseline{};unsigned idleCandidate=0,idleMeasured=0,idleQuality=0;
    if(!ReadGains(in,baseline.measured)||!ReadGains(in,baseline.suggested)||
        !ReadGains(in,baseline.measuredIdle)||!(in>>idleCandidate)||idleCandidate>1U||
        !ReadGains(in,baseline.suggestedIdle)||!SupportedMotionGains(baseline.measured)||
        !SupportedMotionGains(baseline.suggested)||baseline.measuredIdle.kp<=0.0||
        (idleCandidate&&!SupportedIdleCandidate(baseline.suggestedIdle)))return false;
    double* metrics[]={&baseline.peakErrorMm,&baseline.rmsMovingErrorMm,&baseline.rmsIdleErrorMm,
        &baseline.maxOvershootMm,&baseline.maxSettlingSeconds,&baseline.maxSettledJitterMm};
    for(double* metric:metrics)if(!(in>>*metric)||!std::isfinite(*metric)||*metric<0.0)return false;
    if(!(in>>idleMeasured>>idleQuality)||idleMeasured>1U||idleQuality>1U)return false;
    double* idleMetrics[]={&baseline.idleTailRmsMm,&baseline.idlePeakBiasMm,&baseline.idleJitterMm,&baseline.idleObservedSeconds,&baseline.idleRequiredObservationSeconds};
    for(double* metric:idleMetrics)if(!(in>>*metric)||!std::isfinite(*metric)||*metric<0.0)return false;
    if((idleQuality&&!idleMeasured)||(idleCandidate&&(!idleQuality||!idleMeasured||
        baseline.idleRequiredObservationSeconds<=0.0||baseline.idleObservedSeconds+1e-9<baseline.idleRequiredObservationSeconds)))return false;
    in>>std::ws;if(!in.eof())return false;
    result.baselineAvailable=true;
    const auto& profileConfig=job.config.axis[job.capture.axis];
    result.baselineMatched=machine==job.config.machineId&&profile==static_cast<unsigned>(job.config.machineProfile)&&
        axis==job.capture.axis&&type==job.capture.axisType&&identity==job.configIdentityHash&&Near(ppu,job.capture.pulsesPerMm)&&
        originBound==profileConfig.rangeEnabled&&pointBound==job.capture.pointToPoint&&
        (!pointBound||Near(spanNative,job.capture.excursionPulse/job.capture.pulsesPerMm));
    if(!result.baselineMatched)return true;
    if(originBound) {
        result.baselineOriginChecked=true;result.baselineOriginNative=originNative;
        result.currentOriginNative=job.capture.originPulse/job.capture.pulsesPerMm;
        // Q1 binds a deterministic commanded endpoint A; unlike a manually
        // selected legacy origin it does not use the physical settling band
        // as permission to compare a different start target.
        result.baselineOriginTolerance=pointBound?1.0/job.capture.pulsesPerMm:
            (std::max)(1.0/job.capture.pulsesPerMm,profileConfig.settleWindowMm);
        result.baselineOriginMatched=std::isfinite(result.currentOriginNative)&&
            std::isfinite(profileConfig.settleWindowMm)&&profileConfig.settleWindowMm>0.0&&
            std::isfinite(result.baselineOriginTolerance)&&result.baselineOriginTolerance>0.0&&
            std::fabs(result.currentOriginNative-originNative)<=result.baselineOriginTolerance;
        if(!result.baselineOriginMatched){result.baselineMatched=false;return true;}
    }
    baseline.measurementsValid=baseline.qualityPassed=baseline.candidateValid=true;
    baseline.idleCandidateValid=idleCandidate!=0U;baseline.idleMeasurementsValid=idleMeasured!=0U;
    baseline.idleQualityPassed=idleQuality!=0U;
    result.baselineMetrics=baseline;
    result.candidateMatched=SameGains(job.capture.motionGains,baseline.suggested);
    result.idleCandidateMatched=baseline.idleCandidateValid&&SameGains(job.capture.idleGains,baseline.suggestedIdle);
    // Each bank can be adopted separately. Its peer must still be one of the
    // original measured or recommended banks, not an unrelated parameter edit.
    result.motionContextCompatible=SameGains(job.capture.motionGains,baseline.measured)||result.candidateMatched;
    result.idleContextCompatible=SameGains(job.capture.idleGains,baseline.measuredIdle)||result.idleCandidateMatched;
    result.comparisonValid=result.analysis.measurementsValid;
    result.motionVerified=result.candidateMatched&&result.idleContextCompatible&&job.measurementComplete&&
        result.analysis.measurementsValid&&result.analysis.qualityPassed;
    // A user-specified cruise goal is an additional measured acceptance
    // requirement, not a replacement for endpoint/saturation/model checks.
    const auto cruise=EvaluateCruiseTracking(profileConfig,job.capture,job.statistics,job.measurementComplete);
    if(profileConfig.cruiseLagTargetMm>0.0)
        result.motionVerified=result.motionVerified&&cruise.available&&cruise.targetMet;
    result.idleVerified=result.idleCandidateMatched&&result.motionContextCompatible&&job.measurementComplete&&
        result.analysis.measurementsValid&&result.analysis.idleMeasurementsValid&&result.analysis.idleQualityPassed&&
        std::isfinite(result.analysis.idleRequiredObservationSeconds)&&result.analysis.idleRequiredObservationSeconds>0.0&&
        result.analysis.idleObservedSeconds+1e-9>=result.analysis.idleRequiredObservationSeconds;
    // Overall verification requires every available recommendation, while the
    // per-bank flags expose partial application honestly. All flags are scoped
    // to this measured envelope, not all future loads and speeds.
    result.verified=result.motionVerified&&(!baseline.idleCandidateValid||result.idleVerified);
    EvaluateComparison(job,result);
    return true;
}
PerformanceComparison CompareCruiseMetrics(const double* before,const double* after,
    double physicalTolerance,std::size_t count) noexcept {
    if(!std::isfinite(physicalTolerance)||physicalTolerance<=0.0)return PerformanceComparison::Unavailable;
    bool lower=false,higher=false;
    for(std::size_t i=0U;i<count;++i) {
        if(!std::isfinite(before[i])||before[i]<0.0||!std::isfinite(after[i])||after[i]<0.0)
            return PerformanceComparison::Unavailable;
        const double roundoff=32.0*(std::numeric_limits<double>::epsilon)()*
            (std::max)(before[i],after[i]);
        const double decrease=before[i]-after[i],increase=after[i]-before[i];
        // The physical one-pulse boundary itself has floating-point error.
        // Subtract the boundary before the tiny roundoff guard, so a nominal
        // exactly-one-pulse difference remains Equivalent in both runtimes.
        lower=lower||(decrease>physicalTolerance&&decrease-physicalTolerance>roundoff);
        higher=higher||(increase>physicalTolerance&&increase-physicalTolerance>roundoff);
    }
    return lower?(higher?PerformanceComparison::Mixed:PerformanceComparison::Improved):
        higher?PerformanceComparison::Regressed:PerformanceComparison::Equivalent;
}
std::string CruiseBaselinePath(const JobRequest& job) {
    return Join(job.outputDir,"TUNE01_"+job.config.machineId+"_A"+
        std::to_string(job.capture.axis)+"_CRUISE.baseline");
}
void ClearCruiseComparison(JobResult& result,const char* reason) {
    result.cruiseComparisonAvailable=result.cruiseP2Verified=false;
    result.cruisePositiveComparison=result.cruiseNegativeComparison=
        result.cruiseOverallComparison=PerformanceComparison::Unavailable;
    result.cruiseComparisonTolerance=0.0;
    result.cruiseComparisonReason=reason;
}
bool ValidCruiseGains(const CaptureGains& g) noexcept {
    return std::isfinite(g.kp)&&g.kp>0.0&&std::isfinite(g.ki)&&g.ki>=0.0&&
        std::isfinite(g.kd)&&g.kd>=0.0&&std::isfinite(g.kvff)&&g.kvff>=0.0;
}
bool ValidCruiseMetrics(const CruiseTrackingMetrics& m,unsigned expected) noexcept {
    const double tolerance=(std::max)(1.0,m.peakAbsoluteMm)*1e-12;
    return expected>=2U&&expected<=64U&&(expected%2U)==0U&&
        m.sampleCount>0ULL&&m.sampleCount<=14400000ULL&&m.qualifiedLegCount==expected&&
        std::isfinite(m.observedSeconds)&&m.observedSeconds+1e-9>=expected*0.05&&
        m.observedSeconds<=static_cast<double>(m.sampleCount)*0.001+1e-9&&
        std::isfinite(m.peakAbsoluteMm)&&m.peakAbsoluteMm>=0.0&&
        std::isfinite(m.rmsMm)&&m.rmsMm>=0.0&&m.rmsMm<=m.peakAbsoluteMm+tolerance&&
        std::isfinite(m.meanMm)&&std::fabs(m.meanMm)<=m.rmsMm+tolerance;
}
void WriteCruiseMetrics(std::ostream& out,const CruiseTrackingMetrics& m) {
    out<<m.sampleCount<<' '<<m.qualifiedLegCount<<' '<<m.observedSeconds<<' '
        <<m.peakAbsoluteMm<<' '<<m.rmsMm<<' '<<m.meanMm<<'\n';
}
bool ReadCruiseMetrics(std::istream& in,CruiseTrackingMetrics& m,unsigned expected) {
    return static_cast<bool>(in>>m.sampleCount>>m.qualifiedLegCount>>m.observedSeconds>>
        m.peakAbsoluteMm>>m.rmsMm>>m.meanMm)&&ValidCruiseMetrics(m,expected);
}
bool CruiseIdentityValid(const JobRequest& job) noexcept {
    const auto& r=job.capture;
    const double origin=r.originPulse/r.pulsesPerMm,span=r.excursionPulse/r.pulsesPerMm;
    return r.axis>=0&&r.axis<static_cast<int>(kAxisCount)&&r.axisType<=1U&&
        r.sampleStride>0U&&r.sampleStride<=64U&&
        std::isfinite(r.pulsesPerMm)&&r.pulsesPerMm>0.0&&
        std::isfinite(origin)&&std::isfinite(span)&&span>0.0&&
        std::isfinite(origin+span)&&origin+span>origin&&
        ValidCruiseGains(r.motionGains)&&ValidCruiseGains(r.idleGains);
}
bool CruiseCurrentAvailable(const JobRequest& job,const JobResult& result,
    const CruiseTrackingAssessment& cruise) noexcept {
    return job.measurementComplete&&!result.cancelled&&result.analysis.measurementsValid&&
        cruise.available&&CruiseIdentityValid(job);
}
bool SaveCruiseBaseline(const JobRequest& job,JobResult& result) {
    const auto cruise=EvaluateCruiseTracking(job.config.axis[job.capture.axis],job.capture,
        job.statistics,job.measurementComplete);
    if(!CruiseCurrentAvailable(job,result,cruise)||
        !ValidCruiseMetrics(cruise.positive,cruise.expectedLegsPerDirection)||
        !ValidCruiseMetrics(cruise.negative,cruise.expectedLegsPerDirection))return false;
    std::ostringstream payload;payload.imbue(std::locale::classic());payload<<std::setprecision(17);
    payload<<"TUNE16_CRUISE_BASELINE_V1\n"<<job.config.machineId<<' '
        <<static_cast<unsigned>(job.config.machineProfile)<<' '<<job.capture.axis<<' '
        <<job.capture.axisType<<' '<<job.configIdentityHash<<' '<<job.capture.pulsesPerMm<<' '
        <<job.session<<' '<<static_cast<unsigned>(job.capture.pointToPoint)<<' '
        <<job.capture.sampleStride<<' '<<job.capture.originPulse/job.capture.pulsesPerMm<<' '
        <<job.capture.excursionPulse/job.capture.pulsesPerMm<<' '
        <<cruise.expectedLegsPerDirection<<'\n';
    WriteGains(payload,job.capture.motionGains);WriteGains(payload,job.capture.idleGains);
    WriteCruiseMetrics(payload,cruise.positive);WriteCruiseMetrics(payload,cruise.negative);
    const auto bytes=payload.str();
    if(!WriteChecked(CruiseBaselinePath(job),bytes+"CHECKSUM "+std::to_string(Hash(bytes))+"\n"))return false;
    result.cruiseBaselineSession=job.session;return true;
}
void LoadCruiseBaseline(const JobRequest& job,JobResult& result) {
    ClearCruiseComparison(result,"BASELINE_MISSING");
    result.cruiseBaselineSaved=result.cruiseBaselineAvailable=result.cruiseBaselineMatched=false;
    result.cruiseBaselineSession=0ULL;
    result.cruiseBeforePositive=CruiseTrackingMetrics{};result.cruiseBeforeNegative=CruiseTrackingMetrics{};
    result.cruiseBeforeMotionGains=CaptureGains{};result.cruiseBeforeIdleGains=CaptureGains{};
    std::ifstream file(CruiseBaselinePath(job),std::ios::binary);if(!file)return;
    result.cruiseComparisonReason="BASELINE_INVALID";
    std::string bytes;char block[1024];
    while(file){file.read(block,sizeof(block));bytes.append(block,static_cast<std::size_t>(file.gcount()));if(bytes.size()>8192U)return;}
    if(!file.eof())return;
    const auto at=bytes.rfind("CHECKSUM ");if(at==std::string::npos)return;
    const auto payload=bytes.substr(0,at);std::istringstream check(bytes.substr(at+9));
    check.imbue(std::locale::classic());std::uint64_t hash=0ULL;
    if(!(check>>hash))return;
    check>>std::ws;if(!check.eof()||hash!=Hash(payload))return;
    std::istringstream in(payload);in.imbue(std::locale::classic());
    std::string magic,machine;unsigned profile=0U,type=0U,point=0U,stride=0U,expected=0U;
    int axis=-1;std::uint64_t identity=0ULL,session=0ULL;double ppu=0.0,origin=0.0,span=0.0;
    CaptureGains motion{},idle{};CruiseTrackingMetrics positive{},negative{};
    if(!(in>>magic>>machine>>profile>>axis>>type>>identity>>ppu>>session>>point>>stride>>origin>>span>>expected)||
        magic!="TUNE16_CRUISE_BASELINE_V1"||machine.empty()||machine.size()>64U||
        profile>static_cast<unsigned>(MachineProfile::Machine)||axis<0||axis>=static_cast<int>(kAxisCount)||
        type>1U||!identity||!session||point>1U||!stride||stride>64U||
        !std::isfinite(ppu)||ppu<=0.0||!std::isfinite(origin)||!std::isfinite(span)||span<=0.0||
        !std::isfinite(origin+span)||origin+span<=origin||
        !ReadGains(in,motion)||!ReadGains(in,idle)||!ValidCruiseGains(motion)||!ValidCruiseGains(idle)||
        !ReadCruiseMetrics(in,positive,expected)||!ReadCruiseMetrics(in,negative,expected)||
        positive.sampleCount+negative.sampleCount>14400000ULL)return;
    in>>std::ws;if(!in.eof())return;
    result.cruiseBaselineAvailable=true;result.cruiseBaselineSession=session;
    result.cruiseBeforePositive=positive;result.cruiseBeforeNegative=negative;
    result.cruiseBeforeMotionGains=motion;result.cruiseBeforeIdleGains=idle;
    if(!CruiseIdentityValid(job)) {result.cruiseComparisonReason="CURRENT_CONTEXT_INVALID";return;}
    const double tolerance=1.0/job.capture.pulsesPerMm;
    if(machine!=job.config.machineId||profile!=static_cast<unsigned>(job.config.machineProfile)||
        axis!=job.capture.axis||type!=job.capture.axisType||identity!=job.configIdentityHash||
        !Near(ppu,job.capture.pulsesPerMm)||point!=static_cast<unsigned>(job.capture.pointToPoint)||
        stride!=job.capture.sampleStride||expected!=2U*job.config.axis[job.capture.axis].cycles||
        !std::isfinite(tolerance)||tolerance<=0.0||
        std::fabs(origin-job.capture.originPulse/job.capture.pulsesPerMm)>tolerance||
        std::fabs(span-job.capture.excursionPulse/job.capture.pulsesPerMm)>tolerance) {
        result.cruiseComparisonReason="BASELINE_CONTEXT_MISMATCH";return;
    }
    if(g_cruiseEligible[axis].load(std::memory_order_acquire)!=session||
        g_cruiseRevision[axis].load(std::memory_order_acquire)!=g_cruiseJobRevision) {
        result.cruiseComparisonReason="BASELINE_NOT_ELIGIBLE";return;
    }
    result.cruiseBaselineMatched=true;
    const auto cruise=EvaluateCruiseTracking(job.config.axis[axis],job.capture,job.statistics,job.measurementComplete);
    if(result.cancelled){result.cruiseComparisonReason="CANCELLED";return;}
    if(!job.measurementComplete||!result.analysis.measurementsValid){result.cruiseComparisonReason="CURRENT_MEASUREMENT_INVALID";return;}
    if(!cruise.targetSpecified){result.cruiseComparisonReason="TARGET_NOT_SPECIFIED";return;}
    if(!CruiseCurrentAvailable(job,result,cruise)){result.cruiseComparisonReason="CURRENT_CRUISE_UNAVAILABLE";return;}
    const double before[]={positive.peakAbsoluteMm,positive.rmsMm,negative.peakAbsoluteMm,negative.rmsMm};
    const double after[]={cruise.positive.peakAbsoluteMm,cruise.positive.rmsMm,cruise.negative.peakAbsoluteMm,cruise.negative.rmsMm};
    result.cruisePositiveComparison=CompareCruiseMetrics(before,after,tolerance,2U);
    result.cruiseNegativeComparison=CompareCruiseMetrics(before+2,after+2,tolerance,2U);
    result.cruiseOverallComparison=CompareCruiseMetrics(before,after,tolerance,4U);
    if(result.cruiseOverallComparison==PerformanceComparison::Unavailable)return;
    result.cruiseComparisonTolerance=tolerance;result.cruiseComparisonAvailable=true;
    result.cruiseP2Verified=cruise.targetMet;
    result.cruiseComparisonReason=cruise.targetMet?"COMPARISON_AVAILABLE_TARGET_MET":"COMPARISON_AVAILABLE_TARGET_NOT_MET";
}
bool SaveRaw(const JobRequest& job,const std::string& path,const std::string& captureToken) {
    std::ofstream out(path,std::ios::binary|std::ios::trunc);out.imbue(std::locale::classic());out<<std::setprecision(17);
    if(!out)return false;
    out<<"session,axis,cycle,us,command_pulse,actual_pulse,reference_pps,actual_pps,proposal_pps,image_pps,P,I,FF,Kp,Ki,Kd,Kvff,integral,motion_state,gain_bank,valid,saturated,capture_token\n";
    for(std::size_t i=0;i<g_count;++i) {
        if((i&1023U)==0U&&g_cancel.load(std::memory_order_acquire)) {out.close();return false;}
        const auto& s=g_samples[i];
        out<<s.session<<','<<job.capture.axis<<','<<s.cycle<<','<<s.timestampUs<<','<<s.commandPulse<<','<<s.actualPulse
            <<','<<s.referencePps<<','<<s.actualVelocityPps<<','<<s.proposedLogicalPps<<','<<s.outputLogicalPps
            <<','<<s.pTerm<<','<<s.iTerm<<','<<s.feedforwardPps<<','<<s.kp<<','<<s.ki<<','<<s.kd<<','<<s.kvff
            <<','<<s.integral<<','<<s.motionState<<','<<s.gainBank<<','<<s.valid<<','<<s.saturated
            <<','<<captureToken<<'\n';
    }
    out.flush();const bool good=out.good();out.close();
    return good&&!out.fail();
}
std::string Summary(const JobRequest& job,const JobResult& result,
    const std::string& rawFile,const std::string& captureToken) {
    const auto& a=result.analysis;const auto& profile=job.config.axis[job.capture.axis];
    const auto target=EvaluatePositionTarget(profile,a,job.measurementComplete);
    const auto cruise=EvaluateCruiseTracking(profile,job.capture,job.statistics,job.measurementComplete);
    const auto& stats=job.statistics;
    bool statisticsValid=stats.valid&&stats.session==job.session&&stats.sampleCount>0ULL&&
        stats.firstCycle>0ULL&&stats.lastCycle>=stats.firstCycle&&
        stats.lastCycle-stats.firstCycle==stats.sampleCount-1ULL&&
        stats.firstTimestampUs>0ULL&&stats.lastTimestampUs>=stats.firstTimestampUs&&
        stats.movingSampleCount<=stats.sampleCount&&
        std::isfinite(job.capture.pulsesPerMm)&&job.capture.pulsesPerMm>0.0&&
        std::isfinite(stats.peakFollowingErrorPulse)&&stats.peakFollowingErrorPulse>=0.0&&
        std::isfinite(stats.meanSquaredFollowingErrorPulse)&&stats.meanSquaredFollowingErrorPulse>=0.0&&
        std::isfinite(stats.meanSquaredMovingFollowingErrorPulse)&&stats.meanSquaredMovingFollowingErrorPulse>=0.0&&
        std::isfinite(stats.peakReferencePps)&&stats.peakReferencePps>=0.0;
    double fullRatePeak=statisticsValid?stats.peakFollowingErrorPulse/job.capture.pulsesPerMm:0.0;
    double fullRateRms=statisticsValid?std::sqrt(stats.meanSquaredFollowingErrorPulse)/job.capture.pulsesPerMm:0.0;
    double fullRateMovingRms=statisticsValid&&stats.movingSampleCount?
        std::sqrt(stats.meanSquaredMovingFollowingErrorPulse)/job.capture.pulsesPerMm:0.0;
    if(!std::isfinite(fullRatePeak)||!std::isfinite(fullRateRms)||!std::isfinite(fullRateMovingRms)) {
        statisticsValid=false;fullRatePeak=fullRateRms=fullRateMovingRms=0.0;
    }
    const bool statisticsComplete=statisticsValid&&stats.complete&&job.measurementComplete;
    const bool fullRateTrackingQuality=statisticsComplete&&a.measurementsValid&&stats.movingSampleCount>0ULL&&
        !stats.anySaturated&&fullRatePeak<=profile.followingLimitMm&&fullRateMovingRms<=profile.maxRmsErrorMm;
    const bool originMismatch=result.baselineOriginChecked&&!result.baselineOriginMatched;
    const std::string failureReason=!job.failureReason.empty()?job.failureReason:
        originMismatch?"BASELINE_ORIGIN_MISMATCH":"";
    std::ostringstream out;out.imbue(std::locale::classic());out<<std::setprecision(17);
    out<<"TUNE10 - G00 / IDLE gain estimate / comparison result\n"
        <<"ReportVersion=4\nMachineId="<<job.config.machineId<<"\nProfile="<<(job.config.machineProfile==MachineProfile::Bench?"BENCH":"MACHINE")
        <<"\nAxis="<<job.capture.axis<<"\nAxisType="<<job.capture.axisType
        <<"\nPositionUnit="<<(job.capture.axisType==1U?"deg":"mm")
        <<"\nPulsesPerUnit="<<job.capture.pulsesPerMm<<"\nMode="<<job.mode<<"\nSession="<<job.session
        <<"\nCaptureToken="<<captureToken<<"\nRawFile="<<rawFile<<"\nSampleCount="<<g_count
        <<"\nSampleStride="<<job.capture.sampleStride
        <<"\nCaptureMode="<<(job.capture.sampleStride>1U?"FULL_STROKE_METRICS_ONLY":"FULL_RATE")
        <<"\nMetricSource="<<(job.capture.sampleStride>1U?"FULL_RATE_MOTION_BANK":"SEGMENT_FULL_RATE")
        <<"\nRawSamplePeriodSeconds="<<CapturePeriodSeconds
        <<"\nFullRateStatisticsValid="<<statisticsValid
        <<"\nFullRateStatisticsComplete="<<statisticsComplete
        <<"\nFullRateAnySaturated="<<(statisticsValid&&stats.anySaturated)
        <<"\nFullRateSampleCount="<<(statisticsValid?stats.sampleCount:0ULL)
        <<"\nFullRateMovingSampleCount="<<(statisticsValid?stats.movingSampleCount:0ULL)
        <<"\nFullRateFirstCycle="<<(statisticsValid?stats.firstCycle:0ULL)
        <<"\nFullRateLastCycle="<<(statisticsValid?stats.lastCycle:0ULL)
        <<"\nFullRateFirstTimestampUs="<<(statisticsValid?stats.firstTimestampUs:0ULL)
        <<"\nFullRateLastTimestampUs="<<(statisticsValid?stats.lastTimestampUs:0ULL)
        <<"\nFullRateTrackingPeakMm="<<fullRatePeak
        <<"\nFullRateTrackingRmsMm="<<fullRateRms
        <<"\nFullRateMovingRmsMm="<<fullRateMovingRms
        <<"\nFullRateTrackingQualityPassed="<<fullRateTrackingQuality
        <<"\nPulsesPerMm="<<job.capture.pulsesPerMm<<"\nOriginPulse="<<job.capture.originPulse
        <<"\nAxisEnabled="<<profile.enabled<<"\nExcursionMm="<<profile.excursionMm
        <<"\nRangeEnabled="<<profile.rangeEnabled<<"\nRangeMinMCS="<<profile.rangeMinMCS
        <<"\nRangeMaxMCS="<<profile.rangeMaxMCS<<"\nEndPadding="<<profile.endPadding
        <<"\nSpeedMmMin="<<profile.speedMmMin<<"\nAccelerationSeconds="<<profile.accelerationSeconds
        <<"\nDecelerationSeconds="<<profile.decelerationSeconds<<"\nDwellSeconds="<<profile.dwellSeconds
        <<"\nCycles="<<profile.cycles<<"\nFollowingLimitMm="<<profile.followingLimitMm
        <<"\nSettleWindowMm="<<profile.settleWindowMm<<"\nSettleHoldSeconds="<<profile.settleHoldSeconds
        <<"\nSettleTimeoutSeconds="<<profile.settleTimeoutSeconds<<"\nSettleVelocityPps="<<job.capture.settleVelocityPps
        <<"\nMaxOvershootMm="<<profile.maxOvershootMm<<"\nMaxRmsErrorMm="<<profile.maxRmsErrorMm
        <<"\nKpMin="<<profile.kpMin<<"\nKpMax="<<profile.kpMax
        <<"\nKvffMin="<<profile.kvffMin<<"\nKvffMax="<<profile.kvffMax
        <<"\nMaxDurationSeconds="<<job.config.maxDurationSeconds
        <<"\nTrajectoryMode="<<(job.capture.pointToPoint?"POINT_TO_POINT":"SYMMETRIC")
        <<"\nEffectiveExcursionNative="<<(job.capture.excursionPulse/job.capture.pulsesPerMm)
        <<"\nEffectiveMaxDurationSeconds="<<(job.capture.maximumTicks?
            static_cast<double>(job.capture.maximumTicks)*CapturePeriodSeconds:
            job.capture.pointToPoint?job.config.twoPointMaxDurationSeconds:job.config.maxDurationSeconds)
        <<"\nTwoPointMaxDurationSeconds="<<job.config.twoPointMaxDurationSeconds
        <<"\nAutoApproachUsed="<<job.autoApproachUsed
        <<"\nApproachStartNative="<<job.approachStartNative
        <<"\nApproachTargetNative="<<job.approachTargetNative
        <<"\nApproachSeconds="<<job.approachSeconds
        <<"\nMeasurementStartCycle="<<job.measurementStartCycle
        <<"\nLegacyExcursionAndPaddingApplied="<<(!job.capture.pointToPoint)
        <<"\nModelFitMinimum="<<job.config.modelFitMinimum
        <<"\nMinimumVelocityMmS="<<job.config.minimumVelocityMmS
        <<"\nCandidateResponseFactor="<<job.config.candidateResponseFactor
        <<"\nMaxKpIncreaseRatio="<<job.config.maxKpIncreaseRatio
        <<"\nTuningStrategy="<<profile.tuningStrategy
        <<"\nTuningStrategyName="<<TuningStrategyName(profile.tuningStrategy)
        <<"\nTargetPositionMm="<<profile.targetPositionMm
        <<"\nEffectiveCandidateResponseFactor="<<EffectiveCandidateResponseFactor(job.config,profile)
        <<"\nEffectiveMaxKpIncreaseRatio="<<EffectiveMaxKpIncreaseRatio(job.config,profile)
        <<"\nTargetAssessmentAvailable="<<target.available
        <<"\nTargetMet="<<target.met
        <<"\nTargetErrorUpperBoundMm="<<target.errorUpperBoundMm
        <<"\nMeasurementComplete="<<job.measurementComplete<<"\nFailureReason="<<failureReason
        <<"\nIdentity="<<job.configIdentityHash<<"\nMeasurementsValid="<<a.measurementsValid
        <<"\nQualityPassed="<<a.qualityPassed
        <<"\nCandidateValid="<<(job.mode==1U?a.candidateValid:result.baselineMatched)
        <<"\nAnalysisReason="<<AnalysisReasonName(a.reason)<<"\nVerifiedInThisTest="<<result.verified
        <<"\nBaselineMatched="<<result.baselineMatched<<"\nCandidateMatched="<<result.candidateMatched
        <<"\nBaselineSaved="<<result.baselineSaved<<"\nRawSaved="<<result.rawSaved
        <<"\nPeakErrorMm="<<a.peakErrorMm<<"\nMovingRmsMm="<<a.rmsMovingErrorMm<<"\nIdleRmsMm="<<a.rmsIdleErrorMm
        <<"\nOvershootMm="<<a.maxOvershootMm<<"\nSettlingSeconds="<<a.maxSettlingSeconds
        <<"\nSettledJitterMm="<<a.maxSettledJitterMm<<"\nModelFit="<<a.trainingFit
        <<"\nValidationFit="<<a.validationFit<<"\nMeasuredKp="<<a.measured.kp<<"\nMeasuredKi="<<a.measured.ki
        <<"\nMeasuredKd="<<a.measured.kd<<"\nMeasuredKvff="<<a.measured.kvff<<'\n';
    // Persist enough information to replay the raw capture without guessing
    // segment boundaries, and explain bandwidth rejection independently of
    // the whole-record model fit. Negative error/ratio means not evaluated.
    out<<"AnalysisRevision=TUNE16\nModelValid="<<a.modelValid
        <<"\nAnalysisDetail="<<AnalysisDetailName(a.analysisDetail)
        <<"\nAnalysisIntervalSeconds="<<a.analysisIntervalSeconds
        <<"\nResolutionFallbackUsed="<<a.resolutionFallbackUsed
        <<"\nResolutionPhaseModels="<<a.resolutionPhaseModels
        <<"\nResolutionGainMismatch="<<a.resolutionGainMismatch
        <<"\nResolutionResponseMismatch="<<a.resolutionResponseMismatch
        <<"\nSelectedTrainingKp="<<a.selectedTrainingKp
        <<"\nMeasuredPeakSpeedNativePerMin="<<a.measuredPeakSpeedNativePerMin
        <<"\nPlantGain="<<a.plantGain<<"\nTauSeconds="<<a.tauSeconds<<"\nDelaySeconds="<<a.delaySeconds
        <<"\nRequestedExcitationHz="<<a.requestedExcitationHz<<"\nExcitationHz="<<a.excitationHz
        <<"\nBandwidthReductions="<<a.bandwidthReductions
        <<"\nTrainingExcitationRatio="<<a.trainingExcitationRatio
        <<"\nValidationExcitationRatio="<<a.validationExcitationRatio
        <<"\nTrainingFrequencyError="<<a.trainingFrequencyError
        <<"\nValidationFrequencyError="<<a.validationFrequencyError
        <<"\nFrequencyRepeatError="<<a.frequencyRepeatError
        <<"\nSegmentCount="<<job.segments.size()
        <<"\nSegmentColumns=FirstCycle,LastCycle,CommandCompleteCycle,StartPulse,TargetPulse\n";
    out<<"CruiseMetricVersion="<<CruiseQualificationVersion
        <<"\nCruiseLagTargetMm="<<profile.cruiseLagTargetMm
        <<"\nCruiseRequestedSpeedNativePerMin="<<profile.speedMmMin
        <<"\nCruiseStatisticsValid="<<cruise.statisticsValid
        <<"\nCruiseStatisticsComplete="<<cruise.complete
        <<"\nCruiseTargetSpecified="<<cruise.targetSpecified
        <<"\nCruiseAssessmentAvailable="<<cruise.available
        <<"\nCruiseTargetMet="<<cruise.targetMet
        <<"\nCruiseExpectedLegsPerDirection="<<cruise.expectedLegsPerDirection
        <<"\nCruisePeakAbsMm="<<cruise.peakAbsoluteMm
        <<"\nCruisePlateauWarmupSeconds="<<static_cast<double>(CruiseStableWindowUs)*1e-6
        <<"\nCruiseMinimumObservedSeconds="<<static_cast<double>(CruiseMinimumObservedUs)*1e-6
        <<"\nCruiseSpeedToleranceRatio="<<CruiseTargetRelativeTolerance
        <<"\nCruiseStableToleranceRatio="<<CruiseStableRelativeTolerance<<'\n';
    const auto writeTracking=[&out](const char* prefix,const CruiseTrackingMetrics& metrics) {
        out<<prefix<<"SampleCount="<<metrics.sampleCount
            <<'\n'<<prefix<<"QualifiedLegCount="<<metrics.qualifiedLegCount
            <<'\n'<<prefix<<"ObservedSeconds="<<metrics.observedSeconds
            <<'\n'<<prefix<<"PeakAbsMm="<<metrics.peakAbsoluteMm
            <<'\n'<<prefix<<"RmsMm="<<metrics.rmsMm
            <<'\n'<<prefix<<"MeanMm="<<metrics.meanMm<<'\n';
    };
    writeTracking("CruisePositive",cruise.positive);
    writeTracking("CruiseNegative",cruise.negative);
    writeTracking("NonCruiseMoving",cruise.nonCruise);
    out<<"CruiseComparisonVersion=1"
        <<"\nCruiseBaselineSaved="<<result.cruiseBaselineSaved
        <<"\nCruiseBaselineAvailable="<<result.cruiseBaselineAvailable
        <<"\nCruiseBaselineMatched="<<result.cruiseBaselineMatched
        <<"\nCruiseBaselineSession="<<result.cruiseBaselineSession
        <<"\nCruiseComparisonAvailable="<<result.cruiseComparisonAvailable
        <<"\nCruiseP2Verified="<<result.cruiseP2Verified
        <<"\nCruiseComparisonReason="<<result.cruiseComparisonReason
        <<"\nCruiseComparisonToleranceMm="<<result.cruiseComparisonTolerance
        <<"\nCruisePositiveComparison="<<static_cast<unsigned>(result.cruisePositiveComparison)
        <<"\nCruiseNegativeComparison="<<static_cast<unsigned>(result.cruiseNegativeComparison)
        <<"\nCruiseOverallComparison="<<static_cast<unsigned>(result.cruiseOverallComparison)<<'\n';
    writeTracking("CruiseBeforePositive",result.cruiseBeforePositive);
    writeTracking("CruiseBeforeNegative",result.cruiseBeforeNegative);
    const auto writeCruiseGains=[&out](const char* prefix,const CaptureGains& gains) {
        out<<prefix<<"Kp="<<gains.kp<<'\n'<<prefix<<"Ki="<<gains.ki
            <<'\n'<<prefix<<"Kd="<<gains.kd<<'\n'<<prefix<<"Kvff="<<gains.kvff<<'\n';
    };
    writeCruiseGains("CruiseBeforeG00",result.cruiseBeforeMotionGains);
    writeCruiseGains("CruiseBeforeIDLE",result.cruiseBeforeIdleGains);
    // Plain diagnostics do not consume the report reader's bounded key count.
    out<<"SteadyPlateauCounts (train+,train-,held+,held-; minimum 64 each): "
        <<a.steadyPlateauCounts[0]<<','<<a.steadyPlateauCounts[1]<<','
        <<a.steadyPlateauCounts[2]<<','<<a.steadyPlateauCounts[3]<<'\n';
    if(result.baselineOriginChecked)
        out<<"BaselineOriginCheck (P1 native, P2 native, tolerance): "
            <<(result.baselineOriginMatched?"MATCH ":"BASELINE_ORIGIN_MISMATCH ")
            <<result.baselineOriginNative<<','<<result.currentOriginNative<<','<<result.baselineOriginTolerance<<'\n';
    for(std::size_t i=0;i<job.segments.size();++i) {
        const auto& segment=job.segments[i];
        // Plain data rows keep the existing HMI's bounded key/value metadata
        // count intact even for 32 cycles (128 legs). No '=' in these rows.
        out<<"Segment"<<i<<": "<<segment.firstCycle<<','<<segment.lastCycle<<','
            <<segment.commandCompleteCycle<<','<<segment.startPulse<<','<<segment.targetPulse<<'\n';
    }
    const bool idleAvailable=job.mode==1U?a.idleCandidateValid:
        result.baselineMatched&&result.baselineMetrics.idleCandidateValid;
    const CaptureGains idleSuggestion=idleAvailable?
        (job.mode==1U?a.suggestedIdle:result.baselineMetrics.suggestedIdle):CaptureGains{};
    out<<"IdleEnabled="<<profile.idleEnabled<<"\nIdleKpMin="<<profile.idleKpMin<<"\nIdleKpMax="<<profile.idleKpMax
        <<"\nIdleKiMin="<<profile.idleKiMin<<"\nIdleKiMax="<<profile.idleKiMax
        <<"\nIdleIntegralTimeSeconds="<<profile.idleIntegralTimeSeconds<<"\nIdleObservationSeconds="<<profile.idleObservationSeconds
        <<"\nIdleMaxRmsErrorMm="<<profile.idleMaxRmsErrorMm<<"\nIdleMaxJitterMm="<<profile.idleMaxJitterMm
        <<"\nIdleMeasurementsValid="<<a.idleMeasurementsValid<<"\nIdleQualityPassed="<<a.idleQualityPassed
        <<"\nIdleCandidateValid="<<idleAvailable<<"\nIdleCandidateMatched="<<result.idleCandidateMatched
        <<"\nMotionVerifiedInThisTest="<<result.motionVerified<<"\nIdleVerifiedInThisTest="<<result.idleVerified
        <<"\nMotionContextCompatible="<<result.motionContextCompatible<<"\nIdleContextCompatible="<<result.idleContextCompatible
        <<"\nIdleAnalysisReason="<<IdleAnalysisReasonName(a.idleReason)<<"\nIdleReasonCode="<<static_cast<unsigned>(a.idleReason)
        <<"\nMeasuredIdleKp="<<job.capture.idleGains.kp<<"\nMeasuredIdleKi="<<job.capture.idleGains.ki
        <<"\nMeasuredIdleKd="<<job.capture.idleGains.kd<<"\nMeasuredIdleKvff="<<job.capture.idleGains.kvff
        <<"\nSuggestedIdleKp="<<idleSuggestion.kp<<"\nSuggestedIdleKi="<<idleSuggestion.ki
        <<"\nSuggestedIdleKd="<<idleSuggestion.kd<<"\nSuggestedIdleKvff="<<idleSuggestion.kvff
        <<"\nIdleTailRmsMm="<<a.idleTailRmsMm<<"\nIdlePeakBiasMm="<<a.idlePeakBiasMm
        <<"\nIdleJitterMm="<<a.idleJitterMm<<"\nIdleObservedSeconds="<<a.idleObservedSeconds
        <<"\nIdleRequiredObservationSeconds="<<a.idleRequiredObservationSeconds<<'\n';
    if(result.comparisonValid&&result.baselineMetrics.idleMeasurementsValid&&a.idleMeasurementsValid)
        out<<"BeforeIdleTailRmsMm="<<result.baselineMetrics.idleTailRmsMm
            <<"\nBeforeIdlePeakBiasMm="<<result.baselineMetrics.idlePeakBiasMm
            <<"\nBeforeIdleJitterMm="<<result.baselineMetrics.idleJitterMm<<'\n';
    if(result.comparisonValid)out<<"BeforePeakErrorMm="<<result.baselineMetrics.peakErrorMm
        <<"\nBeforeMovingRmsMm="<<result.baselineMetrics.rmsMovingErrorMm
        <<"\nBeforeOvershootMm="<<result.baselineMetrics.maxOvershootMm
        <<"\nBeforeSettlingSeconds="<<result.baselineMetrics.maxSettlingSeconds<<'\n';
    out<<"ComparisonVersion=1\nMotionComparison="<<static_cast<unsigned>(result.motionComparison)
        <<"\nIdleComparison="<<static_cast<unsigned>(result.idleComparison)
        <<"\nAdoptionAdvice="<<static_cast<unsigned>(result.adoptionAdvice)
        <<"\nComparisonPositionTolerance="<<result.comparisonPositionTolerance
        <<"\nComparisonTimeToleranceSeconds="<<result.comparisonTimeToleranceSeconds<<'\n';
    if(job.mode==1U&&a.candidateValid) {
        out<<"\nUNVERIFIED CANDIDATE - apply explicitly in HMI, restart, then run P2; core did not change PID.\n";
        out<<job.capture.axis<<"_Kp_G00="<<a.suggested.kp<<'\n'
            <<job.capture.axis<<"_Ki_G00=0\n"<<job.capture.axis<<"_Kd_G00=0\n"
            <<job.capture.axis<<"_Kvff_G00="<<a.suggested.kvff<<'\n';
    }
    if(job.mode==1U&&idleAvailable) {
        out<<job.capture.axis<<"_Kp_IDLE="<<idleSuggestion.kp<<'\n'
            <<job.capture.axis<<"_Ki_IDLE="<<idleSuggestion.ki<<'\n'
            <<job.capture.axis<<"_Kd_IDLE=0\n"<<job.capture.axis<<"_Kvff_IDLE=0\n";
    }
    if(job.mode==2U&&result.baselineMatched) {
        const auto& expected=result.baselineMetrics.suggested;
        out<<"\nExpectedBaselineKp="<<expected.kp<<"\nExpectedBaselineKi="<<expected.ki
            <<"\nExpectedBaselineKd="<<expected.kd<<"\nExpectedBaselineKvff="<<expected.kvff
            <<"\nValidationStatus="<<(result.verified?"VERIFIED_IN_THIS_TEST":"NOT_VERIFIED")<<'\n';
    }
    out<<"\nLegacy model recommendations retain G00 Ki=0 and Kd=0; their IDLE PI remains an unverified model-based candidate. Cruise comparison records the actual measured banks without requiring that candidate.\n"
        <<"Relative comparison applies only to this recorded P1/P2 combination; it is not a global optimum or statistical proof.\n"
        <<"Absolute verification and relative adoption advice are separate. No PID was changed by this analysis.\n"
        <<"image_pps is local PDO image, not drive acknowledgement.\n"
        <<"Motor encoder results do not prove table accuracy.\n";
    out<<"Target assessment uses recorded endpoint IDLE bias plus jitter only; it does not change guards, machine in-position limits or candidate permission.\n";
    if(job.capture.sampleStride>1U)
        out<<"Full-stroke mode records a decimated curve; tracking peak/RMS use every accepted 250 us measurement sample. Moving RMS uses motion-bank samples with nonzero reference velocity. No model, gain candidate, IDLE target assessment or overall PID verification is inferred from this capture. TUNE16 may independently compare complete bidirectional cruise LAG and its explicit target.\n";
    out<<"Cruise P1/P2 compares measured bidirectional PeakAbs and RMS only; signed means never cancel. A target pass is separate from relative improvement.\n"
        <<"Cruise P2 requires a successful P1 delivered during this same core startup. Restart or a new P1 request retires prior eligibility; files alone never authorize comparison.\n";
    return out.str();
}
void Execute() noexcept {
    try {
        if(g_revokeBaseline) {
            const bool legacyRevoked=!g_result||!g_result->baselineSaved||
                WriteChecked(BaselinePath(g_request),"TUNE01_CANCELLED\n");
            const bool cruiseRevoked=!g_result||!g_result->cruiseBaselineSaved||
                WriteChecked(CruiseBaselinePath(g_request),"TUNE16_CRUISE_CANCELLED\n");
            const bool revoked=legacyRevoked&&cruiseRevoked;
            if(g_result) {
                g_result->baselineSaved=false;g_result->cancelled=true;g_result->verified=false;g_result->motionVerified=false;g_result->idleVerified=false;
                g_result->analysis.candidateValid=false;g_result->analysis.idleCandidateValid=false;
                ClearComparison(*g_result);ClearCruiseComparison(*g_result,"CANCELLED");
                g_result->cruiseBaselineSaved=false;g_result->cruiseBaselineMatched=false;
                g_result->cruiseBaselineSession=0ULL;
                g_result->detail=revoked?"CANCELLED":"CANCEL_BASELINE_IO_FAILED";
            }
            g_revokeBaseline=false;
            g_work.store(Work::Ready,std::memory_order_release);return;
        }
        auto result=std::make_shared<JobResult>();
        const auto& job=g_request;
        // Worker-only disk retirement. Runtime eligibility was already cleared
        // at admission, so failed writes cannot resurrect an older P1.
        const bool directoryReady=EnsureDirectory(job.outputDir);
        const bool cruiseRetired=job.mode!=1U||(directoryReady&&
            WriteChecked(CruiseBaselinePath(job),"TUNE16_CRUISE_PENDING_OR_FAILED_P1\n"));
        if(!g_cancel.load(std::memory_order_acquire))
            result->analysis=job.capture.sampleStride>1U?
                AnalyzeDecimated(job.config,job.capture,job.statistics,g_samples,g_count,job.segments.data(),job.segments.size()):
                Analyze(job.config,job.capture,g_samples,g_count,job.segments.data(),job.segments.size());
        if(!job.measurementComplete) {
            // A late HOLD may arrive after the RT buffer completed but before
            // the NC owner commits the measurement. Keep the raw statistics,
            // but do not label a cancelled sparse test as a valid assessment.
            result->analysis.measurementsValid=false;
            result->analysis.qualityPassed=false;
            result->analysis.idleQualityPassed=false;result->analysis.idleCandidateValid=false;
            result->analysis.idleMeasurementsValid=false;result->analysis.idleReason=IdleAnalysisReason::Incomplete;
            result->analysis.candidateValid=false;
            result->analysis.kpValid=false;
            result->analysis.kvffValid=false;
            result->analysis.reason=AnalysisReason::Incomplete;
        }
        result->cancelled=g_cancel.load(std::memory_order_acquire);
        if(!result->cancelled) {
            if(job.mode==2U&&job.measurementComplete&&job.capture.sampleStride==1U) (void)LoadBaseline(job,*result);
            if(job.mode==2U)LoadCruiseBaseline(job,*result);
            if(directoryReady) {
                const std::string stem="TUNE01_"+job.config.machineId+"_A"+std::to_string(job.capture.axis)+
                    "_P"+std::to_string(job.mode)+"_"+Timestamp()+"_"+std::to_string(job.session);
                const std::string rawFile=stem+".csv";
                // This token pairs saved artifacts; it is neither a security
                // identifier nor evidence that a file belongs to a live run.
                const std::string captureToken=stem+"_"+std::to_string(g_count?g_samples[0].timestampUs:0ULL);
                result->rawSaved=SaveRaw(job,Join(job.outputDir,rawFile),captureToken);
                if(!g_cancel.load(std::memory_order_acquire)) {
                    if(job.mode==1U) {
                        if(job.measurementComplete&&result->analysis.candidateValid)
                            result->baselineSaved=SaveBaseline(job,result->analysis);
                        else (void)WriteChecked(BaselinePath(job),"TUNE01_NO_CANDIDATE\n");
                        const auto cruise=EvaluateCruiseTracking(job.config.axis[job.capture.axis],
                            job.capture,job.statistics,job.measurementComplete);
                        if(!CruiseIdentityValid(job))result->cruiseComparisonReason="CURRENT_CONTEXT_INVALID";
                        else if(CruiseCurrentAvailable(job,*result,cruise)) {
                            result->cruiseBaselineSaved=SaveCruiseBaseline(job,*result);
                            result->cruiseComparisonReason=result->cruiseBaselineSaved?"BASELINE_SAVED":"BASELINE_IO_FAILED";
                        } else result->cruiseComparisonReason=!cruiseRetired?"BASELINE_REVOKE_IO_FAILED":
                            !cruise.targetSpecified?"TARGET_NOT_SPECIFIED":
                            !job.measurementComplete||!result->analysis.measurementsValid?"CURRENT_MEASUREMENT_INVALID":
                            "CURRENT_CRUISE_UNAVAILABLE";
                    }
                    result->summarySaved=WriteChecked(Join(job.outputDir,stem+".txt"),Summary(job,*result,rawFile,captureToken));
                }
            }
            result->cancelled=g_cancel.load(std::memory_order_acquire);
            if(result->cancelled) {ClearComparison(*result);ClearCruiseComparison(*result,"CANCELLED");}
            if(job.mode==1U&&!directoryReady)result->cruiseComparisonReason="BASELINE_IO_FAILED";
            result->detail=result->cancelled?"CANCELLED":!result->rawSaved||!result->summarySaved?"RESULT_IO_FAILED":
                result->cruiseBaselineSaved?"CRUISE_BASELINE_SAVED":
                result->cruiseComparisonAvailable?result->cruiseComparisonReason:
                job.capture.sampleStride>1U&&job.measurementComplete?AnalysisReasonName(result->analysis.reason):
                job.mode==2U&&result->baselineOriginChecked&&!result->baselineOriginMatched?"BASELINE_ORIGIN_MISMATCH":
                job.mode==2U&&!result->baselineMatched?"NO_MATCHING_BASELINE":
                job.mode==2U&&(!result->motionContextCompatible||!result->idleContextCompatible)?"GAINS_OUTSIDE_BASELINE_AND_CANDIDATES":
                job.mode==2U&&(result->motionVerified||result->idleVerified)&&!result->verified?"PARTIALLY_VERIFIED_IN_THIS_TEST":
                job.mode==2U&&!result->candidateMatched?"CURRENT_GAINS_DO_NOT_MATCH_CANDIDATE":
                job.mode==2U&&result->verified?"VERIFIED_IN_THIS_TEST":
                !job.measurementComplete?job.failureReason:AnalysisReasonName(result->analysis.reason);
        }else {result->detail="CANCELLED";ClearCruiseComparison(*result,"CANCELLED");}
        g_result=std::move(result);
    } catch(...) {g_result.reset();}
    g_work.store(Work::Ready,std::memory_order_release);
}
#ifdef _WIN32
DWORD WINAPI Worker(void*) {
    if(!RtSetThreadPriority(GetCurrentThread(),20)) {
        g_result.reset();g_work.store(Work::Ready,std::memory_order_release);return 1U;
    }
    Execute();return 0U;
}
bool JoinReady() {
    if(g_thread==nullptr)return true;
    if(WaitForSingleObject(g_thread,0U)!=WAIT_OBJECT_0)return false;
    CloseHandle(g_thread);g_thread=nullptr;return true;
}
#else
void Worker() {Execute();}
bool JoinReady() {if(g_thread.joinable())g_thread.join();return true;}
#endif
}
void InvalidateCruiseBaseline(int axis) noexcept {
    if(axis<0||axis>=static_cast<int>(kAxisCount))return;
    g_cruiseEligible[axis].store(0ULL,std::memory_order_release);
    g_cruiseRevision[axis].fetch_add(1ULL,std::memory_order_acq_rel);
}
std::uint64_t CruiseBaselineRevision(int axis) noexcept {
    return axis<0||axis>=static_cast<int>(kAxisCount)?0ULL:
        g_cruiseRevision[axis].load(std::memory_order_acquire);
}
bool StartJob(const JobRequest& job,const CaptureSample* samples,std::size_t count) noexcept {
    // Direct callers without an NC admission stamp still revoke before any
    // early rejection (including zero samples or worker creation failure).
    if(job.mode==1U&&job.cruiseBaselineRevision==0ULL)InvalidateCruiseBaseline(job.capture.axis);
    if(g_work.load(std::memory_order_acquire)!=Work::Idle||!samples||count==0U||count>CaptureCapacity||
        job.session==0ULL||job.session!=job.capture.session||(job.mode!=1U&&job.mode!=2U)||
        job.capture.sampleStride==0U||job.capture.sampleStride>64U||
        job.capture.axis<0||job.capture.axis>=static_cast<int>(kAxisCount)||job.capture.axisType>1U||job.configIdentityHash==0ULL)return false;
    try {
        std::string error;if(!ValidateConfig(job.config,error))return false;
        g_request=job;g_samples=samples;g_count=count;g_result.reset();g_revokeBaseline=false;
        g_cruiseJobRevision=job.cruiseBaselineRevision?job.cruiseBaselineRevision:
            CruiseBaselineRevision(job.capture.axis);
        g_session.store(job.session,std::memory_order_release);g_cancel.store(false,std::memory_order_release);
        g_work.store(Work::Running,std::memory_order_release);
#ifdef _WIN32
        g_thread=CreateThread(nullptr,0U,Worker,nullptr,0U,nullptr);
        if(g_thread==nullptr) {g_work.store(Work::Idle,std::memory_order_release);return false;}
#else
        g_thread=std::thread(Worker);
#endif
        return true;
    } catch(...) {g_work.store(Work::Idle,std::memory_order_release);return false;}
}
bool PollJob(std::uint64_t session,std::shared_ptr<const JobResult>& result) noexcept {
    if(session!=g_session.load(std::memory_order_acquire)||g_work.load(std::memory_order_acquire)!=Work::Ready)return false;
    if(!JoinReady())return false;
    if(g_cancel.load(std::memory_order_acquire)&&g_result) {
        g_result->cancelled=true;g_result->verified=false;g_result->motionVerified=false;g_result->idleVerified=false;
        g_result->analysis.candidateValid=false;g_result->analysis.idleCandidateValid=false;g_result->analysis.kpValid=false;g_result->analysis.kvffValid=false;
        ClearComparison(*g_result);ClearCruiseComparison(*g_result,"CANCELLED");
        g_result->cruiseBaselineMatched=false;
        if(g_request.mode==1U)g_cruiseEligible[g_request.capture.axis].store(0ULL,std::memory_order_release);
        if(g_result->baselineSaved||g_result->cruiseBaselineSaved) {
            g_revokeBaseline=true;g_work.store(Work::Running,std::memory_order_release);
#ifdef _WIN32
            g_thread=CreateThread(nullptr,0U,Worker,nullptr,0U,nullptr);
            if(g_thread!=nullptr)return false;
#else
            try {g_thread=std::thread(Worker);return false;}catch(...) {}
#endif
            g_revokeBaseline=false;g_result->detail="CANCEL_BASELINE_WORKER_FAILED";
            g_result->cruiseBaselineSaved=false;
            if(g_request.mode==1U)g_result->cruiseBaselineSession=0ULL;
            g_work.store(Work::Ready,std::memory_order_release);
        }
    }
    if(g_result) {
        const int axis=g_request.capture.axis;
        const bool currentRevision=CruiseBaselineRevision(axis)==g_cruiseJobRevision;
        if(!currentRevision) {
            g_result->cruiseBaselineMatched=false;
            ClearCruiseComparison(*g_result,"BASELINE_NOT_ELIGIBLE");
        }
        if(g_request.mode==1U&&currentRevision&&!g_result->cancelled&&
            g_result->cruiseBaselineSaved&&g_result->rawSaved&&g_result->summarySaved)
            g_cruiseEligible[axis].store(g_request.session,std::memory_order_release);
    }
    result=g_result;g_result.reset();g_samples=nullptr;g_count=0;
    g_work.store(Work::Idle,std::memory_order_release);return true;
}
bool JobBusy() noexcept {return g_work.load(std::memory_order_acquire)!=Work::Idle;}
void CancelJob(std::uint64_t session) noexcept {
    if(session==g_session.load(std::memory_order_acquire))g_cancel.store(true,std::memory_order_release);
}
void ShutdownJobs() noexcept {
    for(unsigned axis=0U;axis<kAxisCount;++axis)InvalidateCruiseBaseline(static_cast<int>(axis));
    g_cancel.store(true,std::memory_order_release);
#ifdef _WIN32
    if(g_thread!=nullptr) {WaitForSingleObject(g_thread,INFINITE);CloseHandle(g_thread);g_thread=nullptr;}
#else
    if(g_thread.joinable())g_thread.join();
#endif
    // Shutdown can bypass the NC PollJob cancellation handshake. All cruise
    // authority was retired above; its audit file alone cannot authorize P2.
    // The legacy model record retains its established off-cycle cleanup.
    if(g_result&&g_result->baselineSaved) {
        bool revoked=false;
        try {revoked=WriteChecked(BaselinePath(g_request),"TUNE01_CANCELLED\n");}catch(...) {}
        if(!revoked) {
#ifdef _WIN32
            RtPrintf("[TUNE01] event=SHUTDOWN_BASELINE_REVOKE_FAILED session=%llu\n",
                static_cast<unsigned long long>(g_request.session));
#else
            std::fputs("[TUNE01] SHUTDOWN_BASELINE_REVOKE_FAILED\n",stderr);
#endif
        }
    }
    g_result.reset();g_samples=nullptr;g_count=0U;g_work.store(Work::Idle,std::memory_order_release);
}
}
