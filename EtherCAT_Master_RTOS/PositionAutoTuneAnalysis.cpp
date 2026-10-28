#include "PositionAutoTuneAnalysis.h"
#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <utility>
#include <vector>

namespace PositionAutoTune {
namespace {
constexpr double Pi=3.14159265358979323846;
constexpr std::size_t Block=8U; // 2 ms velocity averages; analysis only.
constexpr double H=Block*CapturePeriodSeconds;
constexpr unsigned MaximumDelay=16U; // 0..32 ms; no claim outside this model family.
constexpr unsigned FineScale=2U; // Only 1 ms fallback; never 0.5 ms.
constexpr double ResolutionAgreementLimit=0.05;
struct Point { double y=0.0,u=0.0,r=0.0; };
struct Leg {
    std::vector<Point> p;
    bool train=false;
    int direction=0;
};
struct Model { double a=0.0,b=0.0,fit=-1.0; unsigned delay=0U; bool valid=false; };
bool Finite(double x) noexcept { return std::isfinite(x); }
bool Same(double x,double y) noexcept { return x==y; }
bool FiniteSample(const CaptureSample& s) noexcept {
    return Finite(s.commandPulse)&&Finite(s.actualPulse)&&Finite(s.referencePps)&&
        Finite(s.actualVelocityPps)&&Finite(s.proposedLogicalPps)&&Finite(s.outputLogicalPps)&&
        Finite(s.pTerm)&&Finite(s.iTerm)&&Finite(s.feedforwardPps)&&Finite(s.kp)&&
        Finite(s.ki)&&Finite(s.kd)&&Finite(s.kvff)&&Finite(s.integral);
}
bool Match(const CaptureSample& s,const CaptureGains& g) noexcept {
    return Same(s.kp,g.kp)&&Same(s.ki,g.ki)&&Same(s.kd,g.kd)&&Same(s.kvff,g.kvff);
}
std::size_t AtCycle(const CaptureSample* s,std::size_t n,std::uint64_t cycle) noexcept {
    if(!n||cycle<s[0].cycle) return n;
    const std::uint64_t offset=cycle-s[0].cycle;
    return offset<n?static_cast<std::size_t>(offset):n;
}
double RecordedPeakSpeed(const CaptureRequest& request,const CaptureSample* samples,
    std::size_t count) noexcept {
    if(!samples||count>CaptureCapacity||!request.session||
        !Finite(request.pulsesPerMm)||request.pulsesPerMm<=0.0) return 0.0;
    double peakPps=0.0;
    for(std::size_t i=0;i<count;++i) {
        const auto& sample=samples[i];
        if(sample.valid&&sample.session==request.session&&Finite(sample.referencePps))
            peakPps=std::max(peakPps,std::fabs(sample.referencePps));
    }
    const double speed=peakPps/request.pulsesPerMm*60.0;
    return Finite(speed)?speed:0.0;
}

// External trajectory velocity and its difference are instruments.  They
// avoid ordinary least-squares regression of feedback-correlated command
// noise onto measured velocity.  Insufficient rank rejects the model.
Model FitIV(const std::vector<Leg>& legs,unsigned delay,unsigned scale=1U) noexcept {
    // scale=1/2 means 2/1 ms. Keep 4 ms post-delay startup and 256 ms
    // aggregate support; unnormalized energy floors scale with point count.
    const double interval=H/scale;
    long double m00=0,m01=0,m10=0,m11=0,c0=0,c1=0;
    long double z1energy=0,z2energy=0;
    std::size_t count=0;
    for(const Leg& leg:legs) if(leg.train) {
        for(std::size_t i=delay+2U*scale;i<leg.p.size();++i) {
            const Point& p=leg.p[i];
            const double z1=leg.p[i-delay].r;
            const double z2=(leg.p[i-delay].r-leg.p[i-delay-1U].r)/interval;
            const double x0=leg.p[i-1U].y, x1=leg.p[i-delay].u;
            m00+=z1*x0; m01+=z1*x1; m10+=z2*x0; m11+=z2*x1;
            c0+=z1*p.y; c1+=z2*p.y;
            z1energy+=z1*z1; z2energy+=z2*z2; ++count;
        }
    }
    Model model{}; model.delay=delay;
    const long double det=m00*m11-m01*m10;
    const long double matrixScale=std::fabs(m00*m11)+std::fabs(m01*m10);
    if(count<128U*scale||z1energy<1e-12L*scale||z2energy<1e-12L*scale||
        matrixScale<1e-18L*scale*scale||std::fabs(det)<matrixScale*1e-6L) return model;
    model.a=static_cast<double>((c0*m11-m01*c1)/det);
    model.b=static_cast<double>((m00*c1-c0*m10)/det);
    // Preserve the same physical tau family across discretizations. Fixed
    // discrete a bounds on a shorter grid would silently change that family.
    const double gain=model.b/(1.0-model.a);
    const double minimumA=scale==1U?0.02:std::pow(0.02,1.0/scale);
    const double maximumA=scale==1U?0.995:std::pow(0.995,1.0/scale);
    model.valid=Finite(model.a)&&Finite(model.b)&&model.a>=minimumA&&model.a<=maximumA&&
        model.b>0.0&&Finite(gain)&&gain>=0.25&&gain<=4.0;
    return model;
}
double Score(const Model& model,const std::vector<Leg>& legs,bool training,int direction=0,unsigned scale=1U) noexcept {
    // Original 36 ms burn-in and 128 ms support, independent of grid spacing.
    long double sum=0.0L,sq=0.0L,sse=0.0L;
    std::size_t count=0U;
    for(const Leg& leg:legs) {
        if(leg.train!=training||(direction&&leg.direction!=direction)||leg.p.size()<4U*scale) continue;
        double predicted=leg.p.front().y;
        for(std::size_t i=1U;i<leg.p.size();++i) {
            const double input=leg.p[i>=model.delay?i-model.delay:0U].u;
            predicted=model.a*predicted+model.b*input;
            if(i<(MaximumDelay+2U)*scale) continue;
            const double actual=leg.p[i].y;
            const double error=predicted-actual;
            sum+=actual; sq+=actual*actual; sse+=error*error; ++count;
        }
    }
    if(count<64U*scale) return -1.0;
    const long double centered=sq-sum*sum/count;
    if(centered<=1e-12L*scale) return -1.0;
    return 1.0-std::sqrt(static_cast<double>(sse/centered));
}

// A DC feedforward estimate is independent of whether dynamic bandwidth was
// identifiable.  Only established plateaus, both directions, are included.
bool SteadyGain(const std::vector<Leg>& legs,double minimumVelocity,double& gain,
    std::array<std::uint32_t,4>& diagnosticCounts,AnalysisDetail& detail) noexcept {
    // Four independent groups: training/held-out x positive/negative.  Do
    // not average a changing drive gain into a plausible-looking feedforward.
    double sumU[4]={0,0,0,0},sumY[4]={0,0,0,0};
    std::size_t count[4]={0U,0U,0U,0U};
    for(const Leg& leg:legs) {
        for(std::size_t i=32U;i+4U<leg.p.size();++i) {
            const Point& p=leg.p[i];
            if(std::fabs(p.r)<minimumVelocity||p.u*p.y<=0.0||
                std::fabs(p.y)<minimumVelocity) continue;
            const double magnitude=std::fabs(p.r);
            bool flat=true;
            for(std::size_t j=i-32U;j<=i+4U;++j) {
                if(std::fabs(leg.p[j].r-p.r)>magnitude*0.001||
                    std::fabs(leg.p[j].u-p.u)>std::fabs(p.u)*0.02||
                    std::fabs(leg.p[j].y-p.y)>std::fabs(p.y)*0.02) { flat=false; break; }
            }
            if(!flat) continue;
            const unsigned k=(leg.train?0U:2U)+(p.r>0.0?0U:1U);
            sumU[k]+=p.u;sumY[k]+=p.y;++count[k];
        }
    }
    for(unsigned k=0U;k<4U;++k) diagnosticCounts[k]=static_cast<std::uint32_t>(count[k]);
    double minimum=4.0,maximum=0.0,sum=0.0;
    for(unsigned k=0U;k<4U;++k) {
        if(count[k]<64U) {detail=AnalysisDetail::InsufficientSteadyPlateau;return false;}
        if(std::fabs(sumU[k])<1e-9) {detail=AnalysisDetail::InvalidSteadyGain;return false;}
        const double value=sumY[k]/sumU[k];
        if(!Finite(value)||value<=0.0) {detail=AnalysisDetail::InvalidSteadyGain;return false;}
        minimum=std::min(minimum,value);maximum=std::max(maximum,value);sum+=value;
    }
    if(maximum-minimum>0.05*maximum) {detail=AnalysisDetail::InconsistentSteadyGain;return false;}
    gain=sum*0.25;
    const bool valid=Finite(gain)&&gain>=0.25&&gain<=4.0;
    detail=valid?AnalysisDetail::None:AnalysisDetail::SteadyGainOutsideModelRange;
    return valid;
}

struct FrequencyEvidence {
    std::complex<double> response{};
    double excitationRatio=-1.0, modelError=-1.0;
    bool supported=false;
};

// Keep training and held-out evidence separate: bandwidth selection may use
// training legs only. The held-out legs get one check of the selected value,
// never a search for another value that happens to pass validation.
FrequencyEvidence MeasureFrequency(const Model& model,const std::vector<Leg>& legs,
    bool training,double frequency,unsigned resolutionScale=1U) noexcept {
    using Complex=std::complex<double>;
    FrequencyEvidence evidence{};
    Complex numerator{},denominator{};
    double referencePower=0.0,timeSquare=0.0,timeEnergy=0.0,timeCount=0.0;
    const double w=2.0*Pi*frequency;
    const double interval=H/resolutionScale;
    // Keep the original 62.5 Hz ceiling even on the finer numerical grid.
    if(!(frequency>0.0&&frequency<1.0/(8.0*H))) return evidence;
    for(const Leg& leg:legs) {
        if(leg.train!=training) continue;
        Complex y{},u{},r{};
        for(std::size_t i=0;i<leg.p.size();++i) {
            const Complex rotation=std::polar(1.0,-w*interval*static_cast<double>(i));
            y+=leg.p[i].y*rotation*interval;u+=leg.p[i].u*rotation*interval;r+=leg.p[i].r*rotation*interval;
            timeEnergy+=leg.p[i].r*leg.p[i].r;
            timeCount+=1.0;
        }
        numerator+=y*std::conj(r);denominator+=u*std::conj(r);
        referencePower+=std::norm(r);
        const double duration=interval*leg.p.size();timeSquare+=duration*duration;
    }
    if(timeCount<64.0*resolutionScale||timeSquare<=0.0) return evidence;
    const double spectralAmplitude=std::sqrt(referencePower/timeSquare);
    const double rms=std::sqrt(timeEnergy/timeCount);
    if(!Finite(rms)||!Finite(spectralAmplitude)||rms<1e-9) return evidence;
    evidence.excitationRatio=spectralAmplitude/rms;
    const double denominatorMagnitude=std::abs(denominator);
    if(!Finite(denominatorMagnitude)||denominatorMagnitude<1e-12) return evidence;
    evidence.response=numerator/denominator;
    const Complex predicted=model.b*std::polar(1.0,-w*interval*model.delay)/
        (Complex(1.0,0.0)-model.a*std::polar(1.0,-w*interval));
    const double scale=std::abs(predicted);
    if(!Finite(scale)||scale<=0.0) return evidence;
    const double error=std::abs(evidence.response-predicted)/scale;
    if(!Finite(error)) return evidence;
    evidence.modelError=error;
    evidence.supported=evidence.excitationRatio>=0.005&&error<=0.20;
    return evidence;
}

Model TrainingModel(const std::vector<Leg>& legs,unsigned scale=1U) noexcept {
    Model best{};
    for(unsigned delay=0U;delay<=MaximumDelay*scale;++delay) {
        Model candidate=FitIV(legs,delay,scale);
        if(!candidate.valid) continue;
        candidate.fit=Score(candidate,legs,true,0,scale);
        if(candidate.fit>best.fit) best=candidate;
    }
    return best;
}

bool FineClockSupported(const CaptureSample* samples,std::size_t count) noexcept {
    // All rolling four-tick windows cover every decimation phase, including
    // each segment's independent alignment. The original raw clock checks
    // remain in force; a finer interval must earn its own timing evidence.
    constexpr std::size_t fineBlock=Block/FineScale;
    for(std::size_t i=fineBlock;i<count;++i) {
        const auto elapsed=samples[i].timestampUs-samples[i-fineBlock].timestampUs;
        if(elapsed<875ULL||elapsed>1125ULL) return false;
    }
    return true;
}

std::vector<Leg> FineLegs(const CaptureSample* samples,std::size_t sampleCount,
    const AnalysisSegment* segments,std::size_t segmentCount,double ppu,unsigned phase) {
    constexpr std::size_t fineBlock=Block/FineScale;
    constexpr double interval=H/FineScale;
    std::vector<Leg> legs;legs.reserve(segmentCount);
    for(std::size_t k=0U;k<segmentCount;++k) {
        const auto first=AtCycle(samples,sampleCount,segments[k].firstCycle);
        const auto last=AtCycle(samples,sampleCount,segments[k].lastCycle);
        Leg leg{};leg.train=(k%4U)<2U;leg.direction=segments[k].targetPulse>segments[k].startPulse?1:-1;
        leg.p.reserve((last-first)/fineBlock);
        for(std::size_t i=first+fineBlock+phase;i<=last;i+=fineBlock) {
            Point point{};
            point.y=(samples[i].actualPulse-samples[i-fineBlock].actualPulse)/(interval*ppu);
            for(std::size_t j=i-fineBlock;j<i;++j) {
                point.u+=samples[j].outputLogicalPps/(fineBlock*ppu);
                point.r+=samples[j].referencePps/(fineBlock*ppu);
            }
            leg.p.push_back(point);
        }
        legs.push_back(std::move(leg));
    }
    return legs;
}

bool SelectTrainingKp(const Config& config,const CaptureRequest& request,const Model& model,
    const std::vector<Leg>& legs,unsigned scale,AnalysisResult& evidence,
    double& kp,FrequencyEvidence& training) noexcept {
    const auto& axis=config.axis[request.axis];
    // The original 2 ms design reserve is independent of the numerical grid.
    const double rawKp=EffectiveCandidateResponseFactor(config,axis)/
        (evidence.plantGain*(evidence.tauSeconds+evidence.delaySeconds+H));
    kp=std::min(rawKp,std::min(axis.kpMax,request.motionGains.kp*EffectiveMaxKpIncreaseRatio(config,axis)));
    if(!Finite(kp)||kp<axis.kpMin||kp>axis.kpMax) {
        evidence.analysisDetail=AnalysisDetail::KpOutsideBounds;
        evidence.reason=AnalysisReason::CandidateBounds;return false;
    }
    evidence.requestedExcitationHz=kp*evidence.plantGain/(2.0*Pi);
    for(;;) {
        evidence.excitationHz=kp*evidence.plantGain/(2.0*Pi);
        training=MeasureFrequency(model,legs,true,evidence.excitationHz,scale);
        evidence.trainingExcitationRatio=training.excitationRatio;
        evidence.trainingFrequencyError=training.modelError;
        if(training.supported) break;
        if(kp<=axis.kpMin||evidence.bandwidthReductions>=16U) {
            evidence.analysisDetail=AnalysisDetail::TrainingFrequencyUnsupported;
            evidence.reason=AnalysisReason::Excitation;return false;
        }
        kp=std::max(axis.kpMin,kp*0.5);
        ++evidence.bandwidthReductions;
    }
    evidence.selectedTrainingKp=kp;
    return true;
}

void CopyTrainingEvidence(const AnalysisResult& from,AnalysisResult& to) noexcept {
    to.requestedExcitationHz=from.requestedExcitationHz;
    to.excitationHz=from.excitationHz;to.bandwidthReductions=from.bandwidthReductions;
    to.trainingExcitationRatio=from.trainingExcitationRatio;
    to.trainingFrequencyError=from.trainingFrequencyError;
    to.selectedTrainingKp=from.selectedTrainingKp;
}

std::complex<double> ModelResponse(const Model& model,double interval,double frequency) noexcept {
    const double w=2.0*Pi*frequency;
    return model.b*std::polar(1.0,-w*interval*model.delay)/
        (std::complex<double>(1.0,0.0)-model.a*std::polar(1.0,-w*interval));
}

bool ResolutionAgrees(const Model& coarse,const std::array<Model,4>& phases,
    double frequency,AnalysisResult& result) noexcept {
    const double gain=phases[0].b/(1.0-phases[0].a);
    const auto predicted=ModelResponse(phases[0],H/FineScale,frequency);
    const double magnitude=std::abs(predicted);
    if(!Finite(gain)||gain<=0.0||!Finite(magnitude)||magnitude<=0.0) return false;
    result.resolutionGainMismatch=result.resolutionResponseMismatch=0.0;
    for(unsigned i=0U;i<4U;++i) {
        const Model& other=i==0U?coarse:phases[i];
        const double otherGain=other.b/(1.0-other.a);
        const double gainDifference=std::fabs(otherGain-gain)/gain;
        const double responseDifference=std::abs(ModelResponse(other,i==0U?H:H/FineScale,frequency)-predicted)/magnitude;
        if(!Finite(gainDifference)||!Finite(responseDifference)) {
            result.resolutionGainMismatch=result.resolutionResponseMismatch=-1.0;return false;
        }
        result.resolutionGainMismatch=std::max(result.resolutionGainMismatch,gainDifference);
        result.resolutionResponseMismatch=std::max(result.resolutionResponseMismatch,responseDifference);
    }
    return result.resolutionGainMismatch<=ResolutionAgreementLimit&&
        result.resolutionResponseMismatch<=ResolutionAgreementLimit;
}

AnalysisResult AnalyzeImpl(const Config& config,const CaptureRequest& request,
    const CaptureSample* samples,std::size_t sampleCount,
    const AnalysisSegment* segments,std::size_t segmentCount) {
    AnalysisResult result{};result.measured=request.motionGains;
    result.suggested=request.motionGains;
    result.sampleCount=sampleCount;result.segmentCount=static_cast<std::uint32_t>(segmentCount);
    if(!request.session||request.axis<0||request.axis>=static_cast<int>(kAxisCount)||request.axisType>1U||
        !(Finite(request.pulsesPerMm)&&request.pulsesPerMm>0.0)||
        !Finite(request.originPulse)||!Finite(request.excursionPulse)||request.excursionPulse<=0.0||
        !Finite(config.modelFitMinimum)||config.modelFitMinimum<=0.0||config.modelFitMinimum>=1.0||
        !Finite(config.minimumVelocityMmS)||config.minimumVelocityMmS<=0.0||
        !Finite(config.candidateResponseFactor)||config.candidateResponseFactor<=0.0||
        !Finite(config.maxKpIncreaseRatio)||config.maxKpIncreaseRatio<1.0) {
        result.reason=AnalysisReason::Configuration;return result;
    }
    double pointToPointEnd=0.0;
    if(request.pointToPoint) {
        // Q1 keeps A as the measurement/baseline origin. Excursion is the
        // complete A->B leg distance; it is never a half-span about a center.
        pointToPointEnd=request.originPulse+request.excursionPulse;
        if(!Finite(pointToPointEnd)||pointToPointEnd<=request.originPulse) {
            result.reason=AnalysisReason::Configuration;return result;
        }
    }
    const AxisConfig& axis=config.axis[request.axis];
    if(axis.tuningStrategy>3U||!Finite(axis.targetPositionMm)||axis.targetPositionMm<0.0||
        !Finite(axis.followingLimitMm)||axis.followingLimitMm<=0.0||
        !Finite(axis.settleWindowMm)||axis.settleWindowMm<=0.0||
        !Finite(axis.settleHoldSeconds)||axis.settleHoldSeconds<=0.0||
        !Finite(axis.settleTimeoutSeconds)||axis.settleTimeoutSeconds<=0.0||
        !Finite(axis.maxOvershootMm)||axis.maxOvershootMm<0.0||
        !Finite(axis.maxRmsErrorMm)||axis.maxRmsErrorMm<=0.0||
        !Finite(axis.kpMin)||!Finite(axis.kpMax)||axis.kpMin<=0.0||axis.kpMax<axis.kpMin||
        !Finite(axis.kvffMin)||!Finite(axis.kvffMax)||axis.kvffMin<=0.0||axis.kvffMax<axis.kvffMin) {
        result.reason=AnalysisReason::Configuration;return result;
    }
    if(!samples||!segments||sampleCount<512U||sampleCount>CaptureCapacity||
        segmentCount<4U||segmentCount!=static_cast<std::size_t>(axis.cycles)*4U||segmentCount>128U) {
        result.reason=AnalysisReason::Incomplete;return result;
    }
    const double ppu=request.pulsesPerMm;
    bool saturated=false;
    for(std::size_t i=0;i<sampleCount;++i) {
        const CaptureSample& s=samples[i];
        const bool motion=s.gainBank==static_cast<std::uint32_t>(GainBank::Motion);
        const bool idle=s.gainBank==static_cast<std::uint32_t>(GainBank::Idle);
        if(!s.valid||s.session!=request.session||!FiniteSample(s)||(!motion&&!idle)||
            !Match(s,motion?request.motionGains:request.idleGains)) {
            result.reason=AnalysisReason::InvalidSample;return result;
        }
        if(i&&(samples[i-1U].cycle==(std::numeric_limits<std::uint64_t>::max)()||
            s.cycle!=samples[i-1U].cycle+1ULL||s.timestampUs<=samples[i-1U].timestampUs||
            s.timestampUs-samples[i-1U].timestampUs>1000ULL)) {
            result.reason=AnalysisReason::Discontinuity;return result;
        }
        // Fixed-period model arithmetic requires physical clock evidence.
        // Permit local QPC jitter; reject sustained period scaling/drift.
        if(i>=128U&&(i%128U)==0U) {
            const std::uint64_t elapsed=s.timestampUs-samples[i-128U].timestampUs;
            if(elapsed<30400ULL||elapsed>33600ULL) {
                result.reason=AnalysisReason::Discontinuity;return result;
            }
        }
        if(i>=Block&&(i%Block)==0U) {
            const std::uint64_t elapsed=s.timestampUs-samples[i-Block].timestampUs;
            if(elapsed<1750ULL||elapsed>2250ULL) {
                result.reason=AnalysisReason::Discontinuity;return result;
            }
        }
        saturated=saturated||s.saturated!=0U;
        result.peakErrorMm=std::max(result.peakErrorMm,std::fabs(s.commandPulse-s.actualPulse)/ppu);
    }
    const double expectedUs=static_cast<double>(sampleCount-1U)*250.0;
    const double elapsedUs=static_cast<double>(samples[sampleCount-1U].timestampUs-samples[0].timestampUs);
    if(std::fabs(elapsedUs-expectedUs)>std::max(1000.0,expectedUs*0.02)) {
        result.reason=AnalysisReason::Discontinuity;return result;
    }
    result.durationSeconds=elapsedUs*1e-6;
    std::vector<Leg> legs;legs.reserve(segmentCount);
    long double movingSquared=0.0L,idleSquared=0.0L;
    std::size_t movingCount=0U,idleCount=0U;
    bool settledAll=true;
    for(std::size_t k=0;k<segmentCount;++k) {
        const AnalysisSegment& segment=segments[k];
        const std::size_t first=AtCycle(samples,sampleCount,segment.firstCycle);
        std::size_t complete=AtCycle(samples,sampleCount,segment.commandCompleteCycle);
        const std::size_t last=AtCycle(samples,sampleCount,segment.lastCycle);
        const double delta=segment.targetPulse-segment.startPulse;
        const double expectedStart=request.pointToPoint?((k%2U)==0U?request.originPulse:pointToPointEnd):
            request.originPulse+((k%4U)==1U?request.excursionPulse:(k%4U)==3U?-request.excursionPulse:0.0);
        const double expectedTarget=request.pointToPoint?((k%2U)==0U?pointToPointEnd:request.originPulse):
            request.originPulse+((k%4U)==0U?request.excursionPulse:(k%4U)==2U?-request.excursionPulse:0.0);
        const bool wrongPointDirection=request.pointToPoint&&((k%2U)==0U?delta<=0.0:delta>=0.0);
        if(first>=sampleCount||last>=sampleCount||complete>=sampleCount||
            first>=complete||complete>last||!Finite(delta)||delta==0.0||wrongPointDirection||
            std::fabs(segment.startPulse-expectedStart)>1.0||
            std::fabs(segment.targetPulse-expectedTarget)>1.0||
            (k&&segment.firstCycle<=segments[k-1U].lastCycle)||
            std::fabs(samples[complete].commandPulse-segment.targetPulse)>1.0||
            std::fabs(samples[complete].referencePps)>1.0) {
            result.reason=AnalysisReason::Segment;return result;
        }
        while(complete>first+1U&&
            std::fabs(samples[complete-1U].commandPulse-segment.targetPulse)<=1.0&&
            std::fabs(samples[complete-1U].referencePps)<=1.0) --complete;
        const double direction=delta>0.0?1.0:-1.0;
        std::size_t lastOutside=complete;
        bool hadOutside=false;
        for(std::size_t i=first;i<=last;++i) {
            const double error=(samples[i].commandPulse-samples[i].actualPulse)/ppu;
            if(i<complete) { movingSquared+=error*error;++movingCount; }
            else { idleSquared+=error*error;++idleCount; }
            result.maxOvershootMm=std::max(result.maxOvershootMm,
                std::max(0.0,direction*(samples[i].actualPulse-segment.targetPulse)/ppu));
            if(i>=complete&&std::fabs(samples[i].actualPulse-segment.targetPulse)/ppu>axis.settleWindowMm) {
                lastOutside=i;hadOutside=true;
            }
        }
        const std::size_t settleBegin=hadOutside?lastOutside+1U:complete;
        const double settledDuration=settleBegin<=last?
            (samples[last].timestampUs-samples[settleBegin].timestampUs)*1e-6:0.0;
        const double settlingSeconds=settleBegin<=last?
            (samples[settleBegin].timestampUs-samples[complete].timestampUs)*1e-6:
            (samples[last].timestampUs-samples[complete].timestampUs)*1e-6+CapturePeriodSeconds;
        result.maxSettlingSeconds=std::max(result.maxSettlingSeconds,settlingSeconds);
        if(settledDuration+1e-12<axis.settleHoldSeconds||settlingSeconds>axis.settleTimeoutSeconds) settledAll=false;
        if(settleBegin<=last) {
            double minimum=samples[settleBegin].actualPulse,maximum=minimum;
            for(std::size_t i=settleBegin;i<=last;++i) {
                minimum=std::min(minimum,samples[i].actualPulse);maximum=std::max(maximum,samples[i].actualPulse);
            }
            result.maxSettledJitterMm=std::max(result.maxSettledJitterMm,(maximum-minimum)/ppu);
        }
        Leg leg{};leg.train=(k%4U)<2U;leg.direction=delta>0.0?1:-1;
        leg.p.reserve((last-first)/Block);
        for(std::size_t i=first+Block;i<=last;i+=Block) {
            Point point{};
            point.y=(samples[i].actualPulse-samples[i-Block].actualPulse)/(H*ppu);
            for(std::size_t j=i-Block;j<i;++j) {
                point.u+=samples[j].outputLogicalPps/(Block*ppu);
                point.r+=samples[j].referencePps/(Block*ppu);
            }
            leg.p.push_back(point);
        }
        legs.push_back(std::move(leg));
    }
    if(!movingCount||!idleCount) { result.reason=AnalysisReason::Incomplete;return result; }
    result.rmsMovingErrorMm=std::sqrt(static_cast<double>(movingSquared/movingCount));
    result.rmsIdleErrorMm=std::sqrt(static_cast<double>(idleSquared/idleCount));
    if(!Finite(result.rmsMovingErrorMm)||!Finite(result.rmsIdleErrorMm)||
        !Finite(result.peakErrorMm)||!Finite(result.maxOvershootMm)||!Finite(result.maxSettledJitterMm)) {
        result.reason=AnalysisReason::InvalidSample;return result;
    }
    result.measurementsValid=true;
    result.qualityPassed=settledAll&&!saturated&&result.peakErrorMm<=axis.followingLimitMm&&
        result.rmsMovingErrorMm<=axis.maxRmsErrorMm&&result.maxOvershootMm<=axis.maxOvershootMm;
    if(saturated) { result.reason=AnalysisReason::Saturated;return result; }
    if(!result.qualityPassed) { result.reason=AnalysisReason::Quality;return result; }
    // TUNE01 does not infer integral/derivative values or alter their policy.
    if(request.motionGains.ki!=0.0||request.motionGains.kd!=0.0||
        !(request.motionGains.kp>0.0&&request.motionGains.kvff>0.0)) {
        result.reason=AnalysisReason::UnsupportedGains;return result;
    }
    result.suggested.ki=0.0;result.suggested.kd=0.0;
    double dcGain=0.0;
    AnalysisDetail steadyDetail=AnalysisDetail::None;
    if(SteadyGain(legs,config.minimumVelocityMmS,dcGain,result.steadyPlateauCounts,steadyDetail)) {
        const double candidate=1.0/dcGain;
        if(Finite(candidate)&&candidate>=axis.kvffMin&&candidate<=axis.kvffMax) {
            result.suggested.kvff=candidate;result.kvffValid=true;
        } else steadyDetail=AnalysisDetail::FeedforwardOutsideBounds;
    }
    const Model coarse=TrainingModel(legs);
    Model best=coarse;
    if(best.valid) result.trainingFit=best.fit;
    if(!best.valid||best.fit<config.modelFitMinimum) {
        result.analysisDetail=!best.valid?AnalysisDetail::NoValidDynamicModel:AnalysisDetail::TrainingTimeFitBelowMinimum;
        result.reason=AnalysisReason::Model;return result;
    }
    const double coarseTau=-H/std::log(coarse.a);
    const bool fineAttempt=Finite(coarseTau)&&coarseTau>0.0&&coarseTau<H;
    unsigned resolutionScale=1U;
    std::vector<Leg> fineLegs;
    const std::vector<Leg>* selectedLegs=&legs;
    std::array<Model,4> phaseModels{};
    if(fineAttempt) {
        // The only resolution trigger uses the coarse TRAINING model. No
        // held-out time/frequency score or DC group selects a finer grid.
        result.resolutionFallbackUsed=true;result.analysisIntervalSeconds=H/FineScale;
        if(!FineClockSupported(samples,sampleCount)) {
            result.analysisDetail=AnalysisDetail::FineClockUnsupported;
            result.reason=AnalysisReason::Model;return result;
        }
        resolutionScale=FineScale;
        fineLegs=FineLegs(samples,sampleCount,segments,segmentCount,ppu,0U);
        selectedLegs=&fineLegs;
        phaseModels[0]=TrainingModel(fineLegs,FineScale);
        best=phaseModels[0];result.trainingFit=best.valid?best.fit:0.0;
        for(unsigned phase=0U;phase<phaseModels.size();++phase) {
            if(phase) {
                const auto shifted=FineLegs(samples,sampleCount,segments,segmentCount,ppu,phase);
                phaseModels[phase]=TrainingModel(shifted,FineScale);
            }
            const auto& model=phaseModels[phase];
            const double tau=model.valid?-(H/FineScale)/std::log(model.a):0.0;
            if(!model.valid||model.fit<config.modelFitMinimum||!Finite(tau)||tau<H/FineScale) {
                result.analysisDetail=AnalysisDetail::ResolutionPhaseUnsupported;
                result.reason=AnalysisReason::Model;return result;
            }
            ++result.resolutionPhaseModels;
        }
    }
    const double interval=H/resolutionScale;
    // Cache selection using training data only before looking at held-out
    // scores. The coarse path commits legacy diagnostic fields in its original
    // order, preserving all successful 2 ms results exactly.
    AnalysisResult trainingChoice=result;
    trainingChoice.plantGain=best.b/(1.0-best.a);
    trainingChoice.tauSeconds=-interval/std::log(best.a);
    trainingChoice.delaySeconds=best.delay*interval;
    const double trainingModelFf=1.0/trainingChoice.plantGain;
    const bool feedforwardAgrees=result.kvffValid&&
        std::fabs(trainingModelFf-result.suggested.kvff)<=0.05*result.suggested.kvff;
    double kp=0.0;FrequencyEvidence training{};
    // DC includes held-out plateaus and is rejection-only. It must not decide
    // whether the training proposal is selected or change that proposal.
    const bool trainingSelected=Finite(trainingChoice.tauSeconds)&&
        trainingChoice.tauSeconds>=interval&&SelectTrainingKp(config,request,best,*selectedLegs,
            resolutionScale,trainingChoice,kp,training);
    result.selectedTrainingKp=trainingChoice.selectedTrainingKp;
    if(fineAttempt) {
        result.plantGain=trainingChoice.plantGain;result.tauSeconds=trainingChoice.tauSeconds;
        result.delaySeconds=trainingChoice.delaySeconds;
        CopyTrainingEvidence(trainingChoice,result);
        if(!trainingSelected) {
            result.analysisDetail=trainingChoice.analysisDetail;result.reason=trainingChoice.reason;return result;
        }
        // Phase zero and its crossover are frozen. Other phases and the
        // coarse training model may only reject; they never select a phase,
        // reduce Kp again, or trigger a third numerical resolution.
        if(!ResolutionAgrees(coarse,phaseModels,result.excitationHz,result)) {
            result.analysisDetail=AnalysisDetail::ResolutionMismatch;
            result.reason=AnalysisReason::Model;return result;
        }
        if(!feedforwardAgrees) {
            result.analysisDetail=!result.kvffValid?steadyDetail:AnalysisDetail::ModelFeedforwardDisagreement;
            result.reason=AnalysisReason::Validation;return result;
        }
    }
    result.validationFit=std::min(Score(best,*selectedLegs,false,0,resolutionScale),
        std::min(Score(best,*selectedLegs,false,1,resolutionScale),Score(best,*selectedLegs,false,-1,resolutionScale)));
    if(result.validationFit<config.modelFitMinimum) {
        result.analysisDetail=AnalysisDetail::HeldOutTimeFitBelowMinimum;
        result.reason=AnalysisReason::Validation;return result;
    }
    result.plantGain=trainingChoice.plantGain;
    result.tauSeconds=trainingChoice.tauSeconds;
    result.delaySeconds=trainingChoice.delaySeconds;
    if(!Finite(result.tauSeconds)||result.tauSeconds<interval) {
        result.analysisDetail=AnalysisDetail::ModelTimeConstantUnresolved;
        result.reason=AnalysisReason::Model;return result;
    }
    result.modelValid=true;
    if(!feedforwardAgrees) {
        result.analysisDetail=!result.kvffValid?steadyDetail:AnalysisDetail::ModelFeedforwardDisagreement;
        result.reason=AnalysisReason::Validation;return result;
    }
    if(!fineAttempt) CopyTrainingEvidence(trainingChoice,result);
    if(!trainingSelected) {
        result.analysisDetail=trainingChoice.analysisDetail;result.reason=trainingChoice.reason;return result;
    }
    const FrequencyEvidence validation=MeasureFrequency(best,*selectedLegs,false,result.excitationHz,resolutionScale);
    result.validationExcitationRatio=validation.excitationRatio;
    result.validationFrequencyError=validation.modelError;
    if(!validation.supported) {
        result.analysisDetail=AnalysisDetail::HeldOutFrequencyUnsupported;
        result.reason=AnalysisReason::Excitation;return result;
    }
    const std::complex<double> predicted=ModelResponse(best,interval,result.excitationHz);
    result.frequencyRepeatError=std::abs(training.response-validation.response)/std::abs(predicted);
    if(!Finite(result.frequencyRepeatError)||result.frequencyRepeatError>0.15) {
        result.analysisDetail=AnalysisDetail::FrequencyRepeatMismatch;
        result.reason=AnalysisReason::Excitation;return result;
    }
    result.suggested.kp=kp;result.kpValid=true;
    result.candidateValid=result.kpValid&&result.kvffValid;
    result.reason=AnalysisReason::None;
    return result;
}

// IDLE has no independent rich excitation. Use the already cross-validated
// velocity-plant model only as a conservative design seed, and separately
// require real stopped-command IDLE observations at every endpoint. This does
// not identify an "optimal Ki", nor simulate a load/disturbance that was absent.
void AnalyzeIdle(const Config& config,const CaptureRequest& request,
    const CaptureSample* samples,std::size_t sampleCount,
    const AnalysisSegment* segments,std::size_t segmentCount,AnalysisResult& result) noexcept {
    result.measuredIdle=request.idleGains;
    if(request.axis<0||request.axis>=static_cast<int>(kAxisCount)||request.axisType>1U) {
        result.idleReason=IdleAnalysisReason::Configuration;return;
    }
    const AxisConfig& axis=config.axis[request.axis];
    if(!axis.idleEnabled) {result.idleReason=IdleAnalysisReason::Disabled;return;}
    if(!Finite(axis.idleKpMin)||axis.idleKpMin<=0.0||!Finite(axis.idleKpMax)||axis.idleKpMax<axis.idleKpMin||
        !Finite(axis.idleKiMin)||axis.idleKiMin<0.0||!Finite(axis.idleKiMax)||axis.idleKiMax<axis.idleKiMin||
        !Finite(axis.idleIntegralTimeSeconds)||axis.idleIntegralTimeSeconds<=0.0||
        !Finite(axis.idleObservationSeconds)||axis.idleObservationSeconds<0.05||
        !Finite(axis.idleMaxRmsErrorMm)||axis.idleMaxRmsErrorMm<=0.0||
        !Finite(axis.idleMaxJitterMm)||axis.idleMaxJitterMm<0.0) {
        result.idleReason=IdleAnalysisReason::Configuration;return;
    }
    result.idleRequiredObservationSeconds=axis.idleObservationSeconds;
    if(Finite(request.idleGains.kp)&&request.idleGains.kp>0.0&&
        Finite(request.idleGains.ki)&&request.idleGains.ki>0.0) {
        const double integralHorizon=2.0*(request.idleGains.kp/request.idleGains.ki);
        if(!Finite(integralHorizon)) {result.idleReason=IdleAnalysisReason::Incomplete;return;}
        result.idleRequiredObservationSeconds=std::max(result.idleRequiredObservationSeconds,integralHorizon);
    }
    if(!result.measurementsValid||!samples||!segments||!sampleCount||!segmentCount) {
        result.idleReason=result.reason==AnalysisReason::InvalidSample||result.reason==AnalysisReason::Discontinuity?
            IdleAnalysisReason::InvalidSample:IdleAnalysisReason::Incomplete;return;
    }
    bool saturated=false;
    for(std::size_t i=0;i<sampleCount;++i) saturated=saturated||samples[i].saturated!=0U;
    double minimumObserved=(std::numeric_limits<double>::infinity)();
    for(std::size_t k=0;k<segmentCount;++k) {
        const auto& segment=segments[k];
        const std::size_t first=AtCycle(samples,sampleCount,segment.firstCycle);
        const std::size_t last=AtCycle(samples,sampleCount,segment.lastCycle);
        if(first>=sampleCount||last>=sampleCount||first>=last) {
            result.idleReason=IdleAnalysisReason::Incomplete;return;
        }
        auto idleEndpoint=[&](std::size_t i) noexcept {
            const auto& s=samples[i];
            return s.gainBank==static_cast<std::uint32_t>(GainBank::Idle)&&s.motionState==0U&&
                std::fabs(s.referencePps)<=1.0&&std::fabs(s.commandPulse-segment.targetPulse)<=1.0;
        };
        if(!idleEndpoint(last)) {result.idleReason=IdleAnalysisReason::Incomplete;return;}
        std::size_t tail=last;
        while(tail>first&&idleEndpoint(tail-1U)) --tail;
        const double observed=(samples[last].timestampUs-samples[tail].timestampUs)*1e-6;
        minimumObserved=std::min(minimumObserved,observed);
        result.idleObservedSeconds=minimumObserved;
        if(observed+1e-12<axis.idleObservationSeconds) {
            result.idleReason=IdleAnalysisReason::Incomplete;return;
        }
        // Include at least the requested physical time; at most one sampling
        // interval is added at the leading edge of this final observation.
        std::size_t begin=last;
        while(begin>tail&&(samples[last].timestampUs-samples[begin].timestampUs)*1e-6<axis.idleObservationSeconds) --begin;
        long double sum=0.0L,squared=0.0L;
        double minimum=samples[begin].actualPulse,maximum=minimum;
        for(std::size_t i=begin;i<=last;++i) {
            const double error=(segment.targetPulse-samples[i].actualPulse)/request.pulsesPerMm;
            sum+=error;squared+=error*error;
            minimum=std::min(minimum,samples[i].actualPulse);maximum=std::max(maximum,samples[i].actualPulse);
        }
        const std::size_t count=last-begin+1U;
        result.idleTailRmsMm=std::max(result.idleTailRmsMm,std::sqrt(static_cast<double>(squared/count)));
        result.idlePeakBiasMm=std::max(result.idlePeakBiasMm,std::fabs(static_cast<double>(sum/count)));
        result.idleJitterMm=std::max(result.idleJitterMm,(maximum-minimum)/request.pulsesPerMm);
    }
    if(!Finite(result.idleTailRmsMm)||!Finite(result.idlePeakBiasMm)||!Finite(result.idleJitterMm)||
        !Finite(result.idleObservedSeconds)) {
        result.idleReason=IdleAnalysisReason::InvalidSample;return;
    }
    result.idleMeasurementsValid=true;
    result.idleQualityPassed=!saturated&&result.idleTailRmsMm<=axis.idleMaxRmsErrorMm&&
        result.idleJitterMm<=axis.idleMaxJitterMm;
    if(saturated) {result.idleReason=IdleAnalysisReason::Saturated;return;}
    if(!result.idleQualityPassed) {result.idleReason=IdleAnalysisReason::Quality;return;}
    const CaptureGains& current=request.idleGains;
    if(!Finite(current.kp)||current.kp<=0.0||!Finite(current.ki)||current.ki<0.0||
        current.kd!=0.0||current.kvff!=0.0) {
        result.idleReason=IdleAnalysisReason::UnsupportedGains;return;
    }
    if(!result.candidateValid||!result.modelValid||!Finite(result.plantGain)||result.plantGain<=0.0||
        !Finite(result.tauSeconds)||result.tauSeconds<=0.0||!Finite(result.delaySeconds)||result.delaySeconds<0.0) {
        result.idleReason=IdleAnalysisReason::Model;return;
    }
    const double kp=std::min(result.suggested.kp,
        std::min(axis.idleKpMax,current.kp*EffectiveMaxKpIncreaseRatio(config,axis)));
    if(!Finite(kp)||kp<axis.idleKpMin||kp>axis.idleKpMax) {
        result.idleReason=IdleAnalysisReason::CandidateBounds;return;
    }
    // For Gv(s)=gain/(tau*s+1), position PI gives
    // tau*s^3+s^2+gain*Kp*s+gain*Ki. Ti=Kp/Ki > tau is the
    // delay-free Routh condition. Ten lag/delay periods reserve margin, and
    // Ti>=4/(gain*Kp) keeps the low-frequency PI approximation overdamped.
    // Delays/friction/load still require the user's independent P2 test.
    const double ti=std::max(axis.idleIntegralTimeSeconds,
        std::max(10.0*(result.tauSeconds+result.delaySeconds+H),4.0/(result.plantGain*kp)));
    const double ki=axis.idleKiMax==0.0?0.0:std::min(kp/ti,axis.idleKiMax);
    if(!Finite(ti)||ti<=0.0||!Finite(ki)||ki<axis.idleKiMin||ki>axis.idleKiMax) {
        result.idleReason=IdleAnalysisReason::CandidateBounds;return;
    }
    const double candidateHorizon=ki>0.0?2.0*(kp/ki):axis.idleObservationSeconds;
    if(!Finite(candidateHorizon)||result.idleObservedSeconds+1e-9<
        std::max(result.idleRequiredObservationSeconds,candidateHorizon)) {
        result.idleReason=IdleAnalysisReason::Incomplete;return;
    }
    result.suggestedIdle.kp=kp;result.suggestedIdle.ki=ki;
    result.suggestedIdle.kd=0.0;result.suggestedIdle.kvff=0.0;
    result.idleCandidateValid=true;result.idleReason=IdleAnalysisReason::None;
}
}

AnalysisResult Analyze(const Config& config,const CaptureRequest& request,
    const CaptureSample* samples,std::size_t sampleCount,
    const AnalysisSegment* segments,std::size_t segmentCount) noexcept {
    const double recordedPeakSpeed=RecordedPeakSpeed(request,samples,sampleCount);
    try {
        AnalysisResult result=AnalyzeImpl(config,request,samples,sampleCount,segments,segmentCount);
        result.measuredPeakSpeedNativePerMin=recordedPeakSpeed;
        AnalyzeIdle(config,request,samples,sampleCount,segments,segmentCount,result);
        return result;
    }
    catch(...) { AnalysisResult result{};result.reason=AnalysisReason::Allocation;
        result.measuredPeakSpeedNativePerMin=recordedPeakSpeed;return result; }
}
AnalysisResult AnalyzeDecimated(const Config& config,const CaptureRequest& request,
    const CaptureStatistics& statistics,const CaptureSample* samples,
    std::size_t sampleCount,const AnalysisSegment* segments,std::size_t segmentCount) noexcept {
    AnalysisResult result{};
    result.measured=request.motionGains;result.measuredIdle=request.idleGains;
    result.suggested=request.motionGains;
    result.sampleCount=sampleCount;
    result.analysisIntervalSeconds=0.0; // No model analysis grid in this mode.
    if(!request.session||request.axis<0||request.axis>=static_cast<int>(kAxisCount)||request.axisType>1U||
        !request.pointToPoint||request.sampleStride<=1U||request.sampleStride>64U||
        !Finite(request.pulsesPerMm)||request.pulsesPerMm<=0.0||
        !Finite(request.originPulse)||!Finite(request.excursionPulse)||request.excursionPulse<=0.0||
        !request.maximumTicks) {
        result.reason=AnalysisReason::Configuration;return result;
    }
    const auto& axis=config.axis[request.axis];
    const double endpointB=request.originPulse+request.excursionPulse;
    if(!axis.rangeEnabled||axis.cycles==0U||axis.cycles>32U||axis.tuningStrategy>3U||
        !Finite(axis.targetPositionMm)||axis.targetPositionMm<0.0||
        !Finite(endpointB)||endpointB<=request.originPulse) {
        result.reason=AnalysisReason::Configuration;return result;
    }
    if(!samples||!segments||sampleCount==0U||sampleCount>CaptureCapacity||
        segmentCount!=static_cast<std::size_t>(axis.cycles)*4U||segmentCount>128U||
        !statistics.valid||!statistics.complete||statistics.session!=request.session||
        !statistics.firstCycle||statistics.lastCycle<statistics.firstCycle||
        !statistics.firstTimestampUs||statistics.lastTimestampUs<=statistics.firstTimestampUs||
        statistics.sampleCount<2ULL||statistics.sampleCount>request.maximumTicks||
        statistics.sampleCount!=statistics.lastCycle-statistics.firstCycle+1ULL||
        !statistics.movingSampleCount||statistics.movingSampleCount>statistics.sampleCount) {
        result.reason=AnalysisReason::Incomplete;return result;
    }
    result.segmentCount=static_cast<std::uint32_t>(segmentCount);
    if(!Finite(statistics.peakFollowingErrorPulse)||statistics.peakFollowingErrorPulse<0.0||
        !Finite(statistics.meanSquaredFollowingErrorPulse)||statistics.meanSquaredFollowingErrorPulse<0.0||
        !Finite(statistics.meanSquaredMovingFollowingErrorPulse)||statistics.meanSquaredMovingFollowingErrorPulse<0.0||
        !Finite(statistics.peakReferencePps)||statistics.peakReferencePps<=1.0) {
        result.reason=AnalysisReason::InvalidSample;return result;
    }
    const double peakTolerance=1e-9*(std::max)(1.0,statistics.peakFollowingErrorPulse);
    const double referenceTolerance=1e-9*(std::max)(1.0,statistics.peakReferencePps);
    const double totalRmsPulse=std::sqrt(statistics.meanSquaredFollowingErrorPulse);
    const double movingRmsPulse=std::sqrt(statistics.meanSquaredMovingFollowingErrorPulse);
    // Check subset/coherent moments without squaring a potentially huge peak
    // or multiplying a mean by a long raw count.
    const double movingContribution=statistics.meanSquaredMovingFollowingErrorPulse*
        (static_cast<double>(statistics.movingSampleCount)/static_cast<double>(statistics.sampleCount));
    const double momentTolerance=1e-9*(std::max)(1.0,statistics.meanSquaredFollowingErrorPulse);
    if(totalRmsPulse>statistics.peakFollowingErrorPulse+peakTolerance||
        movingRmsPulse>statistics.peakFollowingErrorPulse+peakTolerance||
        !Finite(movingContribution)||movingContribution>statistics.meanSquaredFollowingErrorPulse+momentTolerance) {
        result.reason=AnalysisReason::InvalidSample;return result;
    }
    const auto& first=samples[0];const auto& last=samples[sampleCount-1U];
    if(first.cycle!=statistics.firstCycle||first.timestampUs!=statistics.firstTimestampUs||
        last.cycle>statistics.lastCycle||last.timestampUs>statistics.lastTimestampUs||
        statistics.lastCycle-last.cycle>=request.sampleStride||
        sampleCount!=(statistics.sampleCount-1ULL)/request.sampleStride+1ULL) {
        result.reason=AnalysisReason::Discontinuity;return result;
    }
    const std::uint64_t tailTicks=statistics.lastCycle-last.cycle;
    const std::uint64_t tailUs=statistics.lastTimestampUs-last.timestampUs;
    // A same-cycle endpoint has the exact same timestamp. An omitted tail
    // still obeyed the full-rate guard: each raw tick advanced by 1..1000 us.
    if((!tailTicks&&tailUs)||tailUs<tailTicks||tailUs>tailTicks*1000ULL) {
        result.reason=AnalysisReason::Discontinuity;return result;
    }
    const long double rawExpectedUs=static_cast<long double>(statistics.sampleCount-1ULL)*250.0L;
    const long double rawElapsedUs=static_cast<long double>(statistics.lastTimestampUs-statistics.firstTimestampUs);
    if(std::fabs(rawElapsedUs-rawExpectedUs)>(std::max)(1000.0L,rawExpectedUs*0.02L)) {
        result.reason=AnalysisReason::Discontinuity;return result;
    }
    const std::uint64_t stride=request.sampleStride;
    const long double storedExpectedUs=static_cast<long double>(stride)*250.0L;
    const long double storedToleranceUs=(std::max)(125.0L,storedExpectedUs*0.125L);
    for(std::size_t i=0U;i<sampleCount;++i) {
        const auto& sample=samples[i];
        const bool motion=sample.gainBank==static_cast<std::uint32_t>(GainBank::Motion);
        const bool idle=sample.gainBank==static_cast<std::uint32_t>(GainBank::Idle);
        if(sample.valid!=1U||sample.saturated>1U||sample.session!=request.session||
            !FiniteSample(sample)||(!motion&&!idle)||
            !Match(sample,motion?request.motionGains:request.idleGains)||
            (sample.saturated&&!statistics.anySaturated)||
            std::fabs(sample.commandPulse-sample.actualPulse)>statistics.peakFollowingErrorPulse+peakTolerance||
            std::fabs(sample.referencePps)>statistics.peakReferencePps+referenceTolerance) {
            result.reason=AnalysisReason::InvalidSample;return result;
        }
        if(i) {
            const auto& previous=samples[i-1U];
            if(previous.cycle>(std::numeric_limits<std::uint64_t>::max)()-stride||
                sample.cycle!=previous.cycle+stride||sample.timestampUs<=previous.timestampUs||
                std::fabs(static_cast<long double>(sample.timestampUs-previous.timestampUs)-storedExpectedUs)>storedToleranceUs) {
                result.reason=AnalysisReason::Discontinuity;return result;
            }
        }
    }
    // Segment markers describe raw RT cycles, not sparse row offsets. They
    // need not land on a saved row, and may never be passed through AtCycle.
    for(std::size_t k=0U;k<segmentCount;++k) {
        const auto& segment=segments[k];
        const bool forward=(k%2U)==0U;
        const double expectedStart=forward?request.originPulse:endpointB;
        const double expectedTarget=forward?endpointB:request.originPulse;
        const double delta=segment.targetPulse-segment.startPulse;
        if(segment.firstCycle<statistics.firstCycle||segment.lastCycle>statistics.lastCycle||
            segment.firstCycle>=segment.commandCompleteCycle||segment.commandCompleteCycle>segment.lastCycle||
            (k&&segment.firstCycle<=segments[k-1U].lastCycle)||
            !Finite(segment.startPulse)||!Finite(segment.targetPulse)||!Finite(delta)||
            (forward?delta<=0.0:delta>=0.0)||
            std::fabs(segment.startPulse-expectedStart)>1.0||std::fabs(segment.targetPulse-expectedTarget)>1.0) {
            result.reason=AnalysisReason::Segment;return result;
        }
    }
    const double ppu=request.pulsesPerMm;
    result.peakErrorMm=statistics.peakFollowingErrorPulse/ppu;
    result.rmsMovingErrorMm=movingRmsPulse/ppu;
    result.measuredPeakSpeedNativePerMin=statistics.peakReferencePps/ppu*60.0;
    result.durationSeconds=static_cast<double>(rawElapsedUs*1e-6L);
    if(!Finite(result.peakErrorMm)||!Finite(result.rmsMovingErrorMm)||
        !Finite(result.measuredPeakSpeedNativePerMin)||!Finite(result.durationSeconds)) {
        result.reason=AnalysisReason::InvalidSample;return result;
    }
    result.measurementsValid=true;result.reason=AnalysisReason::LongStrokeMetricsOnly;
    return result;
}
PositionTargetAssessment EvaluatePositionTarget(const AxisConfig& axis,
    const AnalysisResult& result,bool measurementComplete) noexcept {
    PositionTargetAssessment target{};
    if(!measurementComplete||!result.measurementsValid||!result.idleMeasurementsValid||!result.idleQualityPassed||
        !Finite(axis.targetPositionMm)||axis.targetPositionMm<=0.0||
        !Finite(result.idlePeakBiasMm)||result.idlePeakBiasMm<0.0||
        !Finite(result.idleJitterMm)||result.idleJitterMm<0.0)return target;
    // Largest absolute mean bias plus full peak-to-peak jitter is a
    // conservative bound for the recorded tails, not a future precision claim.
    const double bound=result.idlePeakBiasMm+result.idleJitterMm;
    if(!Finite(bound))return target;
    target.available=true;target.errorUpperBoundMm=bound;target.met=bound<=axis.targetPositionMm;
    return target;
}
CruiseTrackingAssessment EvaluateCruiseTracking(const AxisConfig& axis,
    const CaptureRequest& request,const CaptureStatistics& statistics,
    bool measurementComplete) noexcept {
    CruiseTrackingAssessment result{};
    // A malformed evidence/config path must not make an explicitly requested
    // positive target disappear from the caller's verification gate.
    result.targetSpecified=Finite(axis.cruiseLagTargetMm)&&axis.cruiseLagTargetMm>0.0;
    result.targetMm=result.targetSpecified?axis.cruiseLagTargetMm:0.0;
    if(!Finite(axis.cruiseLagTargetMm)||axis.cruiseLagTargetMm<0.0||
        !Finite(axis.speedMmMin)||axis.speedMmMin<=0.0||!axis.cycles||axis.cycles>32U)return result;
    result.targetMm=axis.cruiseLagTargetMm;result.targetSpecified=axis.cruiseLagTargetMm>0.0;
    result.expectedLegsPerDirection=2U*axis.cycles;
    if(!request.session||!Finite(request.pulsesPerMm)||request.pulsesPerMm<=0.0||
        !Finite(request.cruiseVelocityPps)||request.cruiseVelocityPps<=0.0||
        !Finite(request.maximumVelocityPps)||!Finite(request.maximumReferenceVelocityPps)||
        request.maximumReferenceVelocityPps<0.0||request.maximumReferenceVelocityPps>request.maximumVelocityPps||
        request.cruiseVelocityPps>ReferenceVelocityLimitPps(request)||
        !request.maximumTicks||request.maximumTicks>14400000ULL||
        !statistics.valid||statistics.session!=request.session||
        statistics.cruiseQualificationVersion!=CruiseQualificationVersion||
        !statistics.sampleCount||statistics.sampleCount>request.maximumTicks||
        !statistics.firstCycle||statistics.lastCycle<statistics.firstCycle||
        statistics.lastCycle-statistics.firstCycle!=statistics.sampleCount-1ULL||
        !statistics.firstTimestampUs||statistics.lastTimestampUs<statistics.firstTimestampUs||
        statistics.movingSampleCount>statistics.sampleCount||
        !Finite(statistics.peakFollowingErrorPulse)||statistics.peakFollowingErrorPulse<0.0||
        !Finite(statistics.meanSquaredFollowingErrorPulse)||statistics.meanSquaredFollowingErrorPulse<0.0||
        !Finite(statistics.peakReferencePps)||statistics.peakReferencePps<0.0||
        !Finite(statistics.meanSquaredMovingFollowingErrorPulse)||statistics.meanSquaredMovingFollowingErrorPulse<0.0)
        return result;
    const double totalTolerance=(std::max)(1.0,statistics.peakFollowingErrorPulse)*1e-9;
    const double movingContribution=statistics.meanSquaredMovingFollowingErrorPulse*
        (static_cast<double>(statistics.movingSampleCount)/static_cast<double>(statistics.sampleCount));
    if(std::sqrt(statistics.meanSquaredFollowingErrorPulse)>statistics.peakFollowingErrorPulse+totalTolerance||
        std::sqrt(statistics.meanSquaredMovingFollowingErrorPulse)>statistics.peakFollowingErrorPulse+totalTolerance||
        !Finite(movingContribution)||movingContribution>statistics.meanSquaredFollowingErrorPulse+
            (std::max)(1.0,statistics.meanSquaredFollowingErrorPulse)*1e-9)return result;
    const double ppu=request.pulsesPerMm;
    result.requestedSpeedNativePerMin=request.cruiseVelocityPps/ppu*60.0;
    if(!Finite(result.requestedSpeedNativePerMin)||std::fabs(result.requestedSpeedNativePerMin-axis.speedMmMin)>
        (std::max)(1e-9,axis.speedMmMin*1e-9))return result;
    const CruiseTrackingAssessment unavailable=result;
    const auto validate=[&](const CaptureTrackingMoments& moments,CruiseTrackingMetrics& metrics) noexcept {
        if(moments.sampleCount>statistics.movingSampleCount||
            moments.observedUs>moments.sampleCount*1000ULL||
            moments.observedUs>statistics.lastTimestampUs-statistics.firstTimestampUs||
            moments.qualifiedLegCount>128U||
            static_cast<std::uint64_t>(moments.qualifiedLegCount)*CruiseMinimumObservedUs>moments.observedUs||
            !Finite(moments.peakAbsoluteErrorPulse)||moments.peakAbsoluteErrorPulse<0.0||
            !Finite(moments.meanErrorPulse)||!Finite(moments.meanSquaredErrorPulse)||moments.meanSquaredErrorPulse<0.0)
            return false;
        if(!moments.sampleCount&&(moments.observedUs||moments.qualifiedLegCount||
            moments.peakAbsoluteErrorPulse!=0.0||moments.meanErrorPulse!=0.0||moments.meanSquaredErrorPulse!=0.0))return false;
        const double tolerance=(std::max)(1.0,moments.peakAbsoluteErrorPulse)*1e-9;
        const double rms=std::sqrt(moments.meanSquaredErrorPulse);
        if(std::fabs(moments.meanErrorPulse)>rms+tolerance||rms>moments.peakAbsoluteErrorPulse+tolerance||
            moments.peakAbsoluteErrorPulse>statistics.peakFollowingErrorPulse+
                (std::max)(1.0,statistics.peakFollowingErrorPulse)*1e-9)return false;
        metrics.sampleCount=moments.sampleCount;metrics.qualifiedLegCount=moments.qualifiedLegCount;
        metrics.observedSeconds=static_cast<double>(moments.observedUs)*1e-6;
        metrics.peakAbsoluteMm=moments.peakAbsoluteErrorPulse/ppu;
        metrics.rmsMm=rms/ppu;metrics.meanMm=moments.meanErrorPulse/ppu;
        return Finite(metrics.observedSeconds)&&Finite(metrics.peakAbsoluteMm)&&Finite(metrics.rmsMm)&&Finite(metrics.meanMm);
    };
    if(!validate(statistics.cruisePositive,result.positive)||!validate(statistics.cruiseNegative,result.negative)||
        !validate(statistics.nonCruiseMoving,result.nonCruise))return unavailable;
    const auto& positive=statistics.cruisePositive;const auto& negative=statistics.cruiseNegative;
    const auto& nonCruise=statistics.nonCruiseMoving;
    if(positive.sampleCount+negative.sampleCount+nonCruise.sampleCount!=statistics.movingSampleCount||
        positive.observedUs+negative.observedUs+nonCruise.observedUs>
            statistics.lastTimestampUs-statistics.firstTimestampUs||nonCruise.qualifiedLegCount!=0U)
        return unavailable;
    if(statistics.movingSampleCount) {
        const double count=static_cast<double>(statistics.movingSampleCount);
        const double partitionMeanSquared=positive.meanSquaredErrorPulse*(static_cast<double>(positive.sampleCount)/count)+
            negative.meanSquaredErrorPulse*(static_cast<double>(negative.sampleCount)/count)+
            nonCruise.meanSquaredErrorPulse*(static_cast<double>(nonCruise.sampleCount)/count);
        if(!Finite(partitionMeanSquared)||std::fabs(partitionMeanSquared-statistics.meanSquaredMovingFollowingErrorPulse)>
            (std::max)(1.0,statistics.meanSquaredMovingFollowingErrorPulse)*1e-8)return unavailable;
    }
    result.statisticsValid=true;result.complete=statistics.complete&&measurementComplete;
    result.peakAbsoluteMm=(std::max)(result.positive.peakAbsoluteMm,result.negative.peakAbsoluteMm);
    result.available=result.targetSpecified&&result.complete&&!statistics.anySaturated&&
        result.positive.qualifiedLegCount==result.expectedLegsPerDirection&&
        result.negative.qualifiedLegCount==result.expectedLegsPerDirection;
    result.targetMet=result.available&&result.peakAbsoluteMm<=result.targetMm;
    return result;
}
const char* AnalysisReasonName(AnalysisReason reason) noexcept {
    switch(reason) {
    case AnalysisReason::None:return "UNVERIFIED_CANDIDATE";
    case AnalysisReason::Configuration:return "CONFIGURATION";
    case AnalysisReason::Incomplete:return "INCOMPLETE";
    case AnalysisReason::InvalidSample:return "INVALID_SAMPLE";
    case AnalysisReason::Discontinuity:return "DISCONTINUITY";
    case AnalysisReason::Saturated:return "OUTPUT_SATURATED";
    case AnalysisReason::Segment:return "SEGMENT_IDENTITY";
    case AnalysisReason::Quality:return "MEASURED_QUALITY_FAILED";
    case AnalysisReason::Excitation:return "INSUFFICIENT_BANDWIDTH_EXCITATION";
    case AnalysisReason::Model:return "MODEL_UNIDENTIFIABLE";
    case AnalysisReason::Validation:return "HELD_OUT_MODEL_VALIDATION_FAILED";
    case AnalysisReason::CandidateBounds:return "CANDIDATE_OUTSIDE_BOUNDS";
    case AnalysisReason::UnsupportedGains:return "TUNE01_REQUIRES_G00_KI_KD_ZERO";
    case AnalysisReason::Allocation:return "ANALYSIS_ALLOCATION";
    case AnalysisReason::LongStrokeMetricsOnly:return "LONG_STROKE_METRICS_ONLY";
    default:return "UNKNOWN";
    }
}
const char* AnalysisDetailName(AnalysisDetail detail) noexcept {
    switch(detail) {
    case AnalysisDetail::None:return "NONE";
    case AnalysisDetail::InsufficientSteadyPlateau:return "INSUFFICIENT_STEADY_PLATEAU";
    case AnalysisDetail::InvalidSteadyGain:return "INVALID_STEADY_GAIN";
    case AnalysisDetail::InconsistentSteadyGain:return "INCONSISTENT_STEADY_GAIN";
    case AnalysisDetail::SteadyGainOutsideModelRange:return "STEADY_GAIN_OUTSIDE_MODEL_RANGE";
    case AnalysisDetail::FeedforwardOutsideBounds:return "FEEDFORWARD_OUTSIDE_BOUNDS";
    case AnalysisDetail::NoValidDynamicModel:return "NO_VALID_DYNAMIC_MODEL";
    case AnalysisDetail::TrainingTimeFitBelowMinimum:return "TRAINING_TIME_FIT_BELOW_MINIMUM";
    case AnalysisDetail::HeldOutTimeFitBelowMinimum:return "HELD_OUT_TIME_FIT_BELOW_MINIMUM";
    case AnalysisDetail::ModelTimeConstantUnresolved:return "MODEL_TIME_CONSTANT_UNRESOLVED";
    case AnalysisDetail::ModelFeedforwardDisagreement:return "MODEL_FEEDFORWARD_DISAGREEMENT";
    case AnalysisDetail::KpOutsideBounds:return "KP_OUTSIDE_BOUNDS";
    case AnalysisDetail::TrainingFrequencyUnsupported:return "TRAINING_FREQUENCY_UNSUPPORTED";
    case AnalysisDetail::HeldOutFrequencyUnsupported:return "HELD_OUT_FREQUENCY_UNSUPPORTED";
    case AnalysisDetail::FrequencyRepeatMismatch:return "FREQUENCY_REPEAT_MISMATCH";
    case AnalysisDetail::FineClockUnsupported:return "FINE_CLOCK_UNSUPPORTED";
    case AnalysisDetail::ResolutionPhaseUnsupported:return "RESOLUTION_PHASE_UNSUPPORTED";
    case AnalysisDetail::ResolutionMismatch:return "RESOLUTION_MISMATCH";
    }
    return "UNKNOWN_ANALYSIS_DETAIL";
}
const char* IdleAnalysisReasonName(IdleAnalysisReason reason) noexcept {
    switch(reason) {
    case IdleAnalysisReason::None:return "UNVERIFIED_MODEL_BASED_IDLE_PI";
    case IdleAnalysisReason::Disabled:return "IDLE_ESTIMATE_DISABLED";
    case IdleAnalysisReason::Incomplete:return "INSUFFICIENT_CONTINUOUS_IDLE_OBSERVATION";
    case IdleAnalysisReason::InvalidSample:return "INVALID_IDLE_SAMPLE";
    case IdleAnalysisReason::Saturated:return "OUTPUT_SATURATED";
    case IdleAnalysisReason::Quality:return "MEASURED_IDLE_QUALITY_FAILED";
    case IdleAnalysisReason::Model:return "NO_SUPPORTED_MOTION_PLANT_MODEL";
    case IdleAnalysisReason::CandidateBounds:return "IDLE_CANDIDATE_OUTSIDE_BOUNDS";
    case IdleAnalysisReason::UnsupportedGains:return "IDLE_REQUIRES_ZERO_KD_KVFF";
    case IdleAnalysisReason::Configuration:return "IDLE_CONFIGURATION";
    }
    return "UNKNOWN_IDLE_REASON";
}
}
