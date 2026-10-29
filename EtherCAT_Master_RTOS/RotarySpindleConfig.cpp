#include "RotarySpindleConfig.h"
#include "NCRotarySpindle.h"
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <locale>
#include <sstream>
#ifdef _WIN32
#include <Windows.h>
#else
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#endif

namespace RotarySpindleFile
{
namespace
{
constexpr std::size_t MaximumFileBytes=32768U,MaximumLineBytes=512U;
const char* const Fields[]={"Enabled","AxisNumber","MaxRPM","AccelTimeSec","DecelTimeSec",
    "StopTimeSec","StopToleranceRPM","ReadyToleranceRPM"};
const char Template[]=
    "# EDM289 rotary spindle configuration - startup/stopped worker only.\r\n"
    "# Path: D:\\EtherCAT_Master_Data\\Data\\Parameter\\RotarySpindleConfig.txt\r\n"
    "# Physical output requires Enabled=1 and an accepted MEMORY spindle M command.\r\n"
    "# AxisNumber is 1..8; defaults 4=C, 5=U, 6=V. Installed axis type must be rotary.\r\n"
    "# Enabled spindle mappings must be unique. No silent fallback on invalid files.\r\n"
    "# S and MaxRPM are OUTPUT-shaft rev/min; axis gearing comes from AxisConfig.\r\n"
    "# Effective maximum is the lower of configured MaxRPM and installed axis/PDO caps.\r\n"
    "# An S above that effective maximum is rejected; requested speed is never clamped.\r\n"
    "# Ramp times use configured MaxRPM: zero to MaxRPM, or MaxRPM to zero.\r\n"
    "# DecelTimeSec: same-direction speed reduction.\r\n"
    "# StopTimeSec: full stop before reversal, M-stop, HOLD, and program end.\r\n"
    "# Alarm/RESET use the existing safety stop authority, not this normal stop ramp.\r\n"
    "# These disabled defaults are examples, not verified machine speed ratings.\r\n"
    "SchemaVersion=1\r\n\r\n"
    "Spindle1.Enabled=0\r\nSpindle1.AxisNumber=4\r\nSpindle1.MaxRPM=500\r\n"
    "Spindle1.AccelTimeSec=2\r\nSpindle1.DecelTimeSec=2\r\nSpindle1.StopTimeSec=1\r\n"
    "Spindle1.StopToleranceRPM=0.1\r\nSpindle1.ReadyToleranceRPM=5\r\n\r\n"
    "Spindle2.Enabled=0\r\nSpindle2.AxisNumber=5\r\nSpindle2.MaxRPM=500\r\n"
    "Spindle2.AccelTimeSec=2\r\nSpindle2.DecelTimeSec=2\r\nSpindle2.StopTimeSec=1\r\n"
    "Spindle2.StopToleranceRPM=0.1\r\nSpindle2.ReadyToleranceRPM=5\r\n\r\n"
    "Spindle3.Enabled=0\r\nSpindle3.AxisNumber=6\r\nSpindle3.MaxRPM=500\r\n"
    "Spindle3.AccelTimeSec=2\r\nSpindle3.DecelTimeSec=2\r\nSpindle3.StopTimeSec=1\r\n"
    "Spindle3.StopToleranceRPM=0.1\r\nSpindle3.ReadyToleranceRPM=5\r\n";

bool Fail(Error& error,ErrorCode code,unsigned spindle=0U,const char* key="",unsigned line=0U) noexcept
{
    error=Error{};error.code=code;error.spindle=spindle;error.line=line;
    if(key){std::size_t n=0U;while(key[n]&&n+1U<sizeof(error.key)){error.key[n]=key[n];++n;}}
    return false;
}
bool Positive(double value) noexcept {return std::isfinite(value)&&value>0.0;}
double RpmForPps(double maximumPps,double pulsesPerRevolution) noexcept
{
    double rpm=maximumPps/pulsesPerRevolution*60.0;
    // A finite machine cap can describe more RPM than double represents when
    // the scale is very small. Its largest representable bound is sufficient.
    if(!std::isfinite(rpm))rpm=(std::numeric_limits<double>::max)();
    // Use the exact model/RT forward expression. An inverse conversion can
    // otherwise land one representable RPM above the installed/PDO PPS cap.
    for(unsigned repair=0U;repair<8U&&rpm>0.0;++repair){
        const double pps=rpm/60.0*pulsesPerRevolution;
        if(std::isfinite(pps)&&pps<=maximumPps)return pps>0.0?rpm:0.0;
        rpm=std::nextafter(rpm,0.0);
    }
    // Bounded conversion also fails closed for extreme/subnormal arithmetic
    // that cannot produce a positive, forward-valid bound in these steps.
    return 0.0;
}
bool White(char value) noexcept {return value==' '||value=='\t'||value=='\r';}
bool Digit(char value) noexcept {return value>='0'&&value<='9';}
std::string Trim(const std::string& value)
{
    std::size_t begin=0U,end=value.size();
    while(begin<end&&White(value[begin]))++begin;
    while(end>begin&&White(value[end-1U]))--end;
    return value.substr(begin,end-begin);
}
bool Unsigned(const std::string& text,unsigned& output) noexcept
{
    if(text.empty())return false;unsigned value=0U;
    for(char c:text){if(!Digit(c))return false;const unsigned digit=static_cast<unsigned>(c-'0');
        if(value>((std::numeric_limits<unsigned>::max)()-digit)/10U)return false;value=value*10U+digit;}
    output=value;return true;
}
bool Real(const std::string& text,double& output)
{
    std::size_t p=0U;if(p<text.size()&&(text[p]=='+'||text[p]=='-'))++p;
    bool digits=false;while(p<text.size()&&Digit(text[p])){digits=true;++p;}
    if(p<text.size()&&text[p]=='.'){++p;while(p<text.size()&&Digit(text[p])){digits=true;++p;}}
    if(!digits)return false;
    if(p<text.size()&&(text[p]=='e'||text[p]=='E')){++p;if(p<text.size()&&(text[p]=='+'||text[p]=='-'))++p;
        const auto start=p;while(p<text.size()&&Digit(text[p]))++p;if(p==start)return false;}
    if(p!=text.size())return false;
    std::istringstream stream(text);stream.imbue(std::locale::classic());double value=0.0;stream>>value;
    if(stream.fail()||!stream.eof()||!std::isfinite(value))return false;
    output=value;return true;
}
bool SelectField(const std::string& key,unsigned& spindle,unsigned& field) noexcept
{
    if(key.size()<10U||key.compare(0U,7U,"Spindle")!=0||key[7]<'1'||key[7]>'3'||key[8]!='.')return false;
    spindle=static_cast<unsigned>(key[7]-'1');
    for(unsigned i=0U;i<8U;++i)if(key.compare(9U,std::string::npos,Fields[i])==0){field=i;return true;}
    return false;
}
bool SetField(Spindle& s,unsigned field,const std::string& value)
{
    if(field==0U){if(value!="0"&&value!="1")return false;s.enabled=value=="1";return true;}
    if(field==1U)return Unsigned(value,s.axisNumber);
    double* target=field==2U?&s.maximumRpm:field==3U?&s.accelTimeSeconds:field==4U?&s.decelTimeSeconds:
        field==5U?&s.stopTimeSeconds:field==6U?&s.stoppedToleranceRpm:&s.readyToleranceRpm;
    return Real(value,*target);
}
enum class Presence {Missing,Present,Inaccessible};
Presence Probe(const std::string& path) noexcept
{
#ifdef _WIN32
    HANDLE file=CreateFileA(path.c_str(),GENERIC_READ,FILE_SHARE_READ|FILE_SHARE_WRITE|FILE_SHARE_DELETE,
        nullptr,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,nullptr);
    if(file!=INVALID_HANDLE_VALUE){CloseHandle(file);return Presence::Present;}
    const DWORD error=GetLastError();
    return error==ERROR_FILE_NOT_FOUND||error==ERROR_PATH_NOT_FOUND?Presence::Missing:Presence::Inaccessible;
#else
    struct stat info{};if(::stat(path.c_str(),&info)==0)return S_ISREG(info.st_mode)?Presence::Present:Presence::Inaccessible;
    return errno==ENOENT?Presence::Missing:Presence::Inaccessible;
#endif
}
bool Directory(const std::string& path) noexcept
{
#ifdef _WIN32
    return CreateDirectoryA(path.c_str(),nullptr)!=0||GetLastError()==ERROR_ALREADY_EXISTS;
#else
    if(::mkdir(path.c_str(),0770)==0)return true;
    struct stat info{};return errno==EEXIST&&::stat(path.c_str(),&info)==0&&S_ISDIR(info.st_mode);
#endif
}
bool ParentDirectories(const std::string& path)
{
    std::size_t start=0U;
    if(path.size()>=3U&&path[1]==':')start=3U;
    else if(!path.empty()&&(path[0]=='/'||path[0]=='\\'))start=1U;
    for(std::size_t p=start;p<path.size();++p)
        if((path[p]=='/'||path[p]=='\\')&&p>0U&&!Directory(path.substr(0U,p)))return false;
    return true;
}
bool Read(const std::string& path,std::string& text,Error& error)
{
    std::ifstream file(path,std::ios::binary);if(!file)return Fail(error,ErrorCode::OpenFailed);
    std::string bytes(MaximumFileBytes+1U,'\0');file.read(&bytes[0],static_cast<std::streamsize>(bytes.size()));
    const auto count=file.gcount();
    if(count<0||file.bad()||(file.fail()&&!file.eof()))return Fail(error,ErrorCode::ReadFailed);
    if(static_cast<std::size_t>(count)>MaximumFileBytes)return Fail(error,ErrorCode::FileTooLarge);
    bytes.resize(static_cast<std::size_t>(count));text.swap(bytes);return true;
}
enum class WriteResult {Created,AlreadyExists,Failed};
WriteResult Create(const std::string& path) noexcept
{
    const std::size_t length=sizeof(Template)-1U;std::size_t written=0U;bool okay=true;
#ifdef _WIN32
    // Same supported RTX64 create/write APIs as EDMConditionStartupIO.
    HANDLE file=CreateFileA(path.c_str(),GENERIC_WRITE,FILE_SHARE_READ,nullptr,CREATE_NEW,
        FILE_ATTRIBUTE_NORMAL|FILE_FLAG_WRITE_THROUGH,nullptr);
    if(file==INVALID_HANDLE_VALUE){const DWORD error=GetLastError();return
        error==ERROR_FILE_EXISTS||error==ERROR_ALREADY_EXISTS?WriteResult::AlreadyExists:WriteResult::Failed;}
    while(written<length){DWORD done=0U;if(!WriteFile(file,Template+written,static_cast<DWORD>(length-written),&done,nullptr)||done==0U){okay=false;break;}written+=done;}
    if(!CloseHandle(file))okay=false;
#else
    const int file=::open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL,0660);
    if(file<0)return errno==EEXIST?WriteResult::AlreadyExists:WriteResult::Failed;
    while(written<length){const ssize_t done=::write(file,Template+written,length-written);
        if(done<0&&errno==EINTR)continue;if(done<=0){okay=false;break;}written+=static_cast<std::size_t>(done);}
    if(::fsync(file)!=0)okay=false;if(::close(file)!=0)okay=false;
#endif
    return okay?WriteResult::Created:WriteResult::Failed;
}
}

const char* DefaultPath() noexcept {return "D:\\EtherCAT_Master_Data\\Data\\Parameter\\RotarySpindleConfig.txt";}
const char* DefaultText() noexcept {return Template;}
const char* ErrorName(ErrorCode code) noexcept
{
    switch(code){
#define E(name) case ErrorCode::name:return #name
    E(None);E(FileTooLarge);E(LineTooLong);E(InvalidByte);E(InvalidSyntax);E(UnknownKey);
    E(DuplicateKey);E(MissingKey);E(InvalidValue);E(InvalidSchema);E(AxisRange);E(DuplicateAxis);
    E(RpmRange);E(RampRange);E(ToleranceRange);E(AxisUnavailable);E(AxisNotRotary);E(AxisIdentity);
    E(AxisScale);E(MachineVelocityLimit);E(OpenFailed);E(ReadFailed);E(CreateDirectoryFailed);
    E(CreateFailed);E(WriteFailed);E(AllocationFailure);
#undef E
    default:return "Unknown";}
}

bool Validate(const Settings& settings,Error& error) noexcept
{
    error=Error{};unsigned mask=0U;
    for(unsigned i=0U;i<SpindleCount;++i){const auto& s=settings.spindle[i];
        if(s.axisNumber<1U||s.axisNumber>AxisCapacity)return Fail(error,ErrorCode::AxisRange,i+1U,"AxisNumber");
        if(s.enabled){const unsigned bit=1U<<(s.axisNumber-1U);
            if(mask&bit)return Fail(error,ErrorCode::DuplicateAxis,i+1U,"AxisNumber");mask|=bit;}
        if(!Positive(s.maximumRpm)||s.maximumRpm>MaximumConfiguredRpm)return Fail(error,ErrorCode::RpmRange,i+1U,"MaxRPM");
        const double times[]={s.accelTimeSeconds,s.decelTimeSeconds,s.stopTimeSeconds};
        for(unsigned k=0U;k<3U;++k)if(!Positive(times[k])||times[k]<MinimumRampSeconds||times[k]>MaximumRampSeconds||
            !Positive(s.maximumRpm/times[k]))return Fail(error,ErrorCode::RampRange,i+1U,Fields[k+3U]);
        if(!Positive(s.stoppedToleranceRpm)||s.stoppedToleranceRpm>=s.maximumRpm)
            return Fail(error,ErrorCode::ToleranceRange,i+1U,"StopToleranceRPM");
        if(!Positive(s.readyToleranceRpm)||s.readyToleranceRpm>=s.maximumRpm)
            return Fail(error,ErrorCode::ToleranceRange,i+1U,"ReadyToleranceRPM");
    }
    return true;
}

bool ParseText(const std::string& text,Settings& output,Error& error) noexcept
{
    output=Settings{};error=Error{};
    try{
        if(text.size()>MaximumFileBytes)return Fail(error,ErrorCode::FileTooLarge);
        Settings candidate{};bool schema=false;std::array<unsigned,SpindleCount> seen{};
        std::array<std::array<unsigned,8U>,SpindleCount> lines{};
        std::size_t begin=text.compare(0U,3U,"\xef\xbb\xbf")==0?3U:0U;unsigned lineNumber=0U;
        while(begin<text.size()){
            ++lineNumber;const auto newline=text.find('\n',begin);
            const auto end=newline==std::string::npos?text.size():newline;
            if(end-begin>MaximumLineBytes)return Fail(error,ErrorCode::LineTooLong,0U,"",lineNumber);
            std::string line=text.substr(begin,end-begin);begin=newline==std::string::npos?text.size():newline+1U;
            for(unsigned char c:line)if((c<0x20U&&c!='\t'&&c!='\r')||c>0x7eU)
                return Fail(error,ErrorCode::InvalidByte,0U,"",lineNumber);
            std::size_t comment=line.size();for(std::size_t k=0U;k<line.size();++k)
                if(line[k]=='#'||line[k]==';'||(line[k]=='/'&&k+1U<line.size()&&line[k+1U]=='/')){comment=k;break;}
            line=Trim(line.substr(0U,comment));if(line.empty())continue;
            const auto equal=line.find('=');
            if(equal==std::string::npos||line.find('=',equal+1U)!=std::string::npos)
                return Fail(error,ErrorCode::InvalidSyntax,0U,"",lineNumber);
            const auto key=Trim(line.substr(0U,equal)),value=Trim(line.substr(equal+1U));
            if(key.empty()||value.empty())return Fail(error,ErrorCode::InvalidSyntax,0U,key.c_str(),lineNumber);
            if(key=="SchemaVersion"){
                if(schema)return Fail(error,ErrorCode::DuplicateKey,0U,key.c_str(),lineNumber);
                if(value!="1")return Fail(error,ErrorCode::InvalidSchema,0U,key.c_str(),lineNumber);
                schema=true;continue;
            }
            unsigned spindle=0U,field=0U;
            if(!SelectField(key,spindle,field))return Fail(error,ErrorCode::UnknownKey,0U,key.c_str(),lineNumber);
            if(seen[spindle]&(1U<<field))return Fail(error,ErrorCode::DuplicateKey,spindle+1U,key.c_str(),lineNumber);
            if(!SetField(candidate.spindle[spindle],field,value))return Fail(error,ErrorCode::InvalidValue,spindle+1U,key.c_str(),lineNumber);
            seen[spindle]|=1U<<field;lines[spindle][field]=lineNumber;
        }
        if(!schema)return Fail(error,ErrorCode::MissingKey,0U,"SchemaVersion");
        for(unsigned i=0U;i<SpindleCount;++i)for(unsigned k=0U;k<8U;++k)
            if(!(seen[i]&(1U<<k)))return Fail(error,ErrorCode::MissingKey,i+1U,Fields[k]);
        if(!Validate(candidate,error)){
            if(error.spindle>0U&&error.spindle<=SpindleCount)for(unsigned k=0U;k<8U;++k)
                if(std::strcmp(error.key,Fields[k])==0)error.line=lines[error.spindle-1U][k];
            return false;
        }
        output=candidate;return true;
    }catch(...){return Fail(error,ErrorCode::AllocationFailure);}
}

LoadStatus LoadOrCreateDefault(const std::string& path,Settings& output,Error& error) noexcept
{
    output=Settings{};error=Error{};
    try{
        if(path.empty()||path.size()>4096U||path.find('\0')!=std::string::npos){Fail(error,ErrorCode::OpenFailed);return LoadStatus::IoError;}
        const auto presence=Probe(path);bool created=false;
        if(presence==Presence::Inaccessible){Fail(error,ErrorCode::OpenFailed);return LoadStatus::IoError;}
        if(presence==Presence::Missing){
            if(!ParentDirectories(path)){Fail(error,ErrorCode::CreateDirectoryFailed);return LoadStatus::IoError;}
            const auto written=Create(path);
            if(written==WriteResult::Failed){Fail(error,ErrorCode::CreateFailed);return LoadStatus::IoError;}
            created=written==WriteResult::Created;
        }
        std::string text;if(!Read(path,text,error))return error.code==ErrorCode::FileTooLarge?LoadStatus::Invalid:LoadStatus::IoError;
        if(created&&text!=Template){Fail(error,ErrorCode::WriteFailed);return LoadStatus::IoError;}
        if(!ParseText(text,output,error))return LoadStatus::Invalid;
        return created?LoadStatus::CreatedDisabled:LoadStatus::Loaded;
    }catch(...){Fail(error,ErrorCode::AllocationFailure);return LoadStatus::IoError;}
}

bool ComputeCapability(const Spindle& settings,const AxisFacts& axis,
    Capability& output,Error& error) noexcept
{
    output=Capability{};error=Error{};
    if(settings.axisNumber<1U||settings.axisNumber>AxisCapacity)return Fail(error,ErrorCode::AxisRange,0U,"AxisNumber");
    if(!Positive(settings.maximumRpm)||settings.maximumRpm>MaximumConfiguredRpm)return Fail(error,ErrorCode::RpmRange,0U,"MaxRPM");
    if(!axis.exists)return Fail(error,ErrorCode::AxisUnavailable,0U,"AxisNumber");
    if(axis.axisSlot!=settings.axisNumber-1U)return Fail(error,ErrorCode::AxisIdentity,0U,"AxisNumber");
    if(!axis.rotary)return Fail(error,ErrorCode::AxisNotRotary,0U,"AxisNumber");
    if(!Positive(axis.pulsesPerNative)||!Positive(axis.nativeUnitsPerOutputRevolution))
        return Fail(error,ErrorCode::AxisScale,0U,"AxisNumber");
    Capability candidate{};candidate.configuredRpm=settings.maximumRpm;
    candidate.pulsesPerOutputRevolution=axis.pulsesPerNative*axis.nativeUnitsPerOutputRevolution;
    if(!Positive(candidate.pulsesPerOutputRevolution))return Fail(error,ErrorCode::AxisScale,0U,"PulsesPerRevolution");
    if(!Positive(axis.maximumOutputPps))return Fail(error,ErrorCode::MachineVelocityLimit,0U,"AxisMaxVelocityPPS");
    const double pdoMaximumPps=static_cast<double>((std::numeric_limits<std::int32_t>::max)());
    candidate.axisRpm=RpmForPps(axis.maximumOutputPps,candidate.pulsesPerOutputRevolution);
    candidate.pdoRpm=RpmForPps(pdoMaximumPps,candidate.pulsesPerOutputRevolution);
    candidate.maximumOutputPps=(std::min)(axis.maximumOutputPps,pdoMaximumPps);
    candidate.effectiveRpm=(std::min)(candidate.configuredRpm,(std::min)(candidate.axisRpm,candidate.pdoRpm));
    const double pps=candidate.effectiveRpm/60.0*candidate.pulsesPerOutputRevolution;
    if(!Positive(candidate.effectiveRpm)||!Positive(pps)||pps>candidate.maximumOutputPps)
        return Fail(error,ErrorCode::MachineVelocityLimit,0U,"EffectiveMaxRPM");
    candidate.valid=true;output=candidate;return true;
}

bool BuildRuntime(const Settings& settings,const std::array<AxisFacts,AxisCapacity>& axes,
    RuntimeConfig& output,Error& error) noexcept
{
    output=RuntimeConfig{};error=Error{};if(!Validate(settings,error))return false;
    RuntimeConfig candidate{};
    for(unsigned i=0U;i<SpindleCount;++i){const auto& s=settings.spindle[i];auto& config=candidate[i];
        config.axisSlot=s.axisNumber-1U;config.enabled=s.enabled;
        config.maximumRpm=s.maximumRpm;config.accelerationRpmPerSecond=s.maximumRpm/s.accelTimeSeconds;
        config.decelerationRpmPerSecond=s.maximumRpm/s.decelTimeSeconds;
        config.stopDecelerationRpmPerSecond=s.maximumRpm/s.stopTimeSeconds;
        config.stoppedToleranceRpm=s.stoppedToleranceRpm;config.readyToleranceRpm=s.readyToleranceRpm;
        if(!s.enabled)continue;
        Capability capability{};
        if(!ComputeCapability(s,axes[config.axisSlot],capability,error)){error.spindle=i+1U;return false;}
        config.rotary=true;config.pulsesPerOutputRevolution=capability.pulsesPerOutputRevolution;
        config.maximumRpm=capability.effectiveRpm;
        if(config.stoppedToleranceRpm>=config.maximumRpm)
            return Fail(error,ErrorCode::ToleranceRange,i+1U,"StopToleranceRPM");
        if(config.readyToleranceRpm>=config.maximumRpm)
            return Fail(error,ErrorCode::ToleranceRange,i+1U,"ReadyToleranceRPM");
        if(!NCRotarySpindle::ValidConfig(config,i))return Fail(error,ErrorCode::InvalidValue,i+1U,"RuntimeConfig");
    }
    output=candidate;return true;
}
}
