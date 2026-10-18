#pragma once

#include <array>
#include <cmath>
#include <fstream>
#include <string>

// Boot-time parameter loading only. This helper owns no axis state, PID
// history, motion authority or output. Existing IDLE/G00 parsing is unchanged.
namespace EDM165Config
{
struct FlushGains
{
    double Kp=0.0, Ki=0.0, Kd=0.0, Kvff=1.0;
};

inline bool WhiteSpace(char value) noexcept
{
    return value==' ' || value=='\t' || value=='\r' || value=='\n' ||
        value=='\f' || value=='\v';
}

inline std::string Trim(const std::string& value)
{
    std::size_t begin=0U, end=value.size();
    while(begin<end && WhiteSpace(value[begin]))++begin;
    while(end>begin && WhiteSpace(value[end-1U]))--end;
    return value.substr(begin,end-begin);
}

inline bool ParseGain(const std::string& text,double& output) noexcept
{
    try
    {
        std::size_t consumed=0U;
        const double candidate=std::stod(text,&consumed);
        while(consumed<text.size() && WhiteSpace(text[consumed]))++consumed;
        if(consumed!=text.size() || !std::isfinite(candidate) || candidate<0.0)return false;
        output=candidate;
        return true;
    }
    catch(...){return false;}
}

inline bool LoadAxisFlushGains(const std::string& path,const std::string& prefix,
    const FlushGains& fallback,FlushGains& output,std::string& errorKey)
{
    const std::array<std::string,4U> keys{{prefix+"Kp_FLUSH",prefix+"Ki_FLUSH",
        prefix+"Kd_FLUSH",prefix+"Kvff_FLUSH"}};
    FlushGains candidate=fallback;
    const std::array<double*,4U> targets{{&candidate.Kp,&candidate.Ki,&candidate.Kd,&candidate.Kvff}};
    std::array<bool,4U> seen{{false,false,false,false}};
    errorKey=prefix+"FLUSH_CONFIG";
    std::ifstream file(path);
    // Match the existing PID file contract: an absent file/field inherits
    // the already loaded CNC gain. A present malformed field never inherits.
    if(!file.is_open()){output=candidate;errorKey.clear();return true;}
    std::string line;
    while(std::getline(file,line))
    {
        const std::string trimmed=Trim(line);
        if(trimmed.empty() || trimmed.compare(0U,2U,"//")==0)continue;
        const std::size_t equal=trimmed.find('=');
        const std::string key=Trim(trimmed.substr(0U,equal));
        for(std::size_t field=0U;field<keys.size();++field)
        {
            if(key!=keys[field])continue;
            errorKey=keys[field];
            if(seen[field] || equal==std::string::npos ||
                !ParseGain(trimmed.substr(equal+1U),*targets[field]))return false;
            seen[field]=true;
            break;
        }
    }
    if(file.bad()){errorKey=prefix+"FLUSH_READ";return false;}
    output=candidate;
    errorKey.clear();
    return true;
}
}
