#pragma once
#include <cmath>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <locale>
#include <sstream>
#include <string>

// EDM382/384: boot-only opt-in for physical GAP coordinated line motion while the
// generator is controlled externally. This grants no discharge-output authority.
namespace EDM382
{
struct Config
{
    bool enabled=false,manualExternalPower=false,configurationInvalid=false;
    std::uint32_t requiredECode=210U;
};
inline bool Valid(const Config& c) noexcept
{
    return !c.configurationInvalid&&(!c.enabled||c.manualExternalPower)&&c.requiredECode==210U;
}
inline bool AssertionReady(const Config& c,std::uint32_t selectedECode,unsigned axisMask) noexcept
{
    return c.enabled&&c.manualExternalPower&&Valid(c)&&selectedECode==c.requiredECode&&(axisMask&7U)==7U&&(axisMask&~0x3fU)==0U;
}
inline Config& BootStorage() noexcept { static Config value{};return value; }
inline const Config& Current() noexcept { return BootStorage(); }
inline bool SelectedOrInvalid() noexcept { return Current().enabled||Current().configurationInvalid; }
inline bool InstallBeforeStart(const Config& value) noexcept
{
    static bool attempted=false;
    if(attempted)return false;
    attempted=true;
    if(!Valid(value)){BootStorage()=value;BootStorage().enabled=false;BootStorage().configurationInvalid=true;return false;}
    BootStorage()=value;return true;
}
inline std::string Trim(const std::string& value)
{
    const auto first=value.find_first_not_of(" \t\r\n");
    if(first==std::string::npos)return std::string{};
    return value.substr(first,value.find_last_not_of(" \t\r\n")-first+1U);
}
inline bool Number(const std::string& token,double& output)
{
    std::istringstream stream(token);stream.imbue(std::locale::classic());
    double value=0.0;stream>>std::noskipws>>value;
    if(!stream||!stream.eof()||!std::isfinite(value))return false;
    output=value;return true;
}
inline bool LoadFile(const char* path,Config& output,std::string& error)
{
    output=Config{};error="FILE_OPEN";
    errno=0;std::ifstream file(path,std::ios::binary);
    if(!file){error=errno==ENOENT?"FILE_MISSING":"FILE_OPEN";return false;}
    Config candidate{};
    const char* keys[]={"SchemaVersion","Enabled","ManualExternalPower","RequiredECode"};
    bool seen[4]{};std::size_t bytes=0,lines=0;std::string line;
    while(std::getline(file,line)){
        bytes+=line.size()+1U;
        if(bytes>4096U||++lines>64U||line.size()>256U){error="SIZE";return false;}
        if(lines==1U&&line.size()>=3U&&static_cast<unsigned char>(line[0])==0xefU&&
           static_cast<unsigned char>(line[1])==0xbbU&&static_cast<unsigned char>(line[2])==0xbfU)line.erase(0,3);
        const auto text=Trim(line);if(text.empty()||text[0]=='#'||text.compare(0,2,"//")==0)continue;
        const auto equal=text.find('=');if(equal==std::string::npos){error="FORMAT";return false;}
        const auto key=Trim(text.substr(0,equal)),token=Trim(text.substr(equal+1U));
        unsigned field=4U;for(unsigned i=0;i<4U;++i)if(key==keys[i]){field=i;break;}
        if(field==4U){error="UNKNOWN_KEY:"+key;return false;}
        if(seen[field]){error="DUPLICATE_KEY:"+key;return false;}seen[field]=true;
        double value=0.0;if(!Number(token,value)){error="NUMBER:"+key;return false;}
        {
            if(value<0.0||value>1000.0||std::floor(value)!=value){error="INTEGER:"+key;return false;}
        }
        switch(field){
        case 0:if(value!=1.0){error="SCHEMA";return false;}break;
        case 1:if(value>1.0){error="ENABLED";return false;}candidate.enabled=value==1.0;break;
        case 2:if(value>1.0){error="MANUAL_EXTERNAL_POWER";return false;}candidate.manualExternalPower=value==1.0;break;
        case 3:candidate.requiredECode=static_cast<std::uint32_t>(value);break;
        }
    }
    if(file.bad()){error="FILE_READ";return false;}
    for(unsigned i=0;i<4U;++i)if(!seen[i]){error=std::string("MISSING_KEY:")+keys[i];return false;}
    if(!Valid(candidate)){error="RANGE_OR_ACKNOWLEDGEMENT";return false;}
    output=candidate;error.clear();return true;
}
}
