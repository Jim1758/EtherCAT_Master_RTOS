#include "HoldConfig.h"
#include <array>
#include <cerrno>
#include <fstream>
#include <iterator>
#include <locale>
#include <sstream>

namespace NCHoldRecovery
{
namespace
{
constexpr std::size_t MaximumFileBytes=16384U;
std::string Trim(const std::string& s)
{
    const auto first=s.find_first_not_of(" \t\r\n");
    return first==std::string::npos?std::string{}:s.substr(first,s.find_last_not_of(" \t\r\n")-first+1U);
}
bool Number(const std::string& text,double& value)
{
    std::istringstream input(text);input.imbue(std::locale::classic());
    double candidate=0.0;input>>candidate;
    if(input.fail())return false;
    input>>std::ws;
    if(!input.eof()||!std::isfinite(candidate))return false;
    value=candidate;return true;
}
}
bool ParseText(const std::string& text,Config& output,std::string& errorKey)
{
    errorKey="FILE_SIZE";if(text.size()>MaximumFileBytes)return false;
    Config candidate{};
    const std::array<const char*,10U> names{{"retreatFeedMmMin","resumeFeedMmMin","jogRecoveryFeedMmMin",
        "retreatAccelSeconds","retreatDecelSeconds","resumeAccelSeconds","resumeDecelSeconds",
        "jogAccelSeconds","jogDecelSeconds","historyMaxPrevious"}};
    const std::array<double*,9U> targets{{&candidate.retreatFeedMmMin,&candidate.resumeFeedMmMin,
        &candidate.jogRecoveryFeedMmMin,&candidate.retreatAccelSeconds,&candidate.retreatDecelSeconds,
        &candidate.resumeAccelSeconds,&candidate.resumeDecelSeconds,&candidate.jogAccelSeconds,&candidate.jogDecelSeconds}};
    std::array<bool,10U> seen{};
    const bool bom=text.size()>=3U&&static_cast<unsigned char>(text[0])==0xEFU&&
        static_cast<unsigned char>(text[1])==0xBBU&&static_cast<unsigned char>(text[2])==0xBFU;
    std::istringstream input(bom?text.substr(3U):text);std::string line;
    while(std::getline(input,line)){
        const auto comment=line.find_first_of("#;");if(comment!=std::string::npos)line.erase(comment);
        line=Trim(line);if(line.empty()||line.compare(0U,2U,"//")==0)continue;
        const auto equal=line.find('=');
        errorKey="ASSIGNMENT";
        if(equal==std::string::npos||line.find('=',equal+1U)!=std::string::npos)return false;
        const auto key=Trim(line.substr(0U,equal));std::size_t field=0U;
        while(field<names.size()&&key!=names[field])++field;
        errorKey=key.empty()?"EMPTY_KEY":key;
        if(field==names.size()||seen[field])return false;
        double value=0.0;if(!Number(Trim(line.substr(equal+1U)),value))return false;
        if(field==9U){if(value!=15.0)return false;candidate.historyMaxPrevious=15U;}
        else *targets[field]=value;
        seen[field]=true;
    }
    errorKey="PROFILE";if(!Valid(candidate))return false;
    output=candidate;errorKey.clear();return true;
}
bool LoadFile(const std::string& path,Config& output,std::string& errorKey)
{
    errno=0;std::ifstream file(path.c_str(),std::ios::in|std::ios::binary);
    if(!file.is_open()){
        if(errno==ENOENT){output=Config{};errorKey.clear();return true;}
        errorKey="FILE_OPEN";return false;
    }
    std::string text(MaximumFileBytes+1U,'\0');
    file.read(&text[0],static_cast<std::streamsize>(text.size()));
    if(file.bad()){errorKey="FILE_READ";return false;}
    text.resize(static_cast<std::size_t>(file.gcount()));
    return ParseText(text,output,errorKey);
}
}
