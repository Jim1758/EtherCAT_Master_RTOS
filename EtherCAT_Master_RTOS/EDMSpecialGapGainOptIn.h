#pragma once
#include "EDMSpecialGapGainDefinition.h"
#include <fstream>
#include <string>
#include <cerrno>
#ifdef _WIN32
#include <Windows.h>
#else
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#endif

// EDM368 boot/background-loader only. A create-only, explicitly typed opt-in
// survives later condition saves without rewriting the owner's definitions.
namespace EDMSpecialGapGainOptIn
{
constexpr char Filename[]="SpecialGapGain.ini";
enum class State:std::uint8_t { Missing,Ready,Invalid };
inline std::string Path(const std::string& directory)
{
    if(directory.empty())return Filename;
    const auto last=directory.back();
#ifdef _WIN32
    return directory+(last=='/'||last=='\\'?"":"\\")+Filename;
#else
    return directory+(last=='/'?"":"/")+Filename;
#endif
}
inline std::string Bytes(const EDMRecipe::Catalog& c)
{
    if(c.schemaVersion!=2U||c.profileId!=1U||!c.catalogRevision||!c.definitionRevision)return {};
    std::string text="SchemaVersion=2\r\nMachineProfileId=1\r\nCatalogRevision="+std::to_string(c.catalogRevision)+
        "\r\nDefinitionRevision="+std::to_string(c.definitionRevision)+"\r\n";
    for(std::uint16_t id=EDMSpecialGapGainDefinition::FirstFieldId;id<=EDMSpecialGapGainDefinition::LastFieldId;++id)
        text+="Field"+std::to_string(id)+"="+EDMSpecialGapGainDefinition::Semantic(id)+","+
            EDMSpecialGapGainDefinition::Unit(id)+","+std::to_string(EDMSpecialGapGainDefinition::Decimals(id))+",0,"+
            (id==EDMSpecialGapGainDefinition::SwitchFieldId?"1":"1000")+"\r\n";
    return text;
}
inline State Read(const std::string& directory,const EDMRecipe::Catalog& c) noexcept
{
    try {
        const auto path=Path(directory);
#ifdef _WIN32
        // Use the same RTSS file APIs as EDMConditionStartupIO; do not add
        // a GetFileAttributesA import to the real-time executable.
        HANDLE probe=CreateFileA(path.c_str(),GENERIC_READ,
            FILE_SHARE_READ|FILE_SHARE_WRITE|FILE_SHARE_DELETE,nullptr,
            OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,nullptr);
        if(probe==INVALID_HANDLE_VALUE){const auto e=GetLastError();return
            e==ERROR_FILE_NOT_FOUND||e==ERROR_PATH_NOT_FOUND?State::Missing:State::Invalid;}
        if(!CloseHandle(probe))return State::Invalid;
#else
        struct stat info{};
        if(::stat(path.c_str(),&info)!=0)return errno==ENOENT?State::Missing:State::Invalid;
        if(!S_ISREG(info.st_mode))return State::Invalid;
#endif
        const auto expected=Bytes(c);if(expected.empty())return State::Invalid;
        std::ifstream input(path.c_str(),std::ios::binary);if(!input)return State::Invalid;
        std::string actual;char buffer[256];
        while(input){input.read(buffer,sizeof(buffer));const auto n=input.gcount();
            if(n<0||actual.size()+static_cast<std::size_t>(n)>1024U)return State::Invalid;
            actual.append(buffer,static_cast<std::size_t>(n));}
        return input.eof()&&!input.bad()&&actual==expected?State::Ready:State::Invalid;
    } catch (...) {return State::Invalid;}
}
inline bool Ensure(const std::string& directory,const EDMRecipe::Catalog& c) noexcept
{
    try {
        const auto state=Read(directory,c);if(state!=State::Missing)return state==State::Ready;
        const auto text=Bytes(c);if(text.empty())return false;
        const auto path=Path(directory);
#ifdef _WIN32
        HANDLE h=CreateFileA(path.c_str(),GENERIC_WRITE,FILE_SHARE_READ,nullptr,
            CREATE_NEW,FILE_ATTRIBUTE_NORMAL|FILE_FLAG_WRITE_THROUGH,nullptr);
        if(h==INVALID_HANDLE_VALUE){const auto e=GetLastError();return
            (e==ERROR_FILE_EXISTS||e==ERROR_ALREADY_EXISTS)&&Read(directory,c)==State::Ready;}
        bool okay=true;std::size_t position=0U;
        while(position<text.size()){
            DWORD done=0U;const DWORD count=static_cast<DWORD>(text.size()-position);
            if(!WriteFile(h,text.data()+position,count,&done,nullptr)||!done){okay=false;break;}
            position+=done;
        }
        if(!CloseHandle(h))okay=false;
#else
        int h=::open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL,0660);
        if(h<0)return errno==EEXIST&&Read(directory,c)==State::Ready;
        bool okay=true;std::size_t position=0U;
        while(position<text.size()){
            const auto done=::write(h,text.data()+position,text.size()-position);
            if(done<=0){okay=false;break;}position+=static_cast<std::size_t>(done);
        }
        if(::fsync(h)!=0)okay=false;
        if(::close(h)!=0)okay=false;
#endif
        return okay&&Read(directory,c)==State::Ready;
    } catch (...) {return false;}
}
}
