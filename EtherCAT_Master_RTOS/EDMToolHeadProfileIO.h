#pragma once

#include "EDMToolHeadProfile.h"
#include <cstdio>
#include <memory>
#include <new>

// BOOT / NON-RT ONLY. Strict bounded schema, no runtime reload and no fallback.
// The caller owns catalog publication. Parse/LoadFile clear output on failure.
namespace EDM87
{
constexpr std::size_t MaximumProfileFileBytes=32768U;
constexpr unsigned MaximumProfileFileLines=256U;
constexpr unsigned MaximumProfileLineBytes=256U;
enum class ProfileError : unsigned
{
    None=0U, InvalidArgument, FileOpen, FileRead, Allocation, FileTooLarge,
    EmbeddedNul, TooManyLines, LineTooLong, Syntax, Number, Range,
    MissingRecord, DuplicateProfile, InvalidProfile, InvalidCatalog, TrailingData
};
struct Diagnostic
{
    ProfileError error=ProfileError::None;
    unsigned line=0U;
};
inline const char* ErrorName(ProfileError error) noexcept
{
    switch(error)
    {
    case ProfileError::None:return "OK";
    case ProfileError::InvalidArgument:return "INVALID_ARGUMENT";
    case ProfileError::FileOpen:return "FILE_OPEN";
    case ProfileError::FileRead:return "FILE_READ";
    case ProfileError::Allocation:return "ALLOCATION";
    case ProfileError::FileTooLarge:return "FILE_TOO_LARGE";
    case ProfileError::EmbeddedNul:return "EMBEDDED_NUL";
    case ProfileError::TooManyLines:return "TOO_MANY_LINES";
    case ProfileError::LineTooLong:return "LINE_TOO_LONG";
    case ProfileError::Syntax:return "SYNTAX";
    case ProfileError::Number:return "NUMBER";
    case ProfileError::Range:return "RANGE";
    case ProfileError::MissingRecord:return "MISSING_RECORD";
    case ProfileError::DuplicateProfile:return "DUPLICATE_PROFILE";
    case ProfileError::InvalidProfile:return "INVALID_PROFILE";
    case ProfileError::InvalidCatalog:return "INVALID_CATALOG";
    case ProfileError::TrailingData:return "TRAILING_DATA";
    default:return "UNKNOWN_ERROR";
    }
}
namespace ProfileIODetail
{
inline bool Fail(Diagnostic& diagnostic,ProfileError error,unsigned line) noexcept
{ diagnostic.error=error; diagnostic.line=line; return false; }
inline bool Space(char c) noexcept { return c==' ' || c=='\t'; }
inline bool Digit(char c) noexcept { return c>='0' && c<='9'; }
struct Token { const char* begin=nullptr; unsigned size=0U; };
inline bool Equal(const Token& token,const char* text) noexcept
{
    unsigned index=0U;
    for(;text[index]!='\0';++index)
        if(index>=token.size || token.begin[index]!=text[index]) return false;
    return index==token.size;
}
inline bool Integer(const Token& token,std::uint64_t maximum,std::uint64_t& result) noexcept
{
    if(token.size==0U || token.size>10U) return false;
    std::uint64_t value=0U;
    for(unsigned index=0U;index<token.size;++index)
    {
        if(!Digit(token.begin[index])) return false;
        const auto digit=static_cast<std::uint64_t>(token.begin[index]-'0');
        if(digit>maximum || value>(maximum-digit)/10U) return false;
        value=value*10U+digit;
    }
    result=value; return true;
}
inline bool Number(const Token& token,double& result) noexcept
{
    // ASCII decimal only; independent of current C/C++ locale. No hex, NaN,
    // infinity, comma decimal, trailing suffix, or overflow/underflow-to-zero.
    if(token.size==0U || token.size>64U) return false;
    unsigned index=0U,digits=0U,fraction=0U; bool negative=false;
    if(token.begin[index]=='+' || token.begin[index]=='-')
    { negative=token.begin[index]=='-'; if(++index==token.size) return false; }
    long double significand=0.0L;
    while(index<token.size && Digit(token.begin[index]))
    { significand=significand*10.0L+(token.begin[index++]-'0'); ++digits; }
    if(index<token.size && token.begin[index]=='.')
    {
        ++index;
        while(index<token.size && Digit(token.begin[index]))
        { significand=significand*10.0L+(token.begin[index++]-'0'); ++digits; ++fraction; }
    }
    if(digits==0U) return false;
    int exponent=0;
    if(index<token.size && (token.begin[index]=='e' || token.begin[index]=='E'))
    {
        ++index; bool exponentNegative=false;
        if(index<token.size && (token.begin[index]=='+' || token.begin[index]=='-'))
        { exponentNegative=token.begin[index]=='-'; ++index; }
        const unsigned start=index;
        while(index<token.size && Digit(token.begin[index]))
        {
            exponent=exponent*10+(token.begin[index++]-'0');
            if(exponent>400) return false;
        }
        if(index==start) return false;
        if(exponentNegative) exponent=-exponent;
    }
    if(index!=token.size) return false;
    if(significand==0.0L) { result=0.0; return true; }
    exponent-=static_cast<int>(fraction);
    // Scale in steps: MSVC long double is binary64, so a single pow(10,-400)
    // could underflow before multiplying a representable large significand.
    long double scaled=significand;
    while(exponent>0)
    { const int step=exponent>100?100:exponent; scaled*=std::pow(10.0L,step); exponent-=step; }
    while(exponent<0)
    { const int step=exponent< -100?100:-exponent; scaled*=std::pow(10.0L,-step); exponent+=step; }
    const double value=static_cast<double>(negative?-scaled:scaled);
    if(!EDM81::Finite(value) || value==0.0) return false;
    result=value; return true;
}
struct Reader
{
    const char* bytes=nullptr;
    std::size_t size=0U,offset=0U;
    unsigned line=0U,count=0U;
    std::array<Token,12U> tokens{};
    bool Next(Diagnostic& diagnostic) noexcept
    {
        count=0U;
        while(offset<size)
        {
            ++line;
            if(line>MaximumProfileFileLines) return Fail(diagnostic,ProfileError::TooManyLines,line);
            const std::size_t start=offset;
            while(offset<size && bytes[offset]!='\n') ++offset;
            std::size_t end=offset;
            if(offset<size) ++offset;
            if(end>start && bytes[end-1U]=='\r') --end;
            if(end-start>MaximumProfileLineBytes) return Fail(diagnostic,ProfileError::LineTooLong,line);
            std::size_t cursor=start;
            while(cursor<end && Space(bytes[cursor])) ++cursor;
            if(cursor==end || bytes[cursor]=='#' || bytes[cursor]==';') continue;
            while(cursor<end)
            {
                if(count>=tokens.size()) return Fail(diagnostic,ProfileError::Syntax,line);
                const std::size_t tokenStart=cursor;
                while(cursor<end && !Space(bytes[cursor])) ++cursor;
                tokens[count++]={bytes+tokenStart,static_cast<unsigned>(cursor-tokenStart)};
                while(cursor<end && Space(bytes[cursor])) ++cursor;
            }
            return true;
        }
        return false;
    }
    bool Record(const char* name,unsigned expected,Diagnostic& diagnostic) noexcept
    {
        if(!Next(diagnostic))
        {
            if(diagnostic.error!=ProfileError::None) return false;
            return Fail(diagnostic,ProfileError::MissingRecord,line+1U);
        }
        return (count==expected && Equal(tokens[0],name)) ||
            Fail(diagnostic,ProfileError::Syntax,line);
    }
    bool Vector(unsigned first,EDM86::Vector3& vector,Diagnostic& diagnostic) noexcept
    {
        for(unsigned axis=0U;axis<3U;++axis)
            if(!Number(tokens[first+axis],vector[axis])) return Fail(diagnostic,ProfileError::Number,line);
        return true;
    }
};
}
inline bool Parse(const char* bytes,std::size_t size,Catalog& output,Diagnostic& diagnostic) noexcept
{
    output=Catalog{}; diagnostic=Diagnostic{};
    using namespace ProfileIODetail;
    if(bytes==nullptr) return Fail(diagnostic,ProfileError::InvalidArgument,0U);
    if(size>MaximumProfileFileBytes) return Fail(diagnostic,ProfileError::FileTooLarge,0U);
    for(std::size_t index=0U;index<size;++index)
        if(bytes[index]=='\0') return Fail(diagnostic,ProfileError::EmbeddedNul,0U);
    Reader reader{}; reader.bytes=bytes; reader.size=size;
    Catalog parsed{};
    if(!reader.Record("EDM_TOOL_HEAD_PROFILES",2U,diagnostic)) return false;
    if(!Equal(reader.tokens[1],"1")) return Fail(diagnostic,ProfileError::Syntax,reader.line);
    if(!reader.Record("CATALOG_REVISION",2U,diagnostic)) return false;
    if(!Integer(reader.tokens[1],MaximumRevision,parsed.revision) || parsed.revision==0U)
        return Fail(diagnostic,ProfileError::Range,reader.line);
    unsigned ids=0U;
    for(;;)
    {
        if(!reader.Next(diagnostic))
        {
            if(diagnostic.error!=ProfileError::None) return false;
            return Fail(diagnostic,ProfileError::MissingRecord,reader.line+1U);
        }
        if(Equal(reader.tokens[0],"END_CATALOG"))
        {
            if(reader.count!=1U) return Fail(diagnostic,ProfileError::Syntax,reader.line);
            break;
        }
        if(reader.count!=3U || !Equal(reader.tokens[0],"PROFILE"))
            return Fail(diagnostic,ProfileError::Syntax,reader.line);
        if(parsed.count>=MaximumProfiles) return Fail(diagnostic,ProfileError::Range,reader.line);
        std::uint64_t id=0U; HeadProfile profile{};
        if(!Integer(reader.tokens[1],MaximumProfiles,id) || id==0U ||
            !Integer(reader.tokens[2],MaximumRevision,profile.revision) || profile.revision==0U)
            return Fail(diagnostic,ProfileError::Range,reader.line);
        profile.id=static_cast<unsigned>(id);
        const unsigned bit=1U<<(profile.id-1U);
        if(ids&bit) return Fail(diagnostic,ProfileError::DuplicateProfile,reader.line);
        ids|=bit;
        if(!reader.Record("TIP_MM",4U,diagnostic) || !reader.Vector(1U,profile.toolTipMm,diagnostic) ||
            !reader.Record("DIRECTION",4U,diagnostic) || !reader.Vector(1U,profile.directionUnit,diagnostic))
            return false;
        for(unsigned index=0U;index<3U;++index)
        {
            if(!reader.Record("JOINT",11U,diagnostic)) return false;
            std::uint64_t jointIndex=0U;
            if(!Integer(reader.tokens[1],3U,jointIndex) || jointIndex!=index+1U)
                return Fail(diagnostic,ProfileError::Range,reader.line);
            auto& joint=profile.chain.joints[index];
            if(Equal(reader.tokens[2],"C")) joint.channel=3U;
            else if(Equal(reader.tokens[2],"U")) joint.channel=4U;
            else if(Equal(reader.tokens[2],"V")) joint.channel=5U;
            else return Fail(diagnostic,ProfileError::Syntax,reader.line);
            if(!reader.Vector(3U,joint.axis,diagnostic) || !reader.Vector(6U,joint.parentToPivotMm,diagnostic))
                return false;
            if(Equal(reader.tokens[9],"1") || Equal(reader.tokens[9],"+1")) joint.sign=1;
            else if(Equal(reader.tokens[9],"-1")) joint.sign=-1;
            else return Fail(diagnostic,ProfileError::Range,reader.line);
            if(!Number(reader.tokens[10],joint.zeroAngleDeg))
                return Fail(diagnostic,ProfileError::Number,reader.line);
        }
        if(!reader.Record("END_PROFILE",1U,diagnostic)) return false;
        if(!ValidProfile(profile)) return Fail(diagnostic,ProfileError::InvalidProfile,reader.line);
        parsed.profiles[parsed.count++]=profile;
    }
    if(reader.Next(diagnostic)) return Fail(diagnostic,ProfileError::TrailingData,reader.line);
    if(diagnostic.error!=ProfileError::None) return false;
    if(!ValidCatalog(parsed)) return Fail(diagnostic,ProfileError::InvalidCatalog,reader.line);
    output=parsed; return true;
}
inline bool LoadFile(const char* path,Catalog& output,Diagnostic& diagnostic) noexcept
{
    output=Catalog{}; diagnostic=Diagnostic{};
    using namespace ProfileIODetail;
    if(path==nullptr || path[0]=='\0') return Fail(diagnostic,ProfileError::InvalidArgument,0U);
    std::unique_ptr<char[]> bytes(new(std::nothrow) char[MaximumProfileFileBytes+1U]);
    if(!bytes) return Fail(diagnostic,ProfileError::Allocation,0U);
    std::FILE* file=nullptr;
#if defined(_MSC_VER)
    if(fopen_s(&file,path,"rb")!=0) file=nullptr;
#else
    file=std::fopen(path,"rb");
#endif
    if(file==nullptr) return Fail(diagnostic,ProfileError::FileOpen,0U);
    const std::size_t size=std::fread(bytes.get(),1U,MaximumProfileFileBytes+1U,file);
    const bool readFailed=std::ferror(file)!=0;
    const bool closeFailed=std::fclose(file)!=0;
    if(readFailed || closeFailed) return Fail(diagnostic,ProfileError::FileRead,0U);
    return Parse(bytes.get(),size,output,diagnostic);
}
}
