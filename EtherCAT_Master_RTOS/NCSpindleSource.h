#pragma once
#include "GCodeParser.h"
#include "NCSpindleCodes.h"
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>

// EDM287: NC-owner-thread source decoder only, not a runtime dispatch permit.
// Parse the immutable authored row BEFORE any setting/tool/macro/PLC side effect.
// S belongs only to one explicit spindle M row. It is output-shaft RPM and is
// never an inch/mm value, G-code feed, global modal S, or motor-shaft RPM.
// Existing G65/G66 and EDM tuning S meanings remain owned by their handlers.
namespace NCSpindleSource
{
constexpr bool RuntimeConnected=false;
enum class Status : unsigned char
{ Ok,InvalidSource,HiddenField,UnsupportedBlock,NonLiteral,InvalidSpeed,ResolvedMismatch };
struct Words
{
    int mCode=-1;
    unsigned spindle=0U;
    int direction=0;
    double rpm=0.0;
    bool stop=false,valid=false;
};
inline bool IsReservedM(int m) noexcept {return NCSpindleCodes::IsReserved(m);}
// Classification only: inspect every stored slot so an inconsistent count
// cannot hide a reserved M from the NC/PLC boundary. This grants no authority.
inline bool HasReservedM(const NCBlock& block) noexcept
{
    for(int m:block.mCode)if(IsReservedM(m))return true;
    return false;
}
// G+S remains owned by the authored G handler. A bare S row has no spindle
// meaning and must not inherit a previous M or speed-setting mode.
inline bool IsStandaloneSpeed(const NCBlock& block) noexcept
{ return block.has('S')&&!block.hasG&&block.gCount==0&&block.mCount==0; }
namespace Detail
{
inline bool Integer(const std::string& expression,int& value) noexcept
{
    if(expression.empty()||expression.find('\0')!=std::string::npos)return false;
    const char* p=expression.c_str();const char* const finish=p+expression.size();
    while(p!=finish&&(*p==' '||*p=='\t'||*p=='\r'||*p=='\n'))++p;
    if(p!=finish&&*p=='+')++p;
    const char* const digits=p;int integer=0;
    while(p!=finish&&*p>='0'&&*p<='9'){
        const int digit=*p-'0';
        if(integer>((std::numeric_limits<int>::max)()-digit)/10)return false;
        integer=integer*10+digit;++p;
    }
    if(p==digits)return false;
    // Do not round an authored fractional M/N into a different command.
    // M003 and M3.000 are exact; M3.00000000000000001 is not M3.
    if(p!=finish&&*p=='.'){
        ++p;while(p!=finish&&*p=='0')++p;
    }
    while(p!=finish&&(*p==' '||*p=='\t'||*p=='\r'||*p=='\n'))++p;
    if(p!=finish)return false;
    value=integer;return true;
}
inline bool Literal(const std::string& expression,double& value) noexcept
{
    if(expression.empty()||expression.find('\0')!=std::string::npos)return false;
    // Decimal literals only. Do not let strtod reinterpret hex, macro syntax,
    // NaN/Inf or a prefix of a longer expression as spindle authority.
    const char* p=expression.c_str();const char* const finish=p+expression.size();
    while(p!=finish&&(*p==' '||*p=='\t'||*p=='\r'||*p=='\n'))++p;
    if(p!=finish&&(*p=='+'||*p=='-'))++p;
    bool digit=false;
    while(p!=finish&&*p>='0'&&*p<='9'){digit=true;++p;}
    if(p!=finish&&*p=='.'){
        ++p;while(p!=finish&&*p>='0'&&*p<='9'){digit=true;++p;}
    }
    if(!digit)return false;
    if(p!=finish&&(*p=='e'||*p=='E')){
        ++p;if(p!=finish&&(*p=='+'||*p=='-'))++p;
        const char* const exponent=p;
        while(p!=finish&&*p>='0'&&*p<='9')++p;
        if(p==exponent)return false;
    }
    while(p!=finish&&(*p==' '||*p=='\t'||*p=='\r'||*p=='\n'))++p;
    if(p!=finish)return false;
    errno=0;char* end=nullptr;const char* begin=expression.c_str();
    const double number=std::strtod(begin,&end);
    if(begin==end||errno==ERANGE||!std::isfinite(number))return false;
    while(end!=finish&&(*end==' '||*end=='\t'||*end=='\r'||*end=='\n'))++end;
    if(end!=finish)return false;
    value=number;return true;
}
}
// Routing evidence only. The ordinary resolver rounds a numeric M within
// 1e-9 of an integer; keep that authored row on the reserved lane even if a
// different resolved value is presented later. Decode still requires exact
// lexical integers and will reject every nonzero fractional suffix.
// Dynamic expressions are not evaluated here: their resolved value supplies
// the second classification guard, preserving ordinary dynamic PLC M codes.
inline bool HasReservedM(const NCParsedBlock& parsed) noexcept
{
    for(const auto& expression:parsed.mExpressions){
        int exact=0;
        if(Detail::Integer(expression,exact)){
            if(IsReservedM(exact))return true;
            continue;
        }
        double value=0.0;
        if(!Detail::Literal(expression,value))continue;
        const double rounded=std::round(value);
        if(std::fabs(value-rounded)<=1.e-9&&
            rounded>=static_cast<double>((std::numeric_limits<int>::min)())&&
            rounded<=static_cast<double>((std::numeric_limits<int>::max)())&&
            IsReservedM(static_cast<int>(rounded)))return true;
    }
    return false;
}
inline bool IsStandaloneSpeed(const NCParsedBlock& parsed) noexcept
{ return parsed.has('S')&&parsed.gCount==0&&parsed.mCount==0; }
inline Status Decode(const NCParsedBlock& parsed,Words& output) noexcept
{
    output=Words{};
    if(parsed.error!=NCParseError::NONE||parsed.isEmpty||parsed.isBlockSkip||
        parsed.dependsOnMacroState||parsed.duplicateAddressMask!=0U||
        parsed.controlType!=NCParsedControlType::NONE||parsed.gCount!=0||parsed.mCount!=1)
        return Status::InvalidSource;
    if(parsed.assignment.prefix!=0||!parsed.assignment.indexExpression.empty()||
        !parsed.assignment.valueExpression.empty()||parsed.gotoStatement.conditional||
        !parsed.gotoStatement.conditionExpression.empty()||!parsed.gotoStatement.targetExpression.empty())
        return Status::HiddenField;
    for(const auto& expression:parsed.gExpressions)if(!expression.empty())return Status::HiddenField;
    for(unsigned i=1U;i<parsed.mExpressions.size();++i)
        if(!parsed.mExpressions[i].empty())return Status::HiddenField;
    for(char word='A';word<='Z';++word){
        if(!parsed.has(word)&&!parsed.expression(word).empty())return Status::HiddenField;
        if(parsed.has(word)&&word!='N'&&word!='S')return Status::UnsupportedBlock;
    }
    int m=0;
    if(!Detail::Integer(parsed.mExpressions[0],m))return Status::NonLiteral;
    const auto code=NCSpindleCodes::Decode(m);
    if(!code.valid)return Status::UnsupportedBlock;
    if(parsed.has('N')){
        int n=0;
        if(!Detail::Integer(parsed.expression('N'),n))return Status::NonLiteral;
    }
    double rpm=0.0;
    if(code.stop){if(parsed.has('S'))return Status::UnsupportedBlock;}
    else if(!parsed.has('S')||!Detail::Literal(parsed.expression('S'),rpm)||rpm<=0.0)return Status::InvalidSpeed;
    Words result{};result.mCode=m;result.spindle=code.spindle;
    result.direction=code.direction;result.stop=code.stop;
    result.rpm=rpm;result.valid=true;output=result;return Status::Ok;
}
// EDM288: a resolved M may have been rounded by the ordinary expression
// resolver. Decode the immutable authored row first, then compare its entire
// canonical value (including unused slots) before any G/T/PLC side effect.
inline Status ValidateResolved(const NCParsedBlock& parsed,const NCBlock& block,Words& output) noexcept
{
    output=Words{};Words words{};
    const auto decoded=Decode(parsed,words);
    if(decoded!=Status::Ok)return decoded;
    NCBlock expected{};
    expected.isEmpty=false;expected.mCount=1;expected.mCode[0]=words.mCode;
    if(!words.stop){expected.hasParam['S'-'A']=true;expected.param['S'-'A']=words.rpm;}
    if(parsed.has('N')){
        int n=0;
        if(!Detail::Integer(parsed.expression('N'),n))return Status::NonLiteral;
        expected.hasParam['N'-'A']=true;expected.param['N'-'A']=static_cast<double>(n);
    }
    if(block.isEmpty!=expected.isEmpty||block.isBlockSkip!=expected.isBlockSkip||
        block.isGoto!=expected.isGoto||block.gotoTarget!=expected.gotoTarget||
        block.hasG!=expected.hasG||block.gCount!=expected.gCount||block.gCode!=expected.gCode||
        block.mCount!=expected.mCount)return Status::ResolvedMismatch;
    for(unsigned i=0U;i<NC_MAX_G_CODES_PER_BLOCK;++i)
        if(block.gCodes[i]!=expected.gCodes[i])return Status::ResolvedMismatch;
    for(unsigned i=0U;i<NC_MAX_M_CODES_PER_BLOCK;++i)
        if(block.mCode[i]!=expected.mCode[i])return Status::ResolvedMismatch;
    for(unsigned i=0U;i<26U;++i)
        if(block.hasParam[i]!=expected.hasParam[i]||
            std::memcmp(&block.param[i],&expected.param[i],sizeof(double))!=0)return Status::ResolvedMismatch;
    output=words;return Status::Ok;
}

// EDM295: bounded literal MEMORY program vocabulary for a disjoint XYZ feed
// lane beside the configured C/U/V velocity lanes. This is source shape only;
// the NC and RT owners must independently prove frame, axis mask and authority.
enum class ProgramRowKind : unsigned char
{ Invalid,Empty,Spindle,Dwell,Control,Setup,XYZFeed };
inline bool NeutralProgramSetup(int g) noexcept
{
    switch(g){
    case 21:case 90:case 91:case 17:case 15:case 40:case 49:
    case 69:case 169:case 50:case 150:case 163:return true;
    default:return false;
    }
}
inline ProgramRowKind ClassifyProgramRow(const NCParsedBlock& row) noexcept
{
    if(row.error!=NCParseError::NONE||row.isBlockSkip||row.dependsOnMacroState||row.duplicateAddressMask||
       row.controlType!=NCParsedControlType::NONE||row.assignment.prefix||!row.assignment.indexExpression.empty()||
       !row.assignment.valueExpression.empty()||row.gotoStatement.conditional||
       !row.gotoStatement.conditionExpression.empty()||!row.gotoStatement.targetExpression.empty())return ProgramRowKind::Invalid;
    if(row.isEmpty){
        if(row.gCount||row.mCount)return ProgramRowKind::Invalid;
        for(const auto& expression:row.gExpressions)if(!expression.empty())return ProgramRowKind::Invalid;
        for(const auto& expression:row.mExpressions)if(!expression.empty())return ProgramRowKind::Invalid;
        for(char word='A';word<='Z';++word)
            if(row.has(word)||!row.expression(word).empty())return ProgramRowKind::Invalid;
        return ProgramRowKind::Empty;
    }
    if(HasReservedM(row)){
        Words words{};return Decode(row,words)==Status::Ok?ProgramRowKind::Spindle:ProgramRowKind::Invalid;
    }
    if(row.gCount<0||row.gCount>1||row.mCount<0||row.mCount>1||(row.gCount&&row.mCount))return ProgramRowKind::Invalid;
    for(unsigned i=0U;i<row.gExpressions.size();++i)
        if(i>=static_cast<unsigned>(row.gCount)&&!row.gExpressions[i].empty())return ProgramRowKind::Invalid;
    for(unsigned i=0U;i<row.mExpressions.size();++i)
        if(i>=static_cast<unsigned>(row.mCount)&&!row.mExpressions[i].empty())return ProgramRowKind::Invalid;
    int g=-1,m=-1;
    if(row.gCount&&!Detail::Integer(row.gExpressions[0],g))return ProgramRowKind::Invalid;
    if(row.mCount&&!Detail::Integer(row.mExpressions[0],m))return ProgramRowKind::Invalid;
    ProgramRowKind kind=ProgramRowKind::Empty;
    if(row.gCount){
        kind=g==1?ProgramRowKind::XYZFeed:g==4?ProgramRowKind::Dwell:
            NeutralProgramSetup(g)?ProgramRowKind::Setup:ProgramRowKind::Invalid;
    }else if(row.mCount){
        kind=(m==0||m==1||m==2||m==30)?ProgramRowKind::Control:ProgramRowKind::Invalid;
    }
    if(kind==ProgramRowKind::Invalid)return kind;
    unsigned xyz=0U;bool feed=false;
    for(char word='A';word<='Z';++word){
        if(!row.has(word)){if(!row.expression(word).empty())return ProgramRowKind::Invalid;continue;}
        if(word=='N'){
            int n=0;if(!Detail::Integer(row.expression(word),n))return ProgramRowKind::Invalid;
        }else if(kind==ProgramRowKind::XYZFeed&&(word=='X'||word=='Y'||word=='Z'||word=='F')){
            double value=0.0;if(!Detail::Literal(row.expression(word),value))return ProgramRowKind::Invalid;
            if(word=='F'){if(value<=0.0||value>100.0)return ProgramRowKind::Invalid;feed=true;}
            else xyz|=1U<<static_cast<unsigned>(word-'X');
        }else if(kind==ProgramRowKind::Dwell&&(word=='X'||word=='P')){
            double duration=0.0;if(!Detail::Literal(row.expression(word),duration)||duration<0.0)return ProgramRowKind::Invalid;
        }else return ProgramRowKind::Invalid;
    }
    if(kind==ProgramRowKind::XYZFeed&&(!xyz||!feed))return ProgramRowKind::Invalid;
    if(kind==ProgramRowKind::Dwell&&row.has('X')&&row.has('P'))return ProgramRowKind::Invalid;
    return kind;
}
inline bool ValidateProgramResolved(const NCParsedBlock& row,const NCBlock& block) noexcept
{
    const auto kind=ClassifyProgramRow(row);
    if(kind==ProgramRowKind::Invalid)return false;
    if(kind==ProgramRowKind::Spindle){Words words{};return ValidateResolved(row,block,words)==Status::Ok;}
    NCBlock expected{};expected.isEmpty=row.isEmpty;
    if(row.gCount){int g=0;if(!Detail::Integer(row.gExpressions[0],g))return false;
        expected.hasG=true;expected.gCount=1;expected.gCode=g;expected.gCodes[0]=g;}
    if(row.mCount){int m=0;if(!Detail::Integer(row.mExpressions[0],m))return false;
        expected.mCount=1;expected.mCode[0]=m;}
    for(char word='A';word<='Z';++word)if(row.has(word)){
        double value=0.0;if(!Detail::Literal(row.expression(word),value))return false;
        expected.hasParam[word-'A']=true;expected.param[word-'A']=value;
    }
    if(block.isEmpty!=expected.isEmpty||block.isBlockSkip||block.isGoto||block.gotoTarget!=expected.gotoTarget||
       block.hasG!=expected.hasG||block.gCount!=expected.gCount||block.gCode!=expected.gCode||block.mCount!=expected.mCount)return false;
    for(unsigned i=0U;i<NC_MAX_G_CODES_PER_BLOCK;++i)if(block.gCodes[i]!=expected.gCodes[i])return false;
    for(unsigned i=0U;i<NC_MAX_M_CODES_PER_BLOCK;++i)if(block.mCode[i]!=expected.mCode[i])return false;
    for(unsigned i=0U;i<26U;++i)if(block.hasParam[i]!=expected.hasParam[i]||
        std::memcmp(&block.param[i],&expected.param[i],sizeof(double)))return false;
    return true;
}
}
