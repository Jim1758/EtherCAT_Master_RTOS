#pragma once

// EDM287 architecture: one mapping shared by source validation and the pure
// three-spindle engine. These values do not register runtime M handlers.
namespace NCSpindleCodes
{
struct Code
{
    unsigned spindle=0U; // logical spindle index; physical axis comes from configuration
    int direction=0;           // logical axis direction; hardware reversal is downstream
    bool stop=false,valid=false;
};
inline Code Decode(int m) noexcept
{
    Code code{};
    switch(m){
    case 3: case 4: case 5: code.spindle=0U;code.direction=m==3?1:m==4?-1:0;break;
    case 13: case 14: case 15: code.spindle=1U;code.direction=m==13?1:m==14?-1:0;break;
    case 16: case 17: case 18: code.spindle=2U;code.direction=m==16?1:m==17?-1:0;break;
    default:return code;
    }
    code.stop=code.direction==0;code.valid=true;return code;
}
inline bool IsReserved(int m) noexcept {return Decode(m).valid;}
}
