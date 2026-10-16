#pragma once
#include <cstdint>

// EDM137 NC-owned output intent. This stage has no production discharge
// admission: all diagnostic motion sessions remain physically inhibited.
// A logical request follows the admitted session, never the current NC row
// or an empty look-ahead queue. The PLC image is published as one word.
namespace EDM137PLC
{
constexpr unsigned FlushBit=1U, DischargeBit=2U, PolarityBit=4U;
constexpr bool ProductionDischargeAdmitted=false;
constexpr bool ReservedPoint(int point) noexcept
{return point==80||point==92||point==93;}
constexpr unsigned PointBit(int point) noexcept
{return point==80?FlushBit:point==92?DischargeBit:point==93?PolarityBit:0U;}
struct Input
{
    std::uint64_t session=0ULL,epoch=0ULL,owner=0ULL,recipe=0ULL;
    bool admitted=false,machiningAccepted=false,flushActive=false,polarity=false;
    bool stop=false,unavailable=false;
};
struct Output
{
    unsigned word=0U;
    bool requested=false;
};
class State
{
public:
    const Output& Observe(const Input& input) noexcept
    {
        if(input.stop)return Clear();
        // A failed nonblocking read does not revoke an otherwise owned session.
        // Retain the last coherent status only for the exact live frozen identity.
        // This cannot start a session, change polarity or admit discharge.
        if(input.unavailable) {
            if(!input.admitted||!session_||input.session!=session_||input.epoch!=epoch_||
                input.owner!=owner_||input.recipe!=recipe_||input.polarity!=polarity_||revoked_)return Clear();
            return output_;
        }
        if(!input.admitted||!input.session||!input.epoch||!input.owner||!input.recipe)
            return Clear();
        if(input.session<=retiredThrough_||(session_&&input.session<session_))return Clear();
        if(session_!=input.session) {
            Clear();session_=input.session;epoch_=input.epoch;owner_=input.owner;recipe_=input.recipe;
            polarity_=input.polarity;
        }
        // An existing session cannot silently adopt a changed frozen recipe,
        // owner, epoch or polarity. Its subsequent observations stay blocked.
        if(revoked_||epoch_!=input.epoch||owner_!=input.owner||recipe_!=input.recipe||polarity_!=input.polarity) {
            revoked_=true;output_=Output{};return output_;
        }
        started_=started_||input.machiningAccepted;
        output_.requested=started_;
        output_.word=(started_&&input.flushActive?FlushBit:0U)|(polarity_?PolarityBit:0U);
        if(ProductionDischargeAdmitted&&output_.requested)output_.word|=DischargeBit;
        return output_;
    }
    const Output& Clear() noexcept
    {if(session_>retiredThrough_)retiredThrough_=session_;
     session_=epoch_=owner_=recipe_=0ULL;started_=polarity_=revoked_=false;output_=Output{};return output_;}
    const Output& Current() const noexcept{return output_;}
    std::uint64_t Session() const noexcept{return session_;}
private:
    std::uint64_t session_=0ULL,epoch_=0ULL,owner_=0ULL,recipe_=0ULL,retiredThrough_=0ULL;
    bool started_=false,polarity_=false,revoked_=false;
    Output output_{};
};
static_assert(!ProductionDischargeAdmitted,"EDM137 contains no energized-production admission");
}
