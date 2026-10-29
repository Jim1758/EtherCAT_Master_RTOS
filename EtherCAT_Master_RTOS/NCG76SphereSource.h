#pragma once

#include "NCPreparedBlockQueueShadow.h"
#include "NCG76SphereCoordinates.h"
#include "MotionExecutionContract.h"

#include <array>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <string>

// EDM285: P50 source interpretation only. This header is deliberately separate
// from the ordinary G76 mode registry, dispatch, recipes and Motion producers.
// No function here arms a process, writes a PLC output, or grants execution.
//
// Phase 1 accepts G21 only. R is radius in mm; K is layer spacing in mm, NEVER
// the contour angle used by older P modes. F is an explicit positive mm/min
// literal. The selected radius law is Linear: rho(s)=max(0,R-s), with the
// planner owning s=i*K and its exact terminal s=R rule. No NC word selects a
// different law. EDM286: omitted B inherits the last COMMITTED B in the same
// run/cache/owner scope. With no usable committed source it is unresolved, never
// an implicit B0. Explicit B0..B4 override that source. Both forms remain data
// and confer no execution/discharge authority.
namespace NCG76SphereSource
{
constexpr unsigned ModeP = 50U;

enum class Status : unsigned char
{
    Ok = 0U,
    InvalidSource,
    HiddenField,
    NonLiteral,
    UnsupportedBlock,
    InvalidScalar,
    InvalidProcessOption,
    UnsupportedUnits,
    CoordinateRejected,
    UnresolvedModalB
};

// The caller captures the last committed modal value and the current scope on
// the NC owner thread. A parsed, prepared, rejected or merely submitted block
// must never update this source. These fields mirror m_g76Modal's existing
// run/cache/commit/lease proof; they are provenance, not a Motion permit.
struct ModalBScope
{
    std::uint64_t run = 0ULL, cache = 0ULL;
    MotionOwnerLease lease{};
};

struct ModalB
{
    unsigned value = 0U;
    bool valid = false;
    std::uint64_t commit = 0ULL;
    ModalBScope committed{}, current{};
};

inline bool SameModalBScope(const ModalBScope& a, const ModalBScope& b) noexcept
{
    return a.run == b.run && a.cache == b.cache && a.lease.Matches(b.lease);
}

inline bool ValidModalB(const ModalB& modal) noexcept
{
    return modal.valid && modal.value <= 4U && modal.commit != 0ULL &&
        modal.committed.run != 0ULL && modal.committed.cache != 0ULL &&
        modal.committed.lease.IsValid() && SameModalBScope(modal.committed, modal.current);
}

struct Words
{
    NCBlock authored{};
    NCG76Sphere::Point3 authoredXYZ{}; // Literal G21 values, still in program space.
    unsigned targetMask = 0U;
    double radiusMm = 0.0;
    double layerStepMm = 0.0;
    double feedMmMin = 0.0;
    unsigned requestedB = 0U;
    bool explicitB = false;
    bool inheritedB = false;
    ModalB inheritedBSource{};
    bool valid = false;
};

struct Receipt
{
    Words words{};
    NCG76SphereCoordinates::FrozenFrame frame{};
    NCG76Sphere::Input input{}; // PROGRAM geometry. Map through frame exactly once.
    bool valid = false;
};

namespace Detail
{
// The production literal resolver intentionally rounds G-code integers within
// its compatibility tolerance. P50 additionally requires the authored G value
// to be exactly 76, so a near-integer source cannot silently change identity.
inline bool ExactG76(const std::string& expression) noexcept
{
    if (expression.empty()) return false;
    errno = 0;
    char* end = nullptr;
    const char* begin = expression.c_str();
    const double value = std::strtod(begin, &end);
    if (begin == end || errno == ERANGE || !std::isfinite(value) || value != 76.0)
        return false;
    while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n') ++end;
    return end == begin + expression.size();
}

inline bool AllowedAddress(char word) noexcept
{
    return word == 'N' || word == 'P' || word == 'R' || word == 'K' ||
        word == 'F' || word == 'B' || word == 'X' || word == 'Y' || word == 'Z';
}
}

// The real parser and prepared-block literal resolver remain the only syntax
// and numeric sources. Complete hidden-field checks precede interpretation.
// Failure clears output; parsed strings are never modified.
inline Status Decode(const NCParsedBlock& parsed, const ModalB& inherited, Words& output) noexcept
{
    const ModalB frozenModal = inherited;
    output = Words{};
    if (parsed.error != NCParseError::NONE || parsed.isEmpty || parsed.isBlockSkip ||
        parsed.dependsOnMacroState || parsed.duplicateAddressMask != 0U ||
        parsed.controlType != NCParsedControlType::NONE || parsed.gCount != 1 ||
        parsed.mCount != 0)
        return Status::InvalidSource;
    if (parsed.assignment.prefix != 0 || !parsed.assignment.indexExpression.empty() ||
        !parsed.assignment.valueExpression.empty() || parsed.gotoStatement.conditional ||
        !parsed.gotoStatement.conditionExpression.empty() || !parsed.gotoStatement.targetExpression.empty())
        return Status::HiddenField;
    for (std::size_t i = 1U; i < parsed.gExpressions.size(); ++i)
        if (!parsed.gExpressions[i].empty()) return Status::HiddenField;
    for (const auto& value : parsed.mExpressions)
        if (!value.empty()) return Status::HiddenField;
    for (char word = 'A'; word <= 'Z'; ++word)
    {
        if (!parsed.has(word) && !parsed.expression(word).empty()) return Status::HiddenField;
        if (parsed.has(word) && !Detail::AllowedAddress(word)) return Status::UnsupportedBlock;
        if (parsed.expression(word).find('\0') != std::string::npos) return Status::NonLiteral;
    }
    NCBlock literal{};
    if (!NCPreparedBlockQueueShadow::TryBuildLiteralBlock(parsed, literal) ||
        !Detail::ExactG76(parsed.gExpressions[0])) return Status::NonLiteral;
    if (!literal.hasG || literal.gCode != 76 || literal.gCodes[0] != 76 ||
        !literal.has('P') || literal.val('P') != static_cast<double>(ModeP) ||
        !literal.has('R') || !literal.has('K') || !literal.has('F') ||
        !(literal.has('X') || literal.has('Y') || literal.has('Z')))
        return Status::UnsupportedBlock;
    if (!(literal.val('R') > 0.0) || !(literal.val('K') > 0.0) || !(literal.val('F') > 0.0))
        return Status::InvalidScalar;
    if (literal.has('B') && (literal.val('B') < 0.0 || literal.val('B') > 4.0 ||
        std::floor(literal.val('B')) != literal.val('B')))
        return Status::InvalidProcessOption;
    if (!literal.has('B') && !ValidModalB(frozenModal)) return Status::UnresolvedModalB;
    Words candidate{};
    candidate.authored = literal;
    for (unsigned axis = 0U; axis < 3U; ++axis)
    {
        const char word = static_cast<char>('X' + axis);
        if (!literal.has(word)) continue;
        candidate.targetMask |= 1U << axis;
        candidate.authoredXYZ[axis] = literal.val(word);
    }
    candidate.radiusMm = literal.val('R');
    candidate.layerStepMm = literal.val('K');
    candidate.feedMmMin = literal.val('F');
    candidate.explicitB = literal.has('B');
    candidate.inheritedB = !candidate.explicitB;
    candidate.requestedB = candidate.explicitB ? static_cast<unsigned>(literal.val('B')) : frozenModal.value;
    if (candidate.inheritedB) candidate.inheritedBSource = frozenModal;
    candidate.valid = true;
    output = candidate;
    return Status::Ok;
}

// Compatibility entry point: an explicit B is required without modal context.
inline Status Decode(const NCParsedBlock& parsed, Words& output) noexcept
{
    return Decode(parsed, ModalB{}, output);
}

// Snapshot and start are frozen before clearing output, so rebuilding from
// output.frame members cannot erase an input while it is being read. All
// ordinary failures leave a default-invalid receipt. Geometry construction,
// limits, native mapping and eventual execution authorization remain separate.
inline Status Build(const NCParsedBlock& parsed, const NCTranslationSnapshot& snapshot,
    const NCG76SphereCoordinates::Point8& acceptedStartMCS, const ModalB& inherited,
    Receipt& output) noexcept
{
    const NCTranslationSnapshot frozenSnapshot = snapshot;
    const NCG76SphereCoordinates::Point8 frozenStart = acceptedStartMCS;
    const ModalB frozenModal = inherited;
    output = Receipt{};
    Words words{};
    const Status sourceStatus = Decode(parsed, frozenModal, words);
    if (sourceStatus != Status::Ok) return sourceStatus;
    if (words.inheritedB && frozenModal.current.run != frozenSnapshot.runToken)
        return Status::UnresolvedModalB;
    if (frozenSnapshot.unitsMode != 21) return Status::UnsupportedUnits;
    Receipt candidate{};
    if (NCG76SphereCoordinates::BuildFrame(frozenSnapshot, frozenStart, candidate.frame) !=
        NCG76SphereCoordinates::Status::Ok ||
        NCG76SphereCoordinates::BuildInput(candidate.frame, words.authoredXYZ, words.targetMask,
            words.radiusMm, words.layerStepMm, words.feedMmMin, NCG76Sphere::RadiusLaw::Linear,
            candidate.input) != NCG76SphereCoordinates::Status::Ok)
        return Status::CoordinateRejected;
    candidate.words = words;
    candidate.valid = true;
    output = candidate;
    return Status::Ok;
}

inline Status Build(const NCParsedBlock& parsed, const NCTranslationSnapshot& snapshot,
    const NCG76SphereCoordinates::Point8& acceptedStartMCS, Receipt& output) noexcept
{
    return Build(parsed, snapshot, acceptedStartMCS, ModalB{}, output);
}
}
