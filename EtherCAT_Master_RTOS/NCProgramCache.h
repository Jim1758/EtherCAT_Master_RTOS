#pragma once

#include "GCodeParser.h"
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

// =============================================================================
// Stage NC-0.2C - Parsed Program Cache / Program Commit Boundary
//
// NCProgramCache 在「程式載入階段」完成：
//   * 每一行只呼叫一次 GCodeParser::ParseLine()
//   * 保存 Pure Parsed Block
//   * 建立該 Program Scope 專屬的靜態 N Label Table
//
// Runtime 不會再次 Parse 原始字串。# / @ / $ 運算式仍原樣保存在
// NCParsedBlock 中，必須等目前 PC 真正到達時才由 NCExpressionResolver 求值。
// =============================================================================

using NCProgramCacheGeneration = std::uint64_t;
using NCProgramFrameId = std::uint64_t;
using NCProgramCommitSequence = std::uint64_t;

constexpr NCProgramCacheGeneration NC_PROGRAM_CACHE_GENERATION_INVALID = 0ULL;
constexpr NCProgramFrameId NC_PROGRAM_FRAME_ID_INVALID = 0ULL;
constexpr NCProgramCommitSequence NC_PROGRAM_COMMIT_SEQUENCE_INVALID = 0ULL;

enum class NCProgramScope : std::uint8_t
{
    NONE = 0,
    MEMORY = 1,
    MDI = 2,
    MANUAL_AUTO = 3,
    MACRO = 4
};

struct NCProgramCommitSnapshot
{
    NCProgramScope scope = NCProgramScope::NONE;
    NCProgramCacheGeneration cacheGeneration =
        NC_PROGRAM_CACHE_GENERATION_INVALID;
    NCProgramFrameId frameId = NC_PROGRAM_FRAME_ID_INVALID;
    int sourcePC = -1;
    NCProgramCommitSequence sequence =
        NC_PROGRAM_COMMIT_SEQUENCE_INVALID;

    bool IsValid() const noexcept
    {
        return
            scope != NCProgramScope::NONE &&
            cacheGeneration != NC_PROGRAM_CACHE_GENERATION_INVALID &&
            sourcePC >= 0 &&
            sequence != NC_PROGRAM_COMMIT_SEQUENCE_INVALID &&
            (scope != NCProgramScope::MACRO ||
                frameId != NC_PROGRAM_FRAME_ID_INVALID);
    }
};

struct NCProgramCacheLine
{
    std::string rawLine;
    NCParsedBlock parsedBlock{};
    int sourceLineNumber = 0;
};

class NCProgramCache
{
public:
    NCProgramCache() = default;

    NCProgramCache(const NCProgramCache&) = delete;
    NCProgramCache& operator=(const NCProgramCache&) = delete;

    NCProgramCache(NCProgramCache&&) = default;
    NCProgramCache& operator=(NCProgramCache&&) = default;

    // rawLines 以值傳入，Build 後 Move 到 Cache，避免保留第二份原始程式。
    // 語法錯誤也會被 Cache；仍只在 Runtime 真正走到該 PC 時報警。
    bool Build(
        std::vector<std::string> rawLines,
        const GCodeParser& parser);

    void Clear();

    bool Empty() const noexcept
    {
        return m_lines.empty();
    }

    std::size_t Size() const noexcept
    {
        return m_lines.size();
    }

    const NCProgramCacheLine* TryGetLine(int pc) const noexcept;

    bool TryFindSequence(
        int sequenceNumber,
        int& targetPC) const noexcept;

    NCProgramCacheGeneration GetGeneration() const noexcept
    {
        return m_generation;
    }

    std::size_t GetParsedLineCount() const noexcept
    {
        return m_lines.size();
    }

    int GetFirstParseErrorPC() const noexcept
    {
        return m_firstParseErrorPC;
    }

private:
    std::vector<NCProgramCacheLine> m_lines;
    std::map<int, int> m_jumpTable;

    NCProgramCacheGeneration m_generation =
        NC_PROGRAM_CACHE_GENERATION_INVALID;
    int m_firstParseErrorPC = -1;
};
