#include "NCProgramCache.h"

#include <atomic>
#include <limits>
#include <utility>

namespace
{
    std::atomic<NCProgramCacheGeneration> g_nextProgramCacheGeneration{ 1ULL };

    NCProgramCacheGeneration AllocateProgramCacheGeneration() noexcept
    {
        for (;;)
        {
            const NCProgramCacheGeneration generation =
                g_nextProgramCacheGeneration.fetch_add(
                    1ULL,
                    std::memory_order_relaxed);

            if (generation != NC_PROGRAM_CACHE_GENERATION_INVALID)
            {
                return generation;
            }
        }
    }
}

bool NCProgramCache::Build(
    std::vector<std::string> rawLines,
    const GCodeParser& parser)
{
    if (rawLines.size() >
        static_cast<std::size_t>(std::numeric_limits<int>::max()))
    {
        return false;
    }

    std::vector<NCProgramCacheLine> parsedLines;
    std::map<int, int> jumpTable;
    int firstParseErrorPC = -1;

    try
    {
        parsedLines.reserve(rawLines.size());

        for (std::size_t i = 0U; i < rawLines.size(); ++i)
        {
            NCProgramCacheLine line{};
            line.rawLine = std::move(rawLines[i]);
            line.sourceLineNumber = static_cast<int>(i) + 1;

            // Pure Parse exactly once for this source line.
            line.parsedBlock = parser.ParseLine(line.rawLine);

            if (firstParseErrorPC < 0 &&
                line.parsedBlock.error != NCParseError::NONE)
            {
                firstParseErrorPC = static_cast<int>(i);
            }

            int sequenceNumber = 0;
            if (GCodeParser::TryExtractLiteralSequenceNumber(
                line.parsedBlock,
                sequenceNumber))
            {
                // Duplicate N codes preserve the existing behavior:
                // the first definition wins.
                jumpTable.emplace(
                    sequenceNumber,
                    static_cast<int>(i));
            }

            parsedLines.emplace_back(std::move(line));
        }
    }
    catch (...)
    {
        // Keep the old immutable image untouched when allocation/copy fails.
        return false;
    }

    m_lines.swap(parsedLines);
    m_jumpTable.swap(jumpTable);
    m_firstParseErrorPC = firstParseErrorPC;
    m_generation = AllocateProgramCacheGeneration();
    return true;
}

void NCProgramCache::Clear()
{
    m_lines.clear();
    m_jumpTable.clear();
    m_generation = NC_PROGRAM_CACHE_GENERATION_INVALID;
    m_firstParseErrorPC = -1;
}

const NCProgramCacheLine* NCProgramCache::TryGetLine(
    int pc) const noexcept
{
    if (pc < 0 ||
        static_cast<std::size_t>(pc) >= m_lines.size())
    {
        return nullptr;
    }

    return &m_lines[static_cast<std::size_t>(pc)];
}

bool NCProgramCache::TryFindSequence(
    int sequenceNumber,
    int& targetPC) const noexcept
{
    const auto found = m_jumpTable.find(sequenceNumber);
    if (found == m_jumpTable.end())
    {
        return false;
    }

    targetPC = found->second;
    return true;
}
