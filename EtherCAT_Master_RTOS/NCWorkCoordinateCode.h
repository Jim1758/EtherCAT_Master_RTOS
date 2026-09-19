#pragma once

// Project work-coordinate table: ten banks of six rows. Codes are G54..G59,
// G154..G159, ..., G954..G959. Reject holes before any output mutation.
inline bool TryDecodeNCWorkCoordinateCode(int code, int& rowIndex) noexcept
{
    if (code < 54 || code > 959) return false;
    const int bank = code / 100;
    const int suffix = code % 100;
    if (suffix < 54 || suffix > 59) return false;
    rowIndex = bank * 6 + suffix - 54;
    return true;
}

inline bool TryEncodeNCWorkCoordinateCode(int rowIndex, int& code) noexcept
{
    if (rowIndex < 0 || rowIndex >= 60) return false;
    code = (rowIndex / 6) * 100 + 54 + rowIndex % 6;
    return true;
}

inline bool IsNCWorkCoordinateCode(int code) noexcept
{
    int rowIndex = 0;
    return TryDecodeNCWorkCoordinateCode(code, rowIndex);
}
