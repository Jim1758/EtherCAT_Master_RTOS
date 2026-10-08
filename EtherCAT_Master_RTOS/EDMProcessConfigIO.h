#pragma once

#include "EDMProcessProfile.h"
#include <cmath>
#include <fstream>
#include <limits>
#include <locale>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

// Startup/worker thread only. No file access or allocation is allowed in the RT owner.
namespace EDMProcessConfigIO
{
    static const std::size_t MaximumFileBytes = 65536U;
    static const std::size_t MaximumLineBytes = 2048U;
    static const std::size_t MaximumLines = 512U;

    struct Diagnostic
    {
        std::string path;
        std::size_t line = 0U;
        std::string key;
        std::string reason;

        std::string ToString() const
        {
            std::ostringstream text;
            text.imbue(std::locale::classic());
            text << path;
            if (line != 0U) text << ':' << line;
            if (!key.empty()) text << " [" << key << ']';
            if (!reason.empty()) text << ": " << reason;
            return text.str();
        }
    };

    namespace Detail
    {
        enum class DocumentKind { Process, DischargeGain, FlushGain };
        enum class Kind { Unsigned, Boolean, Real, Mode };
        struct Rule
        {
            std::string name;
            Kind kind = Kind::Real;
            double* real = nullptr;
            std::uint32_t* integer = nullptr;
            bool* boolean = nullptr;
            EDM20::ProcessMode* mode = nullptr;
            double minimum = 0.0;
            double maximum = (std::numeric_limits<double>::max)();
            bool exclusiveMinimum = false;
            bool found = false;
            std::size_t line = 0U;
        };

        inline std::string Trim(const std::string& text)
        {
            const std::size_t first = text.find_first_not_of(" \t\r");
            if (first == std::string::npos) return std::string();
            return text.substr(first, text.find_last_not_of(" \t\r") - first + 1U);
        }

        inline bool Fail(Diagnostic& diagnostic, const std::string& path,
            std::size_t line, const std::string& key, const std::string& reason)
        {
            diagnostic.path = path;
            diagnostic.line = line;
            diagnostic.key = key;
            diagnostic.reason = reason;
            return false;
        }

        inline void AddReal(std::vector<Rule>& rules, const std::string& name, double& target,
            double minimum = 0.0, bool exclusiveMinimum = false,
            double maximum = (std::numeric_limits<double>::max)())
        {
            Rule rule;
            rule.name = name;
            rule.real = &target;
            rule.minimum = minimum;
            rule.maximum = maximum;
            rule.exclusiveMinimum = exclusiveMinimum;
            rules.push_back(rule);
        }

        inline void AddUnsigned(std::vector<Rule>& rules, const std::string& name,
            std::uint32_t& target, std::uint32_t minimum = 0U,
            std::uint32_t maximum = (std::numeric_limits<std::uint32_t>::max)())
        {
            Rule rule;
            rule.name = name;
            rule.kind = Kind::Unsigned;
            rule.integer = &target;
            rule.minimum = static_cast<double>(minimum);
            rule.maximum = static_cast<double>(maximum);
            rules.push_back(rule);
        }

        inline void AddBoolean(std::vector<Rule>& rules, const std::string& name, bool& target)
        {
            Rule rule;
            rule.name = name;
            rule.kind = Kind::Boolean;
            rule.boolean = &target;
            rules.push_back(rule);
        }

        inline std::vector<Rule> MakeRules(EDM20::ProcessProfile& profile, DocumentKind document)
        {
            std::vector<Rule> rules;
            rules.reserve(128U);
            AddUnsigned(rules, "Profile.SchemaVersion", profile.schemaVersion, 2U, 2U);
            AddUnsigned(rules, "Profile.Revision", profile.revision, 1U);
            AddUnsigned(rules, "Profile.MachineProfileId", profile.machineProfileId, 1U);
            if (document != DocumentKind::Process)
            {
                for (unsigned axis = 0U; axis != 8U; ++axis)
                {
                    EDM20::AxisGainProfile& gain = document == DocumentKind::DischargeGain ?
                        profile.axisGains.discharge[axis] : profile.axisGains.flush[axis];
                    const std::string name = "Axis" + std::string(1U, static_cast<char>('0' + axis)) + ".";
                    AddBoolean(rules, name + "InheritCnc", gain.inheritCnc);
                    AddReal(rules, name + "Kp", gain.custom.Kp);
                    AddReal(rules, name + "Ki", gain.custom.Ki);
                    AddReal(rules, name + "Kd", gain.custom.Kd);
                    AddReal(rules, name + "Kvff", gain.custom.Kvff);
                }
                return rules;
            }
            Rule mode;
            mode.name = "Profile.Mode";
            mode.kind = Kind::Mode;
            mode.mode = &profile.mode;
            rules.push_back(mode);
            for (unsigned index = 0U; index != 3U; ++index)
            {
                const std::string number(1U, static_cast<char>('1' + index));
                AddReal(rules, "Servo.PositiveGain" + number, profile.servo.positiveGain[index]);
                AddReal(rules, "Servo.NegativeGain" + number, profile.servo.negativeGain[index]);
            }
            for (unsigned index = 0U; index != 2U; ++index)
            {
                const std::string number(1U, static_cast<char>('1' + index));
                AddReal(rules, "Servo.PositiveBreak" + number + "V", profile.servo.positiveBreakV[index], 0.0, true);
                AddReal(rules, "Servo.NegativeBreak" + number + "V", profile.servo.negativeBreakV[index], 0.0, true);
            }
            AddReal(rules, "Servo.CuttingScaleMmPerVoltMin", profile.servo.cuttingScaleMmPerVoltMin, 0.001);
            AddReal(rules, "Servo.MaxFeedMmPerMin", profile.servo.maxFeedMmPerMin, 0.0, true);
            AddReal(rules, "Servo.MaxRetreatMmPerMin", profile.servo.maxRetreatMmPerMin, 0.0, true);
            AddBoolean(rules, "Servo.LimitsConfirmed", profile.servo.limitsConfirmed);
            AddReal(rules, "Servo.DeadbandV", profile.servo.deadbandV);
            AddBoolean(rules, "ShortCircuit.IdleEnabled", profile.servo.shortIdleEnabled);
            AddReal(rules, "ShortCircuit.IdleThresholdV", profile.servo.shortIdleV);
            AddBoolean(rules, "ShortCircuit.MachiningEnabled", profile.servo.shortMachiningEnabled);
            AddReal(rules, "ShortCircuit.MachiningThresholdV", profile.servo.shortMachiningV);
            AddUnsigned(rules, "ShortCircuit.EnterMs", profile.servo.shortEnterMs);
            AddUnsigned(rules, "ShortCircuit.ExitMs", profile.servo.shortExitMs);
            AddReal(rules, "ShortCircuit.HysteresisV", profile.servo.shortHysteresisV);
            AddReal(rules, "ShortCircuit.RetreatMmPerMin", profile.servo.shortRetreatMmPerMin);
            AddReal(rules, "Flush.FinalApproachMmPerMin", profile.flush.finalApproachMmPerMin, 0.0, true);
            AddReal(rules, "Flush.CenterRetractMmPerMin", profile.flush.centerRetractMmPerMin, 0.0, true);
            AddReal(rules, "Flush.MainRetractMmPerMin", profile.flush.mainRetractMmPerMin, 0.0, true);
            AddReal(rules, "Flush.CenterReturnMmPerMin", profile.flush.centerReturnMmPerMin, 0.0, true);
            AddReal(rules, "Flush.MainReturnMmPerMin", profile.flush.mainReturnMmPerMin, 0.0, true);
            AddReal(rules, "Flush.PathRetractMmPerMin", profile.flush.pathRetractMmPerMin, 0.0, true);
            AddReal(rules, "Flush.InitialSlowMmPerMin", profile.flush.initialSlowMmPerMin, 0.0, true);
            AddReal(rules, "Flush.ManualReturnMmPerMin", profile.flush.manualReturnMmPerMin, 0.0, true);
            AddReal(rules, "Flush.FinalApproachDistanceMm", profile.flush.finalApproachDistanceMm);
            AddReal(rules, "Flush.PathDistanceMm", profile.flush.pathDistanceMm);
            AddReal(rules, "Flush.InitialSlowDistanceMm", profile.flush.initialSlowDistanceMm);
            return rules;
        }

        inline bool IsDecimalReal(const std::string& value)
        {
            if (value.empty()) return false;
            std::size_t cursor = 0U;
            if (value[cursor] == '+' || value[cursor] == '-') ++cursor;
            bool digits = false;
            while (cursor < value.size() && value[cursor] >= '0' && value[cursor] <= '9') { ++cursor; digits = true; }
            if (cursor < value.size() && value[cursor] == '.')
            {
                ++cursor;
                while (cursor < value.size() && value[cursor] >= '0' && value[cursor] <= '9') { ++cursor; digits = true; }
            }
            if (!digits) return false;
            if (cursor < value.size() && (value[cursor] == 'e' || value[cursor] == 'E'))
            {
                ++cursor;
                if (cursor < value.size() && (value[cursor] == '+' || value[cursor] == '-')) ++cursor;
                const std::size_t exponentStart = cursor;
                while (cursor < value.size() && value[cursor] >= '0' && value[cursor] <= '9') ++cursor;
                if (cursor == exponentStart) return false;
            }
            return cursor == value.size();
        }

        inline bool Assign(Rule& rule, const std::string& value, std::string& reason)
        {
            if (rule.kind == Kind::Mode)
            {
                if (value != "SHADOW_ONLY") { reason = "only SHADOW_ONLY is supported by schema 2"; return false; }
                *rule.mode = EDM20::ProcessMode::ShadowOnly;
                return true;
            }
            if (rule.kind == Kind::Boolean)
            {
                if (value != "0" && value != "1") { reason = "expected 0 or 1"; return false; }
                *rule.boolean = value == "1";
                return true;
            }
            double number = 0.0;
            if (rule.kind == Kind::Unsigned)
            {
                if (value.empty()) { reason = "missing unsigned integer"; return false; }
                std::uint32_t integer = 0U;
                for (std::size_t cursor = 0U; cursor < value.size(); ++cursor)
                {
                    if (value[cursor] < '0' || value[cursor] > '9') { reason = "expected an unsigned decimal integer"; return false; }
                    const std::uint32_t digit = static_cast<std::uint32_t>(value[cursor] - '0');
                    if (integer > ((std::numeric_limits<std::uint32_t>::max)() - digit) / 10U)
                    { reason = "unsigned integer overflow"; return false; }
                    integer = integer * 10U + digit;
                }
                if (rule.name == "Profile.SchemaVersion" && integer != 2U)
                { reason = "unsupported schema; expected SchemaVersion=2 with mm-based units and separate gain files"; return false; }
                number = static_cast<double>(integer);
                if (number < rule.minimum || number > rule.maximum) { reason = "integer is outside the permitted range"; return false; }
                *rule.integer = integer;
                return true;
            }
            if (!IsDecimalReal(value)) { reason = "expected a finite decimal number without a unit suffix"; return false; }
            std::istringstream parser(value);
            parser.imbue(std::locale::classic());
            parser >> number;
            if (!parser || !parser.eof() || !std::isfinite(number)) { reason = "number is non-finite or cannot be represented"; return false; }
            if (number == 0.0)
            {
                // Reject nonzero decimal input silently rounded to zero.
                const std::size_t exponent = value.find_first_of("eE");
                const std::size_t end = exponent == std::string::npos ? value.size() : exponent;
                for (std::size_t cursor = 0U; cursor < end; ++cursor)
                    if (value[cursor] >= '1' && value[cursor] <= '9')
                    { reason = "nonzero number underflows the numeric representation"; return false; }
            }
            if (number < rule.minimum || number > rule.maximum || (rule.exclusiveMinimum && number == rule.minimum))
            { reason = "number is outside the permitted range"; return false; }
            *rule.real = number;
            return true;
        }
    }

    // Transactional: out changes only after every key and cross-field rule passes.
    inline bool ParseDocument(const std::string& text, const std::string& path,
        EDM20::ProcessProfile& out, Diagnostic& diagnostic, Detail::DocumentKind document)
    {
        if (text.size() > MaximumFileBytes) return Detail::Fail(diagnostic, path, 0U, "", "file exceeds 65536 bytes");
        if (text.find('\0') != std::string::npos) return Detail::Fail(diagnostic, path, 0U, "", "NUL byte is not permitted");
        EDM20::ProcessProfile candidate;
        std::vector<Detail::Rule> rules = Detail::MakeRules(candidate, document);
        std::map<std::string, std::size_t> lookup;
        std::set<std::string> sections;
        std::set<std::string> seenSections;
        for (std::size_t index = 0U; index < rules.size(); ++index)
        {
            lookup[rules[index].name] = index;
            sections.insert(rules[index].name.substr(0U, rules[index].name.rfind('.')));
        }
        const bool hasBom = text.size() >= 3U && static_cast<unsigned char>(text[0]) == 0xEFU &&
            static_cast<unsigned char>(text[1]) == 0xBBU && static_cast<unsigned char>(text[2]) == 0xBFU;
        std::istringstream lines(hasBom ? text.substr(3U) : text);
        std::string section;
        std::string line;
        std::size_t lineNumber = 0U;
        while (std::getline(lines, line))
        {
            ++lineNumber;
            if (lineNumber > MaximumLines) return Detail::Fail(diagnostic, path, lineNumber, "", "file exceeds 512 lines");
            if (line.size() > MaximumLineBytes) return Detail::Fail(diagnostic, path, lineNumber, "", "line exceeds 2048 bytes");
            for (std::size_t index = 0U; index < line.size(); ++index)
            {
                const unsigned char ch = static_cast<unsigned char>(line[index]);
                if ((ch < 32U && ch != '\t' && ch != '\r') || ch == 127U)
                    return Detail::Fail(diagnostic, path, lineNumber, "", "control character is not permitted");
            }
            const std::size_t comment = line.find_first_of(";#");
            line = Detail::Trim(line.substr(0U, comment));
            if (line.empty()) continue;
            if (line[0] == '[')
            {
                if (line.size() < 3U || line[line.size() - 1U] != ']')
                    return Detail::Fail(diagnostic, path, lineNumber, "", "malformed section header");
                section = Detail::Trim(line.substr(1U, line.size() - 2U));
                if (sections.find(section) == sections.end()) return Detail::Fail(diagnostic, path, lineNumber, section, "unknown section");
                if (!seenSections.insert(section).second) return Detail::Fail(diagnostic, path, lineNumber, section, "duplicate section");
                continue;
            }
            const std::size_t equals = line.find('=');
            if (equals == std::string::npos || line.find('=', equals + 1U) != std::string::npos)
                return Detail::Fail(diagnostic, path, lineNumber, section, "expected exactly one key=value assignment");
            const std::string key = Detail::Trim(line.substr(0U, equals));
            const std::string name = section.empty() ? key : section + '.' + key;
            if (section.empty()) return Detail::Fail(diagnostic, path, lineNumber, name, "key appears before a section");
            if (name == "Servo.CuttingScaleUmPerVoltMin")
                return Detail::Fail(diagnostic, path, lineNumber, name,
                    "obsolete key; use CuttingScaleMmPerVoltMin in mm/(V*min) after explicit unit conversion");
            const std::map<std::string, std::size_t>::const_iterator entry = lookup.find(name);
            if (entry == lookup.end()) return Detail::Fail(diagnostic, path, lineNumber, name, "unknown key (names are case-sensitive)");
            Detail::Rule& rule = rules[entry->second];
            if (rule.found) return Detail::Fail(diagnostic, path, lineNumber, name, "duplicate key");
            std::string reason;
            if (!Detail::Assign(rule, Detail::Trim(line.substr(equals + 1U)), reason))
                return Detail::Fail(diagnostic, path, lineNumber, name, reason);
            rule.found = true;
            rule.line = lineNumber;
        }
        for (std::size_t index = 0U; index < rules.size(); ++index)
            if (!rules[index].found) return Detail::Fail(diagnostic, path, lineNumber + 1U, rules[index].name, "required key is missing");
        candidate.servo.revision = candidate.revision;
        const char* invalidKey = nullptr;
        if (candidate.servo.positiveBreakV[1] <= candidate.servo.positiveBreakV[0]) invalidKey = "Servo.PositiveBreak2V";
        else if (candidate.servo.negativeBreakV[1] <= candidate.servo.negativeBreakV[0]) invalidKey = "Servo.NegativeBreak2V";
        if (invalidKey != nullptr)
            return Detail::Fail(diagnostic, path, rules[lookup[invalidKey]].line, invalidKey, "second breakpoint must exceed the first breakpoint");
        if (candidate.servo.deadbandV >= candidate.servo.positiveBreakV[0] ||
            candidate.servo.deadbandV >= candidate.servo.negativeBreakV[0])
            return Detail::Fail(diagnostic, path, rules[lookup["Servo.DeadbandV"]].line,
                "Servo.DeadbandV", "deadband must be smaller than both first breakpoints");
        if (!std::isfinite(candidate.servo.shortIdleV + candidate.servo.shortHysteresisV) ||
            !std::isfinite(candidate.servo.shortMachiningV + candidate.servo.shortHysteresisV))
            return Detail::Fail(diagnostic, path, rules[lookup["ShortCircuit.HysteresisV"]].line,
                "ShortCircuit.HysteresisV", "threshold plus hysteresis must remain finite");
        if (!EDM20::ValidateProcessProfile(candidate))
            return Detail::Fail(diagnostic, path, 0U, "Profile", "profile cross-field validation failed");
        out = candidate;
        diagnostic = Diagnostic();
        return true;
    }

    inline bool LoadDocument(const std::string& path, EDM20::ProcessProfile& out, Diagnostic& diagnostic,
        Detail::DocumentKind document)
    {
        std::ifstream input(path.c_str(), std::ios::in | std::ios::binary);
        if (!input.is_open()) return Detail::Fail(diagnostic, path, 0U, "", "cannot open file; no defaults were written");
        std::string text(MaximumFileBytes + 1U, '\0');
        input.read(&text[0], static_cast<std::streamsize>(text.size()));
        const std::streamsize count = input.gcount();
        if (input.bad()) return Detail::Fail(diagnostic, path, 0U, "", "file read failed");
        text.resize(static_cast<std::size_t>(count));
        return ParseDocument(text, path, out, diagnostic, document);
    }

    // Process-only helpers deliberately never load dedicated axis-gain files.
    inline bool ParseText(const std::string& text, const std::string& path,
        EDM20::ProcessProfile& out, Diagnostic& diagnostic)
    { return ParseDocument(text, path, out, diagnostic, Detail::DocumentKind::Process); }

    inline bool LoadFile(const std::string& path, EDM20::ProcessProfile& out, Diagnostic& diagnostic)
    { return LoadDocument(path, out, diagnostic, Detail::DocumentKind::Process); }

    inline std::string BundlePath(const std::string& directory, const char* name)
    {
        if (directory.empty()) return std::string(name);
        const char last = directory[directory.size() - 1U];
        return directory + ((last == '/' || last == '\\') ? "" : "/") + name;
    }

    // All three files must form one coherent identity. Commit the full profile
    // only after every file passes; failure leaves the caller's profile intact.
    inline bool LoadBundle(const std::string& directory, EDM20::ProcessProfile& out,
        Diagnostic& diagnostic)
    {
        EDM20::ProcessProfile candidate;
        if (!LoadFile(BundlePath(directory, "EDMProcessConfig.ini"), candidate, diagnostic)) return false;
        const char* names[2] = { "EDMDischargeAxisGain.ini", "EDMFlushAxisGain.ini" };
        for (unsigned bank = 0U; bank < 2U; ++bank)
        {
            EDM20::ProcessProfile gain;
            const std::string path = BundlePath(directory, names[bank]);
            const Detail::DocumentKind document = bank == 0U ?
                Detail::DocumentKind::DischargeGain : Detail::DocumentKind::FlushGain;
            if (!LoadDocument(path, gain, diagnostic, document)) return false;
            if (gain.machineProfileId != candidate.machineProfileId)
                return Detail::Fail(diagnostic, path, 0U, "Profile.MachineProfileId", "bundle identity does not match EDMProcessConfig.ini");
            if (gain.revision != candidate.revision)
                return Detail::Fail(diagnostic, path, 0U, "Profile.Revision", "bundle revision does not match EDMProcessConfig.ini");
            if (bank == 0U) candidate.axisGains.discharge = gain.axisGains.discharge;
            else candidate.axisGains.flush = gain.axisGains.flush;
        }
        if (!EDM20::ValidateProcessProfile(candidate))
            return Detail::Fail(diagnostic, directory, 0U, "Profile", "combined bundle validation failed");
        out = candidate;
        diagnostic = Diagnostic();
        return true;
    }

    inline bool LoadFile(const std::string& path, EDM20::ProcessProfile& out, std::string& diagnostic)
    {
        Diagnostic detail;
        const bool loaded = LoadFile(path, out, detail);
        diagnostic = loaded ? std::string() : detail.ToString();
        return loaded;
    }
}
