// EDM267 FIX1: independent NC compilation unit; original function bodies preserved.
#include "EtherCatMaster.h"
#include "NCManager_GapInternal.h"

using namespace NCGapDetail;

bool NCManager::InstallEDMRecipeCatalogBeforeStart(std::unique_ptr<EDMRecipe::Catalog> catalog) noexcept
{
    if (m_gapDryRun.active || m_gapDryRunSerial != 0ULL)
    {
        ClearEDMRecipeSelectionSameThread();
        return false;
    }
    const bool installed = m_recipeStore.Install(std::move(catalog));
    const bool bound = m_recipe.Bind(m_recipeStore);
    (void)m_edmRecipeTest.Bind(m_recipeStore); // Test fixtures are optional to the live selector.
    if (installed && bound)
        RtPrintf("[EDM19_FIX4] conditionEdit=LIVE logicalRetention=1 startupE1=1 alarmDisplay=1 mode=SHADOW_ONLY physicalPermit=0 discharge=0\n");
    return installed && bound;
}

bool NCManager::SelectEDMRecipeAtStartupBeforeStart(std::uint32_t tableId, bool recoveryDisplayOnly) noexcept
{
    // This cannot create a run/lease receipt. The catalog install and this
    // single attempt both finish before the NC publisher is started.
    if (m_recipeStartupSelectionAttempted || m_gapDryRun.active || m_gapDryRunSerial != 0ULL ||
        m_pathCoreLiveBookkeeping.currentRunToken != 0ULL || m_recipe.Current().tableSelected)
        return false;
    m_recipeStartupSelectionAttempted = true;
    const EDMRecipe::Catalog* catalog = m_recipe.GetCatalog();
    if (!m_recipe.Ready() || !catalog || tableId == 0U ||
        !m_recipe.SelectTableAndE1(tableId))
        return false;
    m_recipeRecoveryDisplayOnly = recoveryDisplayOnly;
    RetainEDMRecipeSelectionSameThread();
    RtPrintf("[EDM19_FIX4] event=STARTUP_SELECT table=%u E=1 recoveryDisplayOnly=%u mode=SHADOW_ONLY physicalPermit=0 discharge=0\n",
        static_cast<unsigned int>(tableId), recoveryDisplayOnly ? 1U : 0U);
    return true;
}

bool NCManager::IsEDMRecipeSelectionRequest(const NCBlock& block) noexcept
{
    return NCGCodeSemantics::Contains(block, 38) ||
        (block.has('E') && !NCGCodeSemantics::Contains(block, 65) && !NCGCodeSemantics::Contains(block, 66));
}

bool NCManager::DecodeEDMRecipeSelectionBlock(const NCBlock& block, bool& table, std::uint32_t& id) noexcept
{
    table = NCGCodeSemantics::Contains(block, 38);
    id = 0U;
    if (block.isEmpty || block.isGoto || block.isBlockSkip || block.mCount != 0) return false;
    if (table)
    {
        if (!block.hasG || block.gCode != 38 || block.gCount != 1 || block.gCodes[0] != 38 || !block.has('P')) return false;
    }
    else if (block.hasG || block.gCount != 0 || !block.has('E')) return false;
    const char selector = table ? 'P' : 'E';
    for (char word = 'A'; word <= 'Z'; ++word)
        if (block.has(word) && ((word != 'N' && word != selector) || !std::isfinite(block.val(word)))) return false;
    const double value = block.val(selector);
    const double maximum = table ? static_cast<double>((std::numeric_limits<std::uint32_t>::max)()) :
        static_cast<double>(EDMRecipe::RowsPerTable);
    if (!std::isfinite(value) || value < 1.0 || value > maximum || std::floor(value) != value) return false;
    id = static_cast<std::uint32_t>(value);
    return true;
}

bool NCManager::IsEDMRecipeStoppedSelectionAllowedSameThread(const NCBlock& block)
{
    bool table = false; std::uint32_t id = 0U;
    return DecodeEDMRecipeSelectionBlock(block, table, id) && IsEDMRecipeStoppedAuthorityCurrentSameThread();
}

bool NCManager::IsEDMRecipeStoppedAuthorityCurrentSameThread()
{
    if (m_recipeRecoveryDisplayOnly) return false;
    // A fresh run is bound before the first motion freezes/publishes its frame.
    // A parameter-only selector must neither require nor create that publication.
    const NCTranslationSnapshot source = CoordSys.GetTranslationSnapshot();
    const bool translationCurrent = CoordSys.IsTranslationRunFrozen() ?
        (CoordSys.IsTranslationRunCurrent() && IsFixedTranslationTravelCurrentSameThread() &&
            m_motion.MatchesNCTranslation(source)) :
        (IsNCTranslationSnapshotValid(source) && m_motion.GetActiveTranslationGeneration() == 0ULL);
    return m_mode == NCOperationMode::MEMORY && m_state == NCState::RUN &&
        !AlarmManager::GetInstance().HasAlarm() && !Close_System_Com_flag &&
        m_edmState != EDMState::NOT_READY && !Homing.IsActive() && !m_isG66Active && m_macroStack.empty() &&
        !m_pathFeed.pending && !m_pathArc.pending && !m_pathReplay.pending &&
        !m_pathHold.armed && !m_pathHold.bound && !m_gapDryRun.active && !m_gapPath.active && !m_gapWindow.active &&
        !m_cncFeed.selected && !m_cncFeed.active && m_cncFeed.count == 0U &&
        !m_pathCoreCommittedBookkeeping.pendingAdmission && !m_cutterLine.leadOutRequired &&
        m_pathCoreLiveBookkeeping.currentRunToken != 0ULL &&
        CoordSys.IsTranslationRunBound() && CoordSys.IsTranslationAxisIdentityCurrent() &&
        IsPathCoreLiveNativeConfigCurrentSameThread() && translationCurrent &&
        m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) &&
        !m_motion.HasPendingSafetyOrRecoveryRequests() && m_motion.IsGroupDone() && m_motion.IsGroupNCDrained() &&
        m_motion.GetQueueSize() == 0U && m_motion.GetCommandIngressSize() == 0U && m_motion.GetCommandReplaySize() == 0U;
}

void NCManager::ClearEDMRecipeSelectionSameThread() noexcept
{
    InvalidateEDMRecipeEditContextsSameThread();
    if (m_recipe.Current().tableSelected || m_recipe.Current().rowSelected) m_recipe.Clear();
    m_recipeRun = m_recipeCache = 0ULL;
    m_recipeEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    m_recipeLease = MotionOwnerLease{};
    m_recipeHoldSeen = false;
    m_recipeIdleSelection = false;
    m_recipePrepared.ready = false;
}

// Keep the operator's logical COND/E and overrides, but retire every program
// receipt. This is display/edit state only; it cannot authorize G39 or motion.
void NCManager::RetainEDMRecipeSelectionSameThread() noexcept
{
    InvalidateEDMRecipeEditContextsSameThread();
    m_recipeRun = m_recipeCache = 0ULL;
    m_recipeEpoch = MOTION_EXECUTION_EPOCH_INVALID;
    m_recipeLease = MotionOwnerLease{};
    m_recipeHoldSeen = false;
    m_recipeIdleSelection = true;
    m_recipePrepared.ready = false;
}

void NCManager::ValidateEDMRecipeSelectionSameThread() noexcept
{
    if (m_state == NCState::HOLD) FenceEDMRecipeHoldSameThread();
    if (!m_recipe.Current().tableSelected) return;
    if (!m_recipe.Ready() || Close_System_Com_flag)
    { ClearEDMRecipeSelectionSameThread(); return; }
    if (AlarmManager::GetInstance().HasAlarm() || m_state == NCState::ALARM)
    {
        // Keep the actual COND/E visible for recovery and diagnosis. An alarm
        // retires program authority once; repeated display reads stay stable.
        if (!m_recipeIdleSelection) RetainEDMRecipeSelectionSameThread();
        return;
    }
    if (m_recipeIdleSelection) return; // Logical-only; no run/lease is implied.
    if (m_recipeRun == 0ULL || m_recipeRun != m_pathCoreLiveBookkeeping.currentRunToken ||
        m_recipeCache != GetBaseProgramCache().GetGeneration() ||
        m_mode != NCOperationMode::MEMORY || (m_state != NCState::RUN && m_state != NCState::HOLD))
    { RetainEDMRecipeSelectionSameThread(); return; }
    if (m_state == NCState::HOLD) return;
    if (m_recipeHoldSeen)
    {
        // Only a previously NC-bound selection can follow a legal resume.
        if (m_motion.HasPendingSafetyOrRecoveryRequests() ||
            !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease)) return;
        m_recipeEpoch = m_motion.GetCurrentExecutionEpoch();
        m_recipeLease = m_programMotionLease;
        m_recipeHoldSeen = false;
    }
    else if (m_recipeEpoch != m_motion.GetCurrentExecutionEpoch() ||
        !m_recipeLease.Matches(m_programMotionLease))
        RetainEDMRecipeSelectionSameThread();
}

bool NCManager::ReadEDMRecipeSummarySameThread(EDMRecipe::Summary& output) noexcept
{
    ValidateEDMRecipeSelectionSameThread();
    output = m_recipe.Current();
    return output.rowSelected && !m_recipeRecoveryDisplayOnly && !m_recipeIdleSelection && !m_recipeHoldSeen &&
        m_recipeRun != 0ULL && m_recipeRun == m_pathCoreLiveBookkeeping.currentRunToken &&
        m_recipeCache == GetBaseProgramCache().GetGeneration() &&
        m_recipeEpoch == m_motion.GetCurrentExecutionEpoch() && m_recipeLease.Matches(m_programMotionLease) &&
        m_mode == NCOperationMode::MEMORY && m_state == NCState::RUN &&
        !AlarmManager::GetInstance().HasAlarm() && !Close_System_Com_flag &&
        m_edmState != EDMState::NOT_READY && !Homing.IsActive() && !m_isG66Active && m_macroStack.empty() &&
        !m_gapDryRun.active && !m_motion.HasPendingSafetyOrRecoveryRequests() &&
        m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease);
}

bool NCManager::ReadEDMRecipeFieldSameThread(std::uint16_t fieldId, EDMRecipe::ParameterSnapshot& output) noexcept
{
    output = EDMRecipe::ParameterSnapshot{};
    EDMRecipe::Summary summary{};
    return ReadEDMRecipeSummarySameThread(summary) && m_recipe.ReadField(fieldId, output);
}

void NCManager::RejectEDMRecipeSelectionSameThread(int line, const char* reason, int alarmCode)
{
    RetainEDMRecipeSelectionSameThread();
    const int code = alarmCode == 0 ? AlarmManager::G_Code_Invalid_parameter : alarmCode;
    RtPrintf("[EDM19_FIX4] event=SELECT result=REJECT line=%d reason=%s alarm=%d mode=SHADOW_ONLY motion=0 physicalPermit=0 discharge=0\n", line, reason, code);
    AlarmManager::GetInstance().Trigger(code, line);
    ChangeState(NCState::ALARM);
}

bool NCManager::PrepareEDMRecipeSelectionSameThread(const NCBlock& block, int sourcePC, int line, NCBlockDispatchId dispatch)
{
    m_recipePrepared.ready = false;
    bool table = false; std::uint32_t id = 0U;
    if (!DecodeEDMRecipeSelectionBlock(block, table, id))
    { RejectEDMRecipeSelectionSameThread(line, "BLOCK_SHAPE"); return false; }
    const NCProgramCacheLine* source = GetBaseProgramCache().TryGetLine(sourcePC);
    if (m_mode != NCOperationMode::MEMORY || !m_macroStack.empty() || !source ||
        source->sourceLineNumber != line || source->parsedBlock.controlType != NCParsedControlType::NONE ||
        dispatch == NC_BLOCK_DISPATCH_ID_INVALID || dispatch != m_currentExecutingBlockDispatchId)
    { RejectEDMRecipeSelectionSameThread(line, "PARSED_IDENTITY"); return false; }
    if (source->parsedBlock.duplicateAddressMask != 0U)
    { RejectEDMRecipeSelectionSameThread(line, "DUPLICATE_ADDRESS"); return false; }
    if (!IsEDMRecipeStoppedSelectionAllowedSameThread(block))
    { RejectEDMRecipeSelectionSameThread(line, "NOT_READY"); return false; }
    ValidateEDMRecipeSelectionSameThread();
    const EDMRecipe::Catalog* catalog = m_recipe.GetCatalog();
    if (!m_recipe.Ready() || !catalog)
    { RejectEDMRecipeSelectionSameThread(line, "CATALOG_UNAVAILABLE"); return false; }
    const std::uint32_t tableId = table ? id : m_recipe.Current().tableId;
    const EDMRecipe::Table* selected = nullptr;
    for (const auto& candidate : catalog->tables) if (candidate.id == tableId) { selected = &candidate; break; }
    if (!selected)
    {
        RtPrintf("[EDM19_FIX4] event=TABLE_NOT_FOUND requested=%u line=%d source=%s\n",
            static_cast<unsigned int>(tableId), line, table ? "G38" : "E");
        RejectEDMRecipeSelectionSameThread(line, table ? "TABLE_UNKNOWN" : "E_NO_TABLE",
            table ? AlarmManager::EDM_CONDITION_TABLE_NOT_FOUND : AlarmManager::G_Code_Invalid_parameter);
        return false;
    }
    if (!table || catalog->schemaVersion == 2U)
    {
        const std::uint32_t selectedE = table ? 1U : id;
        bool configured = false;
        const std::size_t offset = (static_cast<std::size_t>(selectedE) - 1U) * catalog->fields.size();
        for (std::size_t f = 0U; f < catalog->fields.size(); ++f)
            configured = configured || selected->cells[offset + f].kind != EDMRecipe::CellKind::Unset;
        if (!selected->rowPresent[selectedE - 1U] || !configured)
        { RejectEDMRecipeSelectionSameThread(line, "E_UNCONFIGURED"); return false; }
    }
    m_recipePrepared.table = table; m_recipePrepared.id = id;
    m_recipePrepared.line = line; m_recipePrepared.dispatch = dispatch;
    m_recipePrepared.run = m_pathCoreLiveBookkeeping.currentRunToken;
    m_recipePrepared.cache = GetBaseProgramCache().GetGeneration();
    m_recipePrepared.epoch = m_motion.GetCurrentExecutionEpoch();
    m_recipePrepared.lease = m_programMotionLease;
    m_recipePrepared.ready = true;
    return true;
}

bool NCManager::CommitEDMRecipeSelectionSameThread(const NCBlock& block)
{
    bool table = false; std::uint32_t id = 0U;
    const int line = m_recipePrepared.line;
    if (!m_recipePrepared.ready || !DecodeEDMRecipeSelectionBlock(block, table, id) ||
        table != m_recipePrepared.table || id != m_recipePrepared.id ||
        m_recipePrepared.dispatch != m_currentExecutingBlockDispatchId ||
        m_recipePrepared.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        m_recipePrepared.cache != GetBaseProgramCache().GetGeneration() ||
        m_recipePrepared.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !m_recipePrepared.lease.Matches(m_programMotionLease) ||
        !IsEDMRecipeStoppedSelectionAllowedSameThread(block))
    { RejectEDMRecipeSelectionSameThread(line, "COMMIT_SCOPE"); return false; }
    m_recipePrepared.ready = false;
    const bool selectE1 = table && m_recipe.GetCatalog() && m_recipe.GetCatalog()->schemaVersion == 2U;
    const bool applied = table ? (selectE1 ? m_recipe.SelectTableAndE1(id) : m_recipe.SelectTable(id)) :
        m_recipe.SelectE(static_cast<std::uint16_t>(id));
    if (!applied)
    { RejectEDMRecipeSelectionSameThread(line, EDMRecipe::ErrorName(m_recipe.Current().lastError)); return false; }
    m_recipeIdleSelection = false;
    m_recipeRun = m_pathCoreLiveBookkeeping.currentRunToken;
    m_recipeCache = GetBaseProgramCache().GetGeneration();
    m_recipeEpoch = m_motion.GetCurrentExecutionEpoch();
    m_recipeLease = m_programMotionLease;
    m_recipeHoldSeen = false;
    // Capture the successful table switch now; normal close may occur before
    // the next 100ms publisher. This queues atomic state only, never file I/O.
    const auto current = m_recipe.Current();
    if (table) RememberEDMConditionSelectionSameThread(*m_recipe.GetCatalog(), current);
    RtPrintf("[EDM16] event=SELECT result=PASS kind=%s table=%u E=%u catalog=%u revision=%u fields=%u configured=%u generation=%llu mode=SHADOW_ONLY motion=0 physicalPermit=0 discharge=0\n",
        table ? "TABLE" : "E", static_cast<unsigned int>(current.tableId), static_cast<unsigned int>(current.eCode),
        static_cast<unsigned int>(current.catalogRevision), static_cast<unsigned int>(current.tableRevision),
        static_cast<unsigned int>(current.fieldCount), static_cast<unsigned int>(current.configuredCount),
        static_cast<unsigned long long>(current.generation));
    return true;
}

bool NCManager::IsEDMRecipeEditRequest(const NCBlock& block) noexcept
{
    return NCGCodeSemantics::Contains(block, 39);
}

bool NCManager::IsEDMRecipeEditBlockShapeValid(const NCBlock& block) noexcept
{
    if (block.isEmpty || block.isGoto || block.isBlockSkip || !block.hasG || block.gCode != 39 ||
        block.gCount != 1 || block.gCodes[0] != 39 || block.mCount != 0 || !block.has('P') || !block.has('Q')) return false;
    for (char letter = 'A'; letter <= 'Z'; ++letter)
        if (block.has(letter) && ((letter != 'N' && letter != 'P' && letter != 'Q' && letter != 'R') ||
            !std::isfinite(block.val(letter)))) return false;
    const double field = block.val('P'), operation = block.val('Q');
    if (field < 1.0 || field > 65535.0 || std::floor(field) != field ||
        operation < 0.0 || operation > 4.0 || std::floor(operation) != operation) return false;
    if (block.has('R') != (operation >= 1.0 && operation <= 3.0)) return false;
    if (operation == 1.0 && (block.val('R') < 1.0 || block.val('R') > 65535.0 ||
        std::floor(block.val('R')) != block.val('R'))) return false;
    return operation != 2.0 || block.val('R') == -1.0 || block.val('R') == 1.0;
}

bool NCManager::DecodeEDMRecipeEditBlock(const NCBlock& block, const NCParsedBlock& parsed,
    EDMRecipeControl::Request& request) noexcept
{
    request = EDMRecipeControl::Request{};
    if (!IsEDMRecipeEditBlockShapeValid(block) ||
        parsed.isEmpty || parsed.isBlockSkip || parsed.error != NCParseError::NONE ||
        parsed.controlType != NCParsedControlType::NONE || parsed.dependsOnMacroState ||
        parsed.gCount != 1 || parsed.mCount != 0 || parsed.duplicateAddressMask != 0U) return false;
    const auto unsignedLiteral = [](const std::string& text, std::uint32_t maximum, std::uint32_t& value) noexcept
    {
        value = 0U;
        if (text.empty() || text.size() > 32U) return false;
        for (char c : text)
        {
            if (c < '0' || c > '9') return false;
            const std::uint32_t digit = static_cast<std::uint32_t>(c - '0');
            if (value > maximum / 10U || (value == maximum / 10U && digit > maximum % 10U)) return false;
            value = value * 10U + digit;
        }
        return true;
    };
    std::uint32_t code = 0U, field = 0U, operation = 0U, number = 0U;
    if (!unsignedLiteral(parsed.gExpressions[0], 39U, code) || code != 39U) return false;
    for (char letter = 'A'; letter <= 'Z'; ++letter)
    {
        if (block.has(letter) != parsed.has(letter)) return false;
        if (block.has(letter) && ((letter != 'N' && letter != 'P' && letter != 'Q' && letter != 'R') ||
            !std::isfinite(block.val(letter)))) return false;
    }
    if (!parsed.has('P') || !parsed.has('Q') ||
        !unsignedLiteral(parsed.expression('P'), 65535U, field) || field == 0U ||
        !unsignedLiteral(parsed.expression('Q'), 4U, operation) ||
        block.val('P') != static_cast<double>(field) || block.val('Q') != static_cast<double>(operation)) return false;
    if (parsed.has('N') && (!unsignedLiteral(parsed.expression('N'),
        (std::numeric_limits<std::uint32_t>::max)(), number) || block.val('N') != static_cast<double>(number))) return false;
    const bool needsArgument = operation >= 1U && operation <= 3U;
    if (parsed.has('R') != needsArgument) return false;
    request.fieldId = static_cast<std::uint16_t>(field);
    request.op = static_cast<EDMRecipeControl::Operation>(operation);
    if (!needsArgument) return true;
    const std::string& argument = parsed.expression('R');
    if (argument.empty() || argument.size() > 32U || argument.find('\0') != std::string::npos) return false;
    if (operation == 1U)
    {
        if (!unsignedLiteral(argument, 65535U, number) || number == 0U ||
            block.val('R') != static_cast<double>(number)) return false;
        request.argument = static_cast<EDMRecipe::Value>(number);
    }
    else if (operation == 2U)
    {
        if (argument != "1" && argument != "-1") return false;
        request.argument = argument[0] == '-' ? -1LL : 1LL;
        if (block.val('R') != static_cast<double>(request.argument)) return false;
    }
    else
    {
        if (!EDMRecipe::ParseValue(argument.c_str(), request.argument)) return false;
        // Replay only the resolver's literal conversion for identity checking.
        // The exact fixed-point command above never comes from this double.
        const char* begin = argument.c_str();
        const bool negative = *begin == '-';
        if (negative) ++begin;
        char* end = nullptr;
        double resolved = std::strtod(begin, &end);
        if (negative) resolved = -resolved; // MacroParser applies unary minus first.
        if (end != argument.c_str() + argument.size() || !std::isfinite(resolved) ||
            resolved != block.val('R') || std::signbit(resolved) != std::signbit(block.val('R'))) return false;
    }
    return true;
}

void NCManager::InvalidateEDMRecipeEditContextsSameThread() noexcept
{
    m_recipeEditPrepared.ready = false;
    if (m_recipeControlGeneration == (std::numeric_limits<std::uint64_t>::max)())
        m_recipeControlExhausted = true;
    else if (!m_recipeControlExhausted)
        ++m_recipeControlGeneration;
}

void NCManager::FenceEDMRecipeHoldSameThread() noexcept
{
    if (!m_recipeHoldSeen) InvalidateEDMRecipeEditContextsSameThread();
    m_recipeHoldSeen = true;
}

bool NCManager::ReadEDMRecipeEditContextSameThread(std::uint16_t fieldId,
    EDMRecipeEditContext& context, EDMRecipe::ParameterSnapshot& field) noexcept
{
    context = EDMRecipeEditContext{};
    field = EDMRecipe::ParameterSnapshot{};
    EDMRecipe::Summary summary{};
    if (!ReadEDMRecipeSummarySameThread(summary) || m_recipeControlExhausted ||
        !IsEDMRecipeStoppedAuthorityCurrentSameThread()) return false;
    const EDMRecipeControl::Key key = EDMRecipeControl::CaptureKey(summary);
    if (!EDMRecipeControl::IsKeyValid(key) || !m_recipe.ReadField(fieldId, field)) return false;
    context.key = key;
    context.run = m_pathCoreLiveBookkeeping.currentRunToken;
    context.cache = GetBaseProgramCache().GetGeneration();
    context.epoch = m_motion.GetCurrentExecutionEpoch();
    context.lease = m_programMotionLease;
    context.controlGeneration = m_recipeControlGeneration;
    return true;
}

bool NCManager::ApplyEDMRecipeEditSameThread(const EDMRecipeEditContext& context,
    const EDMRecipeControl::Request& request, EDMRecipeControl::Result& result) noexcept
{
    EDMRecipeControl::ResetResult(request, result);
    ValidateEDMRecipeSelectionSameThread();
    result.before = result.after = m_recipe.Current();
    result.beforeFieldValid = m_recipe.ReadField(request.fieldId, result.beforeField);
    result.afterFieldValid = result.beforeFieldValid;
    result.afterField = result.beforeField;
    if (m_recipeControlExhausted)
    { result.reason = EDMRecipeControl::Reason::AuthorityExhausted; return false; }
    if (context.controlGeneration == 0ULL || context.controlGeneration != m_recipeControlGeneration)
    { result.reason = EDMRecipeControl::Reason::AuthorityStale; return false; }
    if (context.run == 0ULL || context.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        context.cache != GetBaseProgramCache().GetGeneration() ||
        context.epoch != m_motion.GetCurrentExecutionEpoch() || !context.lease.Matches(m_programMotionLease))
    { result.reason = EDMRecipeControl::Reason::ScopeStale; return false; }
    EDMRecipe::Summary summary{};
    if (!ReadEDMRecipeSummarySameThread(summary) || !IsEDMRecipeStoppedAuthorityCurrentSameThread())
    { result.reason = EDMRecipeControl::Reason::AuthorityUnavailable; return false; }
    if (!EDMRecipeControl::Apply(m_recipe, context.key, request, result)) return false;
    // Motion can revoke authority while this NC-thread operation is applied.
    // Recheck once, without a retry or refreshing the submitted context/key.
    if (m_recipeControlExhausted || context.controlGeneration != m_recipeControlGeneration ||
        context.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        context.cache != GetBaseProgramCache().GetGeneration() ||
        context.epoch != m_motion.GetCurrentExecutionEpoch() || !context.lease.Matches(m_programMotionLease) ||
        !m_motion.IsMotionOwnerLeaseCurrent(context.lease) || !IsEDMRecipeStoppedAuthorityCurrentSameThread())
    {
        ClearEDMRecipeSelectionSameThread();
        result.success = result.applied = false;
        result.reason = EDMRecipeControl::Reason::Revoked;
        result.domainError = EDMRecipe::Error::None;
        result.after = m_recipe.Current();
        result.afterFieldValid = m_recipe.ReadField(request.fieldId, result.afterField);
        return false;
    }
    return true;
}

bool NCManager::AreEDMRecipeAxesDrainedSameThread() const noexcept
{
    const MotionStopSettleSnapshot stopped = m_motion.GetStopSettleSnapshot();
    return stopped.publicationGeneration != 0ULL && stopped.sampleSequence != 0ULL &&
        stopped.standstill && !stopped.groupActive && stopped.nonIdleAxisCount == 0U &&
        stopped.commandMovingAxisCount == 0U && stopped.actualMovingAxisCount == 0U &&
        stopped.pdoTargetVelocityNonzeroAxisCount == 0U && stopped.commandQueueDepth == 0U &&
        stopped.commandIngressDepth == 0U && stopped.commandReplayDepth == 0U &&
        !m_motion.HasPendingSafetyOrRecoveryRequests() && m_motion.IsGroupDone() && m_motion.IsGroupNCDrained() &&
        m_motion.GetQueueSize() == 0U && m_motion.GetCommandIngressSize() == 0U && m_motion.GetCommandReplaySize() == 0U;
}

bool NCManager::IsEDMRecipeIdleAuthorityCurrentSameThread() const noexcept
{
    const bool idleState = m_state == NCState::IDLE || m_state == NCState::READY || m_state == NCState::P_END;
    return idleState && !m_programRunStartPending && m_pendingProgramRunPhase == ProgramRunStartPhase::IDLE &&
        !m_gotoQueueTailRebasePending && !m_manualAutoRunning &&
        m_holdResumeAdmissionKind == HoldResumeAdmissionKind::NONE &&
        m_resetContinuationPhase == ResetContinuationPhase::IDLE && !m_resetSafetyOutputHoldActive &&
        !m_gapRecovery.active && !m_gapPending.active &&
        !AlarmManager::GetInstance().HasAlarm() && !Close_System_Com_flag &&
        m_edmState != EDMState::NOT_READY && !Homing.IsActive() && !m_isG66Active && m_macroStack.empty() &&
        !m_pathFeed.pending && !m_pathArc.pending && !m_pathReplay.pending &&
        !m_pathHold.armed && !m_pathHold.bound && !m_gapDryRun.active && !m_gapPath.active && !m_gapWindow.active &&
        !m_cncFeed.selected && !m_cncFeed.active && m_cncFeed.count == 0U &&
        !m_pathCoreCommittedBookkeeping.pendingAdmission && !m_cutterLine.leadOutRequired &&
        AreEDMRecipeAxesDrainedSameThread();
}

// HMI edits are bounded changes to the NC-owned shadow condition image.
// RUN/HOLD/axis motion do not make these logical values read-only. Hardware
// output and program receipts remain separate; catalog replacement still drains.
bool NCManager::IsEDMRecipeHmiEditAllowedSameThread() const noexcept
{
    if (m_recipeRecoveryDisplayOnly) return false;
    const bool stateAllowed = m_state == NCState::READY || m_state == NCState::IDLE ||
        m_state == NCState::RUN || m_state == NCState::HOLD || m_state == NCState::P_END;
    return stateAllowed && m_resetContinuationPhase == ResetContinuationPhase::IDLE &&
        !m_resetSafetyOutputHoldActive && !AlarmManager::GetInstance().HasAlarm() &&
        !Close_System_Com_flag && m_edmState != EDMState::NOT_READY;
}

void NCManager::ReadEDMRecipeHmiContextSameThread(EDMRecipeControl::HmiContext& output) noexcept
{
    ValidateEDMRecipeSelectionSameThread();
    output = EDMRecipeControl::HmiContext{};
    const EDMRecipe::Summary summary = m_recipe.Current();
    const EDMRecipe::Catalog* catalog = m_recipe.GetCatalog();
    output.generation = summary.generation;
    output.controlGeneration = m_recipeControlGeneration;
    if (catalog)
    {
        output.schemaVersion = catalog->schemaVersion;
        output.profileId = catalog->profileId;
        output.definitionRevision = catalog->definitionRevision;
        output.catalogRevision = catalog->catalogRevision;
    }
    output.tableRevision = summary.tableRevision; output.tableId = summary.tableId; output.eCode = summary.eCode;
    const bool authority = !m_recipeControlExhausted &&
        m_recipeControlGeneration != (std::numeric_limits<std::uint64_t>::max)();
    const bool idle = authority && IsEDMRecipeIdleAuthorityCurrentSameThread();
    output.canReload = idle && m_recipe.CanRefreshAfterCatalogExchange();
    output.canEdit = authority && m_recipe.Ready() && catalog && catalog->schemaVersion == 2U &&
        IsEDMRecipeHmiEditAllowedSameThread();
}

bool NCManager::ApplyEDMRecipeHmiCommandSameThread(const EDMRecipeControl::HmiContext& expected,
    const EDMRecipeControl::HmiCommand& command, EDMRecipeControl::HmiResult& result) noexcept
{
    using namespace EDMRecipeControl;
    result = HmiResult{};
    HmiContext current{};
    ReadEDMRecipeHmiContextSameThread(current);
    result.after = current;
    if (!IsHmiCommandCanonical(command) || command.operation == HmiOperation::ReloadCatalog)
    { result.status = HmiStatus::InvalidCommand; return false; }
    if (!SameHmiKey(expected, current)) { result.status = HmiStatus::Stale; return false; }
    if (!current.canEdit) { result.status = HmiStatus::NotAllowed; return false; }
    bool applied = false;
    switch (command.operation)
    {
    case HmiOperation::SelectTable: applied = m_recipe.SelectTableAndE1(command.tableId); break;
    case HmiOperation::SelectE: applied = m_recipe.SelectE(command.eCode); break;
    case HmiOperation::SelectStage: applied = m_recipe.SelectStage(command.fieldId, command.stageId); break;
    case HmiOperation::StepStage: applied = m_recipe.StepStage(command.fieldId, command.step); break;
    case HmiOperation::SetActual:
        applied = command.stageId == 0U ? m_recipe.SetActual(command.fieldId, command.actualValue) :
            m_recipe.SetActualFromStage(command.fieldId, command.stageId, command.actualValue);
        break;
    case HmiOperation::ResetOverride: applied = m_recipe.ResetOverride(command.fieldId); break;
    default: result.status = HmiStatus::InvalidCommand; return false;
    }
    if (!applied)
    {
        result.status = HmiStatus::DomainRejected; result.domainError = m_recipe.Current().lastError;
        ReadEDMRecipeHmiContextSameThread(result.after);
        return false;
    }
    // Preserve an existing NC binding, but never manufacture one for a HMI
    // selection. A retained/manual selection remains logical-only until G38/E
    // establishes the ordinary stopped program receipt.
    if (m_recipeRun == 0ULL) m_recipeIdleSelection = true;
    // RESET/ALARM may revoke admission while this NC-owned operation executes.
    // Do not accept the write under the old editing context after that boundary.
    if (!IsEDMRecipeHmiEditAllowedSameThread() || expected.controlGeneration != m_recipeControlGeneration)
    {
        ClearEDMRecipeSelectionSameThread();
        result.status = HmiStatus::NotAllowed;
        ReadEDMRecipeHmiContextSameThread(result.after);
        return false;
    }
    InvalidateEDMRecipeEditContextsSameThread();
    result.status = HmiStatus::Success;
    ReadEDMRecipeHmiContextSameThread(result.after);
    return true;
}

bool NCManager::ReloadValidatedEDMRecipeCatalogSameThread(const EDMRecipeControl::HmiContext& expected,
    std::unique_ptr<EDMRecipe::Catalog>& candidate, std::unique_ptr<const EDMRecipe::Catalog>& retired,
    EDMRecipeControl::HmiResult& result) noexcept
{
    using namespace EDMRecipeControl;
    result = HmiResult{};
    HmiContext current{};
    ReadEDMRecipeHmiContextSameThread(current);
    result.after = current;
    if (!SameHmiKey(expected, current)) { result.status = HmiStatus::Stale; return false; }
    if (!current.canReload || !IsEDMRecipeIdleAuthorityCurrentSameThread())
    { result.status = HmiStatus::NotAllowed; return false; }
    if (!candidate || retired || candidate->schemaVersion != 2U || candidate->profileId == 0U ||
        candidate->definitionRevision == 0U || candidate->catalogRevision == 0U)
    { result.status = HmiStatus::InvalidCommand; return false; }
    const EDMRecipe::Catalog* previous = m_recipe.GetCatalog();
    if (previous && !m_recipeRecoveryDisplayOnly && previous->schemaVersion == 2U &&
        (candidate->profileId != previous->profileId || candidate->catalogRevision < previous->catalogRevision ||
            candidate->definitionRevision < previous->definitionRevision))
    { result.status = HmiStatus::Stale; return false; }
    if (previous && !m_recipeRecoveryDisplayOnly)
    {
        for (const auto& oldTable : previous->tables)
            for (const auto& newTable : candidate->tables)
                if (oldTable.id == newTable.id && newTable.revision < oldTable.revision)
                { result.status = HmiStatus::Stale; return false; }
    }
    // A catalog reload must never blank the operator display. Prefer the prior
    // COND and otherwise the first configured table; every table switch starts E1.
    std::uint32_t reloadedTable = 0U;
    {
        if (current.generation >= (std::numeric_limits<std::uint64_t>::max)() - 1ULL)
        { result.status = HmiStatus::NotAllowed; return false; }
        for (const auto& table : candidate->tables)
        {
            if (!table.rowPresent[0] || table.cells.size() < candidate->fields.size()) continue;
            bool configured = false;
            for (std::size_t f = 0U; f < candidate->fields.size(); ++f)
                configured = configured || table.cells[f].kind != EDMRecipe::CellKind::Unset;
            if (configured && (reloadedTable == 0U || table.id == current.tableId)) reloadedTable = table.id;
        }
        if (reloadedTable == 0U)
        { result.status = HmiStatus::DomainRejected; result.domainError = EDMRecipe::Error::EmptyRow; return false; }
    }
    // Worker has already validated all rows/fields/stages. The NC owner only
    // checks the optimistic identity, admission and bounded revision metadata.
    if (!m_recipe.CanRefreshAfterCatalogExchange())
    { result.status = HmiStatus::NotAllowed; return false; }
    m_edmRecipeTest.Revoke();
    InvalidateEDMRecipeEditContextsSameThread();
    if (!m_recipeStore.ExchangeValidatedOffThreadCandidate(candidate, retired))
    { result.status = HmiStatus::InvalidCommand; return false; }
    (void)m_recipe.RefreshAfterCatalogExchange();
    m_recipeIdleSelection = false; m_recipeHoldSeen = false;
    m_recipeRun = m_recipeCache = 0ULL;
    m_recipeEpoch = MOTION_EXECUTION_EPOCH_INVALID; m_recipeLease = MotionOwnerLease{};
    m_recipePrepared.ready = false;
    // E1 and both required generation advances were preflighted above against
    // this same exclusively owned catalog. Retain display state, not NC authority.
    if (!m_recipe.SelectTableAndE1(reloadedTable))
    { result.status = HmiStatus::DomainRejected; result.domainError = m_recipe.Current().lastError; return false; }
    m_recipeIdleSelection = true;
    m_recipeRecoveryDisplayOnly = false;
    result.status = HmiStatus::Success;
    ReadEDMRecipeHmiContextSameThread(result.after);
    return true;
}

void NCManager::RejectEDMRecipeEditSameThread(int line, const char* reason, EDMRecipe::Error domainError)
{
    RetainEDMRecipeSelectionSameThread();
    RtPrintf("[EDM17] event=COMMAND result=REJECT line=%d reason=%s domain=%s physicalPermit=0 discharge=0\n",
        line, reason, EDMRecipe::ErrorName(domainError));
    AlarmManager::GetInstance().Trigger(AlarmManager::G_Code_Invalid_parameter, line);
    ChangeState(NCState::ALARM);
}

bool NCManager::PrepareEDMRecipeEditSameThread(const NCBlock& block, int sourcePC, int line, NCBlockDispatchId dispatch)
{
    if(m_edm237.authorized&&(m_edm237.program==240U||m_edm237.program==241U||m_edm237.program==242U||m_edm237.program==243U||m_edm237.program==244U||m_edm237.program==245U||m_edm237.program==246U||m_edm237.program==247U||m_edm237.program==248U))return PrepareEDM240RecipeBridgeSameThread(block,sourcePC,line,dispatch);
    m_recipeEditPrepared.ready = false;
    const NCProgramCacheLine* source = GetBaseProgramCache().TryGetLine(sourcePC);
    if (m_mode != NCOperationMode::MEMORY || !m_macroStack.empty() || !source ||
        source->sourceLineNumber != line || dispatch == NC_BLOCK_DISPATCH_ID_INVALID ||
        dispatch != m_currentExecutingBlockDispatchId)
    { RejectEDMRecipeEditSameThread(line, "PARSED_IDENTITY", EDMRecipe::Error::None); return false; }
    EDMRecipeControl::Request request{};
    if (!DecodeEDMRecipeEditBlock(block, source->parsedBlock, request))
    { RejectEDMRecipeEditSameThread(line, "LITERAL_OR_BLOCK_SHAPE", EDMRecipe::Error::None); return false; }
    request.requestId = dispatch;
    EDMRecipeEditContext context{};
    EDMRecipe::ParameterSnapshot field{};
    if (!ReadEDMRecipeEditContextSameThread(request.fieldId, context, field))
    {
        const char* reason = m_recipeControlExhausted ? "AUTHORITY_EXHAUSTED" :
            (!m_recipe.Ready() ? "NOT_READY" : (!m_recipe.Current().rowSelected ? "NO_SELECTION" :
                (!m_recipe.ReadField(request.fieldId, field) ? "FIELD_UNAVAILABLE" : "AUTHORITY_UNAVAILABLE")));
        RejectEDMRecipeEditSameThread(line, reason, EDMRecipe::Error::None);
        return false;
    }
    m_recipeEditPrepared.context = context;
    m_recipeEditPrepared.request = request;
    m_recipeEditPrepared.sourcePC = sourcePC;
    m_recipeEditPrepared.line = line;
    m_recipeEditPrepared.dispatch = dispatch;
    m_recipeEditPrepared.ready = true;
    return true;
}

bool NCManager::CommitEDMRecipeEditSameThread(const NCBlock& block)
{
    if(m_edm237.authorized&&(m_edm237.program==240U||m_edm237.program==241U||m_edm237.program==242U||m_edm237.program==243U||m_edm237.program==244U||m_edm237.program==245U||m_edm237.program==246U||m_edm237.program==247U||m_edm237.program==248U))return CommitEDM240RecipeBridgeSameThread(block);
    const int line = m_recipeEditPrepared.line;
    const NCProgramCacheLine* source = GetBaseProgramCache().TryGetLine(m_recipeEditPrepared.sourcePC);
    EDMRecipeControl::Request decoded{};
    if (!m_recipeEditPrepared.ready || !source || source->sourceLineNumber != line ||
        m_recipeEditPrepared.dispatch != m_currentExecutingBlockDispatchId ||
        !DecodeEDMRecipeEditBlock(block, source->parsedBlock, decoded) ||
        decoded.op != m_recipeEditPrepared.request.op || decoded.fieldId != m_recipeEditPrepared.request.fieldId ||
        decoded.argument != m_recipeEditPrepared.request.argument)
    { RejectEDMRecipeEditSameThread(line, "COMMIT_SOURCE", EDMRecipe::Error::None); return false; }
    // Consume the prepared command before applying it; no reusable dispatch token.
    m_recipeEditPrepared.ready = false;
    EDMRecipeControl::Result result{};
    if (!ApplyEDMRecipeEditSameThread(m_recipeEditPrepared.context, m_recipeEditPrepared.request, result))
    {
        RejectEDMRecipeEditSameThread(line, EDMRecipeControl::ReasonName(result.reason), result.domainError);
        return false;
    }
    char base[40]{}, actual[40]{};
    if (!result.afterFieldValid || !EDMRecipe::FormatValue(result.afterField.baseValue, base, sizeof(base)) ||
        !EDMRecipe::FormatValue(result.afterField.actualValue, actual, sizeof(actual)))
    { RejectEDMRecipeEditSameThread(line, "RESULT_INVARIANT", EDMRecipe::Error::None); return false; }
    RtPrintf("[EDM17] event=COMMAND result=PASS request=%llu op=%s applied=%u field=%u table=%u E=%u stage=%u base=%s actual=%s modified=%u generation=%llu mode=SHADOW_ONLY motion=0 physicalPermit=0 discharge=0\n",
        static_cast<unsigned long long>(result.requestId), EDMRecipeControl::OperationName(result.op), result.applied ? 1U : 0U,
        static_cast<unsigned int>(result.fieldId), static_cast<unsigned int>(result.after.tableId),
        static_cast<unsigned int>(result.after.eCode), static_cast<unsigned int>(result.afterField.stageId), base, actual,
        result.afterField.modified ? 1U : 0U, static_cast<unsigned long long>(result.after.generation));
    return true;
}

bool NCManager::InstallEDMGapInputBeforeStart(const EDMGapInput::Profile& profile, EtherCatMaster* master) noexcept
{
    if (m_edmGapMaster != nullptr || master == nullptr) return false;
    m_edmGapMaster = master;
    LARGE_INTEGER frequency{};
    if (RtQueryPerformanceFrequency(&frequency) && frequency.QuadPart > 0 &&
        static_cast<std::uint64_t>(frequency.QuadPart) <= (std::numeric_limits<std::uint64_t>::max)() / 1000ULL)
        m_edmGapFrequency = static_cast<std::uint64_t>(frequency.QuadPart);
    const bool initialized = m_edmGapChannel.Initialize(profile);
    (void)m_edmGapPhysicalShadow.Initialize(profile); // Frozen configured calibration; observer only.
    (void)master->ConfigureGapPhysicalRTShadowBeforeStart(profile); // PDO-owned diagnostic only; SIM result unchanged.
    (void)m_motion.BindEDMPhysicalSourceBeforeStart(master); // Separate source binding; no legacy motion permit.
    return initialized;
}

bool NCManager::IsEDM173DiagnosticQuietSameThread() const noexcept
{
    // Use only NC-owned cached state. No RT channel read, publication, or
    // control mutation is introduced by asking whether passive IO may wait.
    const auto& s = m_edm94;
    // EDM231 FIX2: passive diagnostics stay quiet during HOLD braking and
    // HELD. Fault/cancel/reset/release still reopen the existing diagnostics.
    const NCState state = m_state.load(std::memory_order_relaxed);
    return (state == NCState::RUN || state == NCState::HOLD) &&
        s.program && s.latched && EDM173::IsArm(s.gapHistoryArm) &&
        !s.complete && !s.cancelled && !s.resetSeen && !s.resetCommitted &&
        !s.releaseRequested && !s.rtFaultReported &&
        !s.feedback.faulted && !s.feedback.cancelled;
}

bool NCManager::IsEDMZFixtureDiagnosticQuietSameThread() const noexcept
{
    if ((m_gapDryRun.physicalRecipeCaptureTest || m_gapDryRun.physicalServoRTTest) && m_gapDryRun.active) return true;
    return m_gapDryRun.physicalZFixtureTest &&
        (m_gapDryRun.active || m_edmZFixture.phase == EDMZFixturePhase::Receipt ||
            m_edmZFixture.cleanupPending || m_edmZFixture.resetPending ||
            m_edmZFixture.failureAlarmPending);
}

EDMGapInput::Snapshot NCManager::ReadEDMGapInputSnapshotSameThread() noexcept
{
    EDMAnalogInput::Snapshot input{};
    const bool readSucceeded = m_edmGapMaster != nullptr && m_edmGapMaster->ReadGapAnalogInputSnapshot(input);
    // Read before the existing QPC acquisition: a later PDO publication must
    // never be compared with an earlier NC clock. Only READY/quiet NC reads
    // this diagnostic bank; formatting and logging remain on the 1 Hz guard.
    const bool rtShadowDiagnostic = !IsEDMZFixtureDiagnosticQuietSameThread() &&
        m_state.load(std::memory_order_relaxed) == NCState::READY;
    EDM64::Snapshot rtPhysical{};
    EDM53::Stamp rtPhysicalStamp{};
    EDM65::Snapshot recipePhysical{};
    EDM53::Stamp recipePhysicalStamp{};
    if (rtShadowDiagnostic && m_edmGapMaster != nullptr)
    {
        rtPhysicalStamp = m_edmGapMaster->ReadGapPhysicalRTShadow(rtPhysical);
        recipePhysicalStamp = m_edmGapMaster->ReadGapPhysicalRecipeRTShadow(recipePhysical);
    }
    LARGE_INTEGER counter{};
    bool clockValid = m_edmGapFrequency != 0ULL && RtQueryPerformanceCounter(&counter) && counter.QuadPart >= 0;
    std::uint64_t nowMs = 0ULL;
    if (clockValid)
    {
        const std::uint64_t ticks = static_cast<std::uint64_t>(counter.QuadPart);
        const std::uint64_t seconds = ticks / m_edmGapFrequency;
        const std::uint64_t fraction = ticks % m_edmGapFrequency * 1000ULL / m_edmGapFrequency;
        clockValid = seconds <= ((std::numeric_limits<std::uint64_t>::max)() - fraction) / 1000ULL;
        if (clockValid) nowMs = seconds * 1000ULL + fraction;
    }
    // Reuse the exact PDO copy and QPC reading used above. The physical shadow
    // never becomes a Motion source and never changes the legacy millisecond path.
    std::uint64_t nowUs = 0ULL;
    const bool shadowClockValid = clockValid &&
        EDMAnalogInput::Publisher::QpcMicroseconds(counter.QuadPart, m_edmGapFrequency, nowUs);
    const EDM63::Receipt physical = m_edmGapPhysicalShadow.Observe(
        input, readSucceeded, nowUs, shadowClockValid, m_edmGapFrequency);
    const EDMGapInput::Snapshot value = m_edmGapChannel.Observe(input, nowMs, clockValid);
    if (!m_edmGapLogSeen || (clockValid && nowMs >= m_edmGapLastLogMs && nowMs - m_edmGapLastLogMs >= 1000ULL))
    {
        if (!IsEDMZFixtureDiagnosticQuietSameThread() && !IsEDM173DiagnosticQuietSameThread())
            RtPrintf("[EDM18] event=GAP_INPUT source=%s AD=%u device=%u raw=%d rawAvailable=%u boardMv=%d boardValid=%u zeroClamped=%u gapMv=%d liveValid=%u calibration=%u status=%s quality=%s band=%s seq=%llu ageMs=%llu physicalPermit=0 discharge=0\n",
                EDMGap::SourceName(value.configuredSource), static_cast<unsigned>(value.adIndex), static_cast<unsigned>(value.deviceId),
                static_cast<int>(value.rawCode), value.rawAvailable ? 1U : 0U, static_cast<int>(value.boardVoltageMv), value.boardVoltageValid ? 1U : 0U,
                value.zeroClamped ? 1U : 0U, static_cast<int>(value.voltageMv), value.liveVoltageValid ? 1U : 0U, value.calibrationConfirmed ? 1U : 0U,
                EDMGapInput::StatusName(value.inputStatus), EDMGap::QualityName(value.gap.quality), EDMGap::BandName(value.gap.band),
                static_cast<unsigned long long>(value.sampleSequence), static_cast<unsigned long long>(value.ageMs));
        // Logging remains on the existing cadence. A READY-only shadow line
        // adds no console IO to running, holding, or stopping NC motion.
        if (!IsEDMZFixtureDiagnosticQuietSameThread() && m_state.load(std::memory_order_relaxed) == NCState::READY)
        {
            char line[896]{};
            const int length = std::snprintf(line, sizeof(line),
                "[EDM64] event=PHYSICAL_SOURCE mode=SHADOW_ONLY state=%s reason=%s usable=%u observerGen=%llu AD=%u device=%u channel=%u producerGeneration=%llu qpcHz=%llu seq=%llu capturedUs=%llu ageUs=%llu ageKnown=%u raw=%d gapMv=%d expectedProfile=%u expectedCalibration=%u calibrationEvidence=CONFIGURED timestamp=HOST_PDO_RECEIPT producerScope=OBJECT_LIFETIME physicalPermit=0 discharge=0\n",
                EDM63::StateName(physical.state), EDM63::ReasonName(physical.reason), physical.Usable() ? 1U : 0U,
                static_cast<unsigned long long>(physical.observerGeneration),
                static_cast<unsigned>(physical.sample.adIndex), static_cast<unsigned>(physical.sample.deviceId),
                static_cast<unsigned>(physical.sample.channelId),
                static_cast<unsigned long long>(physical.sample.producerGeneration),
                static_cast<unsigned long long>(physical.sample.qpcFrequency),
                static_cast<unsigned long long>(physical.sample.sequence),
                static_cast<unsigned long long>(physical.sample.capturedAtUs),
                static_cast<unsigned long long>(physical.ageUs), physical.ageKnown ? 1U : 0U,
                static_cast<int>(physical.sample.rawCode), static_cast<int>(physical.conversion.voltageMv),
                static_cast<unsigned>(physical.expectedProfileRevision),
                static_cast<unsigned>(physical.expectedCalibrationRevision));
            if (length > 0 && static_cast<std::size_t>(length) < sizeof(line)) RtPrintf("%s", line);
            else RtPrintf("[EDM64] event=PHYSICAL_SOURCE reason=LOG_FORMAT physicalPermit=0 discharge=0\n");
            // This is the previously copied bank, projected against the later
            // NC clock. A silent/stopped PDO producer is explicitly not usable.
            EDM64::PhysicalRTShadow::ValidateRead(rtPhysical, rtPhysicalStamp,
                nowUs, shadowClockValid, m_edmGapFrequency);
            const bool rtAgeKnown = shadowClockValid && rtPhysical.input.sample.capturedAtUs != 0ULL &&
                nowUs >= rtPhysical.input.sample.capturedAtUs;
            const std::uint64_t rtAgeUs = rtAgeKnown ? nowUs - rtPhysical.input.sample.capturedAtUs : 0ULL;
            const bool rtUsable = rtPhysical.readerState == EDM64::ReaderState::Fresh &&
                rtPhysical.state == EDM64::State::Valid && rtPhysical.output.valid;
            // EDM64-FIX1: native console records are clipped after 511 bytes.
            // Keep each complete part below that boundary, including its newline.
            // Shared publication/cycle/seq identify the same copied RT snapshot.
            char rtLine[480]{};
            int rtLength = std::snprintf(rtLine, sizeof(rtLine),
                "[EDM64-FIX1] event=RT_PHYSICAL_SHADOW part=STATE publication=%llu cycle=%llu seq=%llu state=%s reason=%s reader=%s inputState=%s inputReason=%s kernelFault=%s usable=%u outputValid=%u appliedToMotion=0 physicalPermit=0 discharge=0\n",
                static_cast<unsigned long long>(rtPhysical.publication),
                static_cast<unsigned long long>(rtPhysical.cycle),
                static_cast<unsigned long long>(rtPhysical.input.sample.sequence),
                EDM64::StateName(rtPhysical.state), EDM64::ReasonName(rtPhysical.reason),
                EDM64::ReaderStateName(rtPhysical.readerState), EDM63::StateName(rtPhysical.input.state),
                EDM63::ReasonName(rtPhysical.input.reason), EDM64::FaultName(rtPhysical.output.fault),
                rtUsable ? 1U : 0U, rtPhysical.output.valid ? 1U : 0U);
            if (rtLength > 0 && static_cast<std::size_t>(rtLength) < sizeof(rtLine)) RtPrintf("%s", rtLine);
            else RtPrintf("[EDM64-FIX1] event=RT_PHYSICAL_SHADOW part=STATE publication=%llu cycle=%llu seq=%llu reason=LOG_FORMAT appliedToMotion=0 physicalPermit=0 discharge=0\n",
                static_cast<unsigned long long>(rtPhysical.publication),
                static_cast<unsigned long long>(rtPhysical.cycle),
                static_cast<unsigned long long>(rtPhysical.input.sample.sequence));
            rtLength = std::snprintf(rtLine, sizeof(rtLine),
                "[EDM64-FIX1] event=RT_PHYSICAL_SHADOW part=SOURCE publication=%llu cycle=%llu seq=%llu observerGen=%llu producerGen=%llu capturedUs=%llu observedUs=%llu ageUs=%llu ageKnown=%u maxAgeUs=%llu maxTickGapUs=%llu timestamp=HOST_PDO_RECEIPT\n",
                static_cast<unsigned long long>(rtPhysical.publication),
                static_cast<unsigned long long>(rtPhysical.cycle),
                static_cast<unsigned long long>(rtPhysical.input.sample.sequence),
                static_cast<unsigned long long>(rtPhysical.input.observerGeneration),
                static_cast<unsigned long long>(rtPhysical.input.sample.producerGeneration),
                static_cast<unsigned long long>(rtPhysical.input.sample.capturedAtUs),
                static_cast<unsigned long long>(rtPhysical.observedUs),
                static_cast<unsigned long long>(rtAgeUs), rtAgeKnown ? 1U : 0U,
                static_cast<unsigned long long>(rtPhysical.maxAgeUs),
                static_cast<unsigned long long>(rtPhysical.maxTickGapUs));
            if (rtLength > 0 && static_cast<std::size_t>(rtLength) < sizeof(rtLine)) RtPrintf("%s", rtLine);
            else RtPrintf("[EDM64-FIX1] event=RT_PHYSICAL_SHADOW part=SOURCE publication=%llu cycle=%llu seq=%llu reason=LOG_FORMAT appliedToMotion=0 physicalPermit=0 discharge=0\n",
                static_cast<unsigned long long>(rtPhysical.publication),
                static_cast<unsigned long long>(rtPhysical.cycle),
                static_cast<unsigned long long>(rtPhysical.input.sample.sequence));
            const bool rtValuesFinite = std::isfinite(rtPhysical.output.rawVoltageV) &&
                std::isfinite(rtPhysical.output.filteredVoltageV) &&
                std::isfinite(rtPhysical.output.targetMmPerMin) &&
                std::isfinite(rtPhysical.output.speedMmPerMin);
            rtLength = rtValuesFinite ? std::snprintf(rtLine, sizeof(rtLine),
                "[EDM64-FIX1] event=RT_PHYSICAL_SHADOW part=VALUES publication=%llu cycle=%llu seq=%llu configRevision=%llu mode=RT_SHADOW_ONLY config=FIXED_SHADOW rawV=%.3f filteredV=%.3f targetMmMin=%.3f advisoryMmMin=%.3f short=%u feedInhibit=%u referenceV=60 capMmMin=30 calibrationEvidence=CONFIGURED appliedToMotion=0 physicalPermit=0 discharge=0\n",
                static_cast<unsigned long long>(rtPhysical.publication),
                static_cast<unsigned long long>(rtPhysical.cycle),
                static_cast<unsigned long long>(rtPhysical.input.sample.sequence),
                static_cast<unsigned long long>(rtPhysical.configRevision),
                rtPhysical.output.rawVoltageV, rtPhysical.output.filteredVoltageV,
                rtPhysical.output.targetMmPerMin, rtPhysical.output.speedMmPerMin,
                rtPhysical.output.shortActive ? 1U : 0U, rtPhysical.output.feedInhibited ? 1U : 0U) : -1;
            if (rtLength > 0 && static_cast<std::size_t>(rtLength) < sizeof(rtLine)) RtPrintf("%s", rtLine);
            else RtPrintf("[EDM64-FIX1] event=RT_PHYSICAL_SHADOW part=VALUES publication=%llu cycle=%llu seq=%llu reason=%s appliedToMotion=0 physicalPermit=0 discharge=0\n",
                static_cast<unsigned long long>(rtPhysical.publication),
                static_cast<unsigned long long>(rtPhysical.cycle),
                static_cast<unsigned long long>(rtPhysical.input.sample.sequence),
                rtValuesFinite ? "LOG_FORMAT" : "NONFINITE_VALUES");
            EDM65::PhysicalRecipeRTShadow::ValidateRead(recipePhysical, recipePhysicalStamp,
                nowUs, shadowClockValid, m_edmGapFrequency);
            LogEDM65PhysicalRecipeShadow(recipePhysical);
            LogEDM66PhysicalCoverage(recipePhysical);
        }
        m_edmGapLogSeen = true; m_edmGapLastLogMs = nowMs;
        m_edmGapLastLogStatus = value.inputStatus; m_edmGapLastLogBand = value.gap.band;
    }
    return value;
}

void NCManager::ReadEDMRecipeDisplaySameThread(EDMRecipe::ActiveSnapshot& output) noexcept
{
    ValidateEDMRecipeSelectionSameThread();
    m_recipe.CopySnapshot(output); // Logical conditions remain visible through HOLD.
}

bool NCManager::InstallEDMVoltageProfileBeforeStart(const EDMVoltage::Profile& profile) noexcept
{
    if (m_gapDryRun.active || m_gapDryRunSerial != 0ULL)
    {
        m_edmVoltageTest.Revoke();
        return false;
    }
    return m_edmVoltageTest.Install(profile);
}

EDMVoltage::Snapshot NCManager::ReadEDMVoltageSnapshotSameThread() noexcept
{
    if (!m_gapDryRun.active || !m_gapDryRun.voltageTest || m_gapDryRun.paused ||
        m_mode != NCOperationMode::MEMORY || m_state != NCState::RUN ||
        AlarmManager::GetInstance().HasAlarm() || Close_System_Com_flag ||
        m_edmState == EDMState::NOT_READY ||
        m_gapDryRun.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        m_gapDryRun.cache != GetBaseProgramCache().GetGeneration() ||
        m_gapDryRun.dispatch != m_waitingBlockDispatchId ||
        m_gapDryRun.epoch != m_motion.GetCurrentExecutionEpoch() ||
        !m_gapDryRun.lease.Matches(m_programMotionLease) ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_gapDryRun.lease) ||
        m_motion.HasPendingSafetyOrRecoveryRequests() || m_isG66Active || !m_macroStack.empty())
        m_edmVoltageTest.Revoke();
    return m_edmVoltageTest.Current();
}

bool NCManager::IsGapDryRunBlockShapeValid(const NCBlock& block) noexcept
{
    if(block.has('P')&&block.val('P')==274.0&&block.has('L')&&(block.val('L')==234.0||block.val('L')==235.0||block.val('L')==236.0||block.val('L')==237.0||block.val('L')==238.0||block.val('L')==240.0||block.val('L')==241.0||block.val('L')==242.0||block.val('L')==243.0||block.val('L')==244.0||block.val('L')==245.0||block.val('L')==246.0||block.val('L')==247.0||block.val('L')==248.0))return IsEDM234BracketShapeValid(block);
    if (block.has('P') && (block.val('P')==292.0 || block.val('P')==293.0 || block.val('P')==294.0 || block.val('P')==295.0 || block.val('P')==296.0 || block.val('P')==297.0 || block.val('P')==298.0 || block.val('P')==299.0 || block.val('P')==300.0 || block.val('P')==301.0)) return IsEDM173BracketShapeValid(block);
    if (block.has('P') && (block.val('P') == 158.0 || block.val('P') == 159.0 ||
        block.val('P') == 160.0 || block.val('P') == 161.0))
    {
        const bool arm = block.val('P') == 158.0 || block.val('P') == 160.0;
        if (block.isEmpty || block.isGoto || block.isBlockSkip || !block.hasG || block.gCode != 180 ||
            block.gCount != 1 || block.gCodes[0] != 180 || block.mCount) return false;
        for (char a = 'A'; a <= 'Z'; ++a)
            if (block.has(a) && ((a != 'N' && a != 'P' && (!arm || (a != 'Q' && a != 'R' && a != 'S'))) ||
                !std::isfinite(block.val(a)))) return false;
        return !arm || (block.has('Q') && block.has('R') && block.has('S') &&
            block.val('Q') == 8.0 && block.val('R') == 1.0 && block.val('S') == 1.0);
    }

    if (block.has('P') && block.val('P') == 131.0)
    {
        if (block.isEmpty || block.isGoto || block.isBlockSkip || !block.hasG || block.gCode != 180 ||
            block.gCount != 1 || block.gCodes[0] != 180 || block.mCount || !block.has('B')) return false;
        EDM116::Mode mode{};
        if (!EDM116::ParseMode(block.val('B'), mode)) return false;
        for (char a = 'A'; a <= 'Z'; ++a)
            if (block.has(a) && ((a != 'N' && a != 'P' && a != 'B') || !std::isfinite(block.val(a)))) return false;
        return true;
    }
    if (block.has('P') && (block.val('P') == 85.0 || block.val('P') == 86.0 ||
        block.val('P') == 87.0 || block.val('P') == 88.0 ||
        block.val('P') == 89.0 || block.val('P') == 90.0 ||
        block.val('P') == 91.0 || block.val('P') == 92.0 ||
        block.val('P') == 93.0 || block.val('P') == 94.0 ||
        block.val('P') == 95.0 || block.val('P') == 96.0 ||
        block.val('P') == 97.0 || block.val('P') == 98.0 ||
        block.val('P') == 99.0 || block.val('P') == 100.0 ||
        block.val('P') == 101.0 || block.val('P') == 102.0 ||
        block.val('P') == 103.0 || block.val('P') == 104.0 ||
        block.val('P') == 105.0 || block.val('P') == 106.0 ||
        block.val('P') == 107.0 || block.val('P') == 108.0 ||
        block.val('P') == 109.0 || block.val('P') == 110.0 ||
        block.val('P') == 111.0 || block.val('P') == 112.0 ||
        block.val('P') == 113.0 || block.val('P') == 114.0 || block.val('P') == 115.0 || block.val('P') == 116.0 || block.val('P') == 117.0 || block.val('P') == 118.0 || block.val('P') == 119.0 || block.val('P') == 120.0 || block.val('P') == 121.0 || block.val('P') == 122.0 || block.val('P') == 123.0 || block.val('P') == 124.0 || block.val('P') == 125.0 || block.val('P') == 126.0 || block.val('P') == 127.0 || block.val('P') == 128.0 || block.val('P') == 129.0 || block.val('P') == 130.0 || block.val('P') == 131.0 || block.val('P') == 132.0 || block.val('P') == 133.0 || block.val('P') == 134.0 || block.val('P') == 135.0 || block.val('P') == 136.0 || block.val('P') == 137.0 || block.val('P') == 138.0 || block.val('P') == 139.0 || block.val('P') == 140.0 || block.val('P') == 141.0 || block.val('P') == 142.0 || block.val('P') == 143.0 || block.val('P') == 144.0 || block.val('P') == 145.0 || block.val('P') == 146.0 || block.val('P') == 147.0 || block.val('P') == 148.0 || block.val('P') == 149.0 || block.val('P') == 150.0 || block.val('P') == 151.0 || block.val('P') == 152.0 || block.val('P') == 153.0 || block.val('P') == 154.0 || block.val('P') == 155.0 || block.val('P') == 156.0 || block.val('P') == 157.0 || block.val('P') == 162.0 || block.val('P') == 163.0 || block.val('P') == 164.0 || block.val('P') == 165.0 || block.val('P') == 166.0 || block.val('P') == 167.0 || block.val('P') == 168.0 || block.val('P') == 169.0 || block.val('P') == 170.0 || block.val('P') == 171.0 || block.val('P') == 172.0 || block.val('P') == 173.0 || block.val('P') == 174.0 || block.val('P') == 175.0 || block.val('P') == 176.0 || block.val('P') == 177.0 || block.val('P') == 178.0 || block.val('P') == 179.0 || block.val('P') == 180.0 || block.val('P') == 181.0 || block.val('P') == 182.0 || block.val('P') == 183.0 || block.val('P') == 184.0 || block.val('P') == 185.0 || block.val('P') == 186.0 || block.val('P') == 187.0 || block.val('P') == 188.0 || block.val('P') == 189.0 || block.val('P') == 190.0 || block.val('P') == 191.0 || block.val('P') == 192.0 || block.val('P') == 193.0 || block.val('P') == 194.0 || block.val('P') == 195.0 || block.val('P') == 196.0 || block.val('P') == 197.0 || block.val('P') == 198.0 || block.val('P') == 199.0 || block.val('P') == 200.0 || block.val('P') == 201.0 || block.val('P') == 202.0 || block.val('P') == 203.0 || block.val('P') == 204.0 || block.val('P') == 205.0 || block.val('P') == 206.0 || block.val('P') == 207.0 || block.val('P') == 208.0 || block.val('P') == 209.0 || block.val('P') == 210.0 || block.val('P') == 211.0 || block.val('P') == 212.0 || block.val('P') == 213.0 || block.val('P') == 214.0 || block.val('P') == 215.0 || block.val('P') == 216.0 || block.val('P') == 217.0 || block.val('P') == 218.0 || block.val('P') == 219.0 || block.val('P') == 220.0 || block.val('P') == 221.0 || block.val('P') == 222.0 || block.val('P') == 223.0 || block.val('P') == 224.0 || block.val('P') == 225.0 || block.val('P') == 226.0 || block.val('P') == 227.0 || block.val('P') == 228.0 || block.val('P') == 229.0 || block.val('P') == 230.0 || block.val('P') == 231.0 || block.val('P') == 232.0 || block.val('P') == 233.0 || block.val('P') == 234.0 || block.val('P') == 236.0 || block.val('P') == 238.0 || block.val('P') == 240.0 || block.val('P') == 242.0 || block.val('P') == 244.0 || block.val('P') == 246.0 || block.val('P') == 248.0 || block.val('P') == 250.0 || block.val('P') == 252.0 || block.val('P') == 235.0 || block.val('P') == 237.0 || block.val('P') == 239.0 || block.val('P') == 241.0 || block.val('P') == 243.0 || block.val('P') == 245.0 || block.val('P') == 247.0 || block.val('P') == 249.0 || block.val('P') == 251.0 || block.val('P') == 253.0 || block.val('P') == 254.0 || block.val('P') == 255.0 || block.val('P') == 256.0 || block.val('P') == 257.0 || block.val('P') == 258.0 || block.val('P') == 259.0 || block.val('P') == 260.0 || block.val('P') == 261.0 || block.val('P') == 262.0 || block.val('P') == 263.0 || block.val('P') == 264.0 || block.val('P') == 265.0 || block.val('P') == 266.0 || block.val('P') == 267.0 || block.val('P') == 268.0 || block.val('P') == 269.0 || block.val('P') == 270.0 || block.val('P') == 271.0 || block.val('P') == 272.0 || block.val('P') == 273.0 || block.val('P') == 274.0 || block.val('P') == 275.0 || block.val('P') == 276.0 || block.val('P') == 277.0 || block.val('P') == 278.0 || block.val('P') == 279.0 || block.val('P') == 280.0 || block.val('P') == 281.0 || block.val('P') == 282.0 || block.val('P') == 283.0 || block.val('P') == 284.0 || (block.val('P') == 286.0 || block.val('P') == 288.0 || block.val('P') == 290.0) || block.val('P') == 285.0 || (block.val('P') == 287.0 || block.val('P') == 289.0 || block.val('P') == 291.0)))
    {
        if (block.isEmpty || block.isGoto || block.isBlockSkip || !block.hasG || block.gCode != 180 ||
            block.gCount != 1 || block.gCodes[0] != 180 || block.mCount != 0) return false;
        const bool noiseArm = block.val('P') == 93.0;
        const bool burstArm = block.val('P') == 95.0;
        const bool continuousArm = block.val('P') == 97.0;
        const bool enduranceArm = block.val('P') == 99.0;
        const bool pairedArm = block.val('P') == 101.0;
        const bool holdPathArm = EDM149::IsFlushHoldTestArm(static_cast<unsigned>(block.val('P'))) ||
            EDM149::IsManualVoltageTestArm(static_cast<unsigned>(block.val('P')));
        if (block.has('H') && (!holdPathArm || block.val('H') != 1.0)) return false;
        if (block.has('L') && (!holdPathArm || !block.has('H') || block.val('H') != 1.0 ||
            (block.val('L') != 1.0 && block.val('L') != 2.0 && block.val('L') != 3.0) ||
            !EDM130::ValidFlushHoldSelection(static_cast<unsigned>(block.val('P')), static_cast<unsigned>(block.val('L'))))) return false;
        const bool parameterizedArm = block.val('P') == 109.0 || block.val('P') == 111.0 || block.val('P') == 113.0 || block.val('P') == 115.0 || block.val('P') == 117.0 || block.val('P') == 119.0 || block.val('P') == 121.0 || block.val('P') == 123.0 || block.val('P') == 125.0 || block.val('P') == 127.0 || block.val('P') == 129.0 || block.val('P') == 132.0 || block.val('P') == 134.0 || block.val('P') == 136.0 || block.val('P') == 138.0 || block.val('P') == 140.0 || block.val('P') == 142.0 || block.val('P') == 144.0 || block.val('P') == 146.0 || block.val('P') == 148.0 || block.val('P') == 150.0 || block.val('P') == 152.0 || block.val('P') == 154.0 || block.val('P') == 156.0 || block.val('P') == 162.0 || block.val('P') == 164.0 || block.val('P') == 166.0 || block.val('P') == 168.0 || block.val('P') == 170.0 || block.val('P') == 172.0 || block.val('P') == 174.0 || block.val('P') == 176.0 || block.val('P') == 178.0 || block.val('P') == 180.0 || block.val('P') == 182.0 || block.val('P') == 184.0 || block.val('P') == 186.0 || block.val('P') == 188.0 || block.val('P') == 190.0 || block.val('P') == 192.0 || block.val('P') == 194.0 || block.val('P') == 196.0 || block.val('P') == 198.0 || block.val('P') == 200.0 || block.val('P') == 202.0 || block.val('P') == 204.0 || block.val('P') == 206.0 || block.val('P') == 208.0 || block.val('P') == 210.0 || block.val('P') == 212.0 || block.val('P') == 214.0 || block.val('P') == 216.0 || block.val('P') == 218.0 || block.val('P') == 220.0 || block.val('P') == 222.0 || block.val('P') == 224.0 || block.val('P') == 226.0 || block.val('P') == 228.0 || block.val('P') == 230.0 || block.val('P') == 232.0 || block.val('P') == 234.0 || block.val('P') == 236.0 || block.val('P') == 238.0 || block.val('P') == 240.0 || block.val('P') == 242.0 || block.val('P') == 244.0 || block.val('P') == 246.0 || block.val('P') == 248.0 || block.val('P') == 250.0 || block.val('P') == 252.0 || block.val('P') == 254.0 || block.val('P') == 256.0 || block.val('P') == 258.0 || block.val('P') == 260.0 || block.val('P') == 262.0 || block.val('P') == 264.0 || block.val('P') == 266.0 || block.val('P') == 268.0 || block.val('P') == 270.0 || block.val('P') == 272.0 || block.val('P') == 274.0 || block.val('P') == 276.0 || block.val('P') == 278.0 || block.val('P') == 280.0 || block.val('P') == 282.0 || block.val('P') == 284.0 || (block.val('P') == 286.0 || block.val('P') == 288.0 || block.val('P') == 290.0);
        const bool flushArm = block.val('P') == 103.0 || block.val('P') == 105.0 || block.val('P') == 107.0 || parameterizedArm;
        const bool tuningArm = block.val('P') == 91.0 || noiseArm || burstArm || continuousArm || enduranceArm || pairedArm || flushArm;
        for (char a = 'A'; a <= 'Z'; ++a)
            if (block.has(a) && ((a != 'N' && a != 'P' && ((a != 'H' && a != 'L') || !holdPathArm) &&
                (!tuningArm || (a != 'Q' && a != 'R' && a != 'S' &&
                    (!parameterizedArm || (a != 'I' && a != 'J' && a != 'K'))))) ||
                !std::isfinite(block.val(a)))) return false;
        if (tuningArm)
        {
            if (!block.has('Q') || !block.has('R') || !block.has('S')) return false;
            const EDM97::Tuning tuning{ block.val('Q'), block.val('R'), block.val('S') };
            if (!(noiseArm ? EDM98::ValidTuning(tuning) : EDM97::ValidTuning(tuning))) return false;
            if (parameterizedArm)
            {
                EDM107::FlushProfile profile{};
                if (!block.has('I') || !block.has('J') || !block.has('K') ||
                    !EDM107::ParseFlushProfile(block.val('I'), block.val('J'), block.val('K'), profile) ||
                    (E131SpeedArm(static_cast<unsigned>(block.val('P'))) && block.val('J') != 50.0) ||
                    (E132HeightArm(static_cast<unsigned>(block.val('P'))) && (block.val('I') != 10.0 || block.val('K') != 100.0))) return false;
            }
        }
        return true;
    }
    // EDM82 FIX1: record the actual decoded block BEFORE admission. This is
    // NC-thread diagnostics only; accepted profiles and all rejection rules
    // below remain unchanged. RtPrintf cannot format floating-point values.
    const auto reportShape = [&block](bool accepted, const char* reason, char badAddress) noexcept
    {
        const bool edm82Profile = block.has('P') &&
            (block.val('P') == 65.0 || block.val('P') == 66.0);
        if (accepted && !edm82Profile) return accepted;
        unsigned mask = 0U;
        for (unsigned i = 0U; i < 26U; ++i)
            if (block.hasParam[i]) mask |= 1U << i;
        char text[480]{};
        const int length = std::snprintf(text, sizeof(text),
            "[EDM82_FIX1] event=SHAPE build=EDM82_FIX1_20261011 accepted=%u reason=%s empty=%u goto=%u skip=%u hasG=%u G=%d gCount=%d G0=%d mCount=%d mask=%08X hasP=%u P=%.17g Pfinite=%u bad=%c badValue=%.17g output=NONE\n",
            accepted ? 1U : 0U, reason,
            block.isEmpty ? 1U : 0U, block.isGoto ? 1U : 0U,
            block.isBlockSkip ? 1U : 0U, block.hasG ? 1U : 0U,
            block.gCode, block.gCount, block.gCodes[0], block.mCount, mask,
            block.has('P') ? 1U : 0U, block.val('P'),
            std::isfinite(block.val('P')) ? 1U : 0U, badAddress,
            badAddress >= 'A' && badAddress <= 'Z' ? block.val(badAddress) : 0.0);
        if (length > 0 && static_cast<std::size_t>(length) < sizeof(text)) RtPrintf("%s", text);
        else RtPrintf("[EDM82_FIX1] event=SHAPE build=EDM82_FIX1_20261011 accepted=%u reason=LOG_FORMAT output=NONE\n",
            accepted ? 1U : 0U);
        return accepted;
    };
    if (block.isEmpty || block.isGoto || !block.hasG || block.gCode != 180 ||
        block.gCount != 1 || block.gCodes[0] != 180 || block.mCount != 0 ||
        !block.has('P') || (block.val('P') != 1.0 && block.val('P') != 2.0 && block.val('P') != 3.0 && block.val('P') != 4.0 && block.val('P') != 5.0 && block.val('P') != 6.0 && block.val('P') != 7.0 && block.val('P') != 8.0 && block.val('P') != 9.0 && block.val('P') != 10.0 && block.val('P') != 11.0 && block.val('P') != 12.0 && block.val('P') != 13.0 && block.val('P') != 14.0 && block.val('P') != 15.0 && block.val('P') != 16.0 && block.val('P') != 17.0 && block.val('P') != 18.0 && block.val('P') != 19.0 && block.val('P') != 20.0 && block.val('P') != 21.0 && block.val('P') != 22.0 && block.val('P') != 23.0 && block.val('P') != 24.0 && block.val('P') != 25.0 && block.val('P') != 26.0 && block.val('P') != 27.0 && block.val('P') != 28.0 && block.val('P') != 29.0 && block.val('P') != 30.0 && block.val('P') != 31.0 && block.val('P') != 32.0 && block.val('P') != 33.0 && block.val('P') != 34.0 && block.val('P') != 35.0 && block.val('P') != 36.0 && block.val('P') != 37.0 && block.val('P') != 38.0 && block.val('P') != 39.0 && block.val('P') != 40.0 && block.val('P') != 41.0 && block.val('P') != 42.0 && block.val('P') != 43.0 && block.val('P') != 44.0 && block.val('P') != 45.0 && block.val('P') != 46.0 && block.val('P') != 47.0 && block.val('P') != 48.0 && block.val('P') != 49.0 && block.val('P') != 50.0 && block.val('P') != 51.0 && block.val('P') != 52.0 && block.val('P') != 53.0 && block.val('P') != 54.0 && block.val('P') != 55.0 && block.val('P') != 56.0 && block.val('P') != 57.0 && block.val('P') != 58.0 && block.val('P') != 59.0 && block.val('P') != 60.0 && block.val('P') != 61.0 && block.val('P') != 62.0 && block.val('P') != 63.0 && block.val('P') != 64.0 && block.val('P') != 65.0 && block.val('P') != 66.0 && block.val('P') != 67.0 && block.val('P') != 68.0 && block.val('P') != 69.0 && block.val('P') != 70.0 && block.val('P') != 71.0 && block.val('P') != 72.0 && block.val('P') != 73.0 && block.val('P') != 74.0 && block.val('P') != 75.0 && block.val('P') != 76.0 && block.val('P') != 77.0 && block.val('P') != 78.0 && block.val('P') != 79.0 && block.val('P') != 80.0 && block.val('P') != 81.0 && block.val('P') != 82.0 && block.val('P') != 83.0 && block.val('P') != 84.0)) return reportShape(false, "HEADER_OR_PROFILE", '-');
    for (int i = 0; i < 26; ++i)
    {
        if (!block.hasParam[i]) continue;
        const bool operatorWindow = i == ('Q' - 'A') && (block.val('P') == 17.0 || block.val('P') == 22.0) &&
            (block.param[i] == 2.0 || block.param[i] == 3.0);
        const bool recipeSequence = i == ('Q' - 'A') &&
            (block.val('P') == 61.0 || block.val('P') == 62.0) &&
            (block.param[i] == 1.0 || block.param[i] == 2.0);
        const bool coordinatedScenario = i == ('Q' - 'A') &&
            (block.val('P') == 69.0 || block.val('P') == 70.0 || block.val('P') == 71.0 || block.val('P') == 72.0 || block.val('P') == 73.0 || block.val('P') == 74.0 || block.val('P') == 75.0 || block.val('P') == 76.0 || block.val('P') == 77.0 || block.val('P') == 78.0 || block.val('P') == 79.0 || block.val('P') == 80.0 || block.val('P') == 81.0 || block.val('P') == 82.0 || block.val('P') == 83.0 || block.val('P') == 84.0) &&
            (block.param[i] == 1.0 || block.param[i] == 2.0 || block.param[i] == 3.0 ||
                ((block.val('P') == 71.0 || block.val('P') == 72.0 || block.val('P') == 73.0 || block.val('P') == 74.0 || block.val('P') == 75.0 || block.val('P') == 76.0 || block.val('P') == 77.0 || block.val('P') == 78.0 || block.val('P') == 79.0 || block.val('P') == 80.0 || block.val('P') == 81.0 || block.val('P') == 82.0 || block.val('P') == 83.0 || block.val('P') == 84.0) && block.param[i] == 4.0));
        const bool toolHeadProfile = i == ('R' - 'A') &&
            (block.val('P') == 71.0 || block.val('P') == 72.0 || block.val('P') == 73.0 || block.val('P') == 74.0 || block.val('P') == 75.0 || block.val('P') == 76.0 || block.val('P') == 77.0 || block.val('P') == 78.0 || block.val('P') == 79.0 || block.val('P') == 80.0 || block.val('P') == 81.0 || block.val('P') == 82.0 || block.val('P') == 83.0 || block.val('P') == 84.0) &&
            block.has('Q') && block.val('Q') == 4.0 && std::isfinite(block.param[i]) &&
            block.param[i] >= 1.0 && block.param[i] <= static_cast<double>(EDM87::MaximumProfiles) &&
            std::floor(block.param[i]) == block.param[i];
        // S is a local sampled-source diagnostic selector, never a spindle command.
        const bool sampledSource = i == ('S' - 'A') &&
            (block.val('P') == 73.0 || block.val('P') == 74.0 || block.val('P') == 75.0 || block.val('P') == 76.0 || block.val('P') == 77.0 || block.val('P') == 78.0 || block.val('P') == 79.0 || block.val('P') == 80.0 || block.val('P') == 81.0 || block.val('P') == 82.0 || block.val('P') == 83.0 || block.val('P') == 84.0) &&
            (block.param[i] == 0.0 || block.param[i] == 1.0 || block.param[i] == 2.0 || block.param[i] == 3.0);
        if ((i != ('N' - 'A') && i != ('P' - 'A') && !operatorWindow && !recipeSequence && !coordinatedScenario && !toolHeadProfile && !sampledSource) ||
            !std::isfinite(block.param[i])) return reportShape(false, std::isfinite(block.param[i]) ? "ADDRESS_OR_Q_VALUE" : "NONFINITE_ADDRESS", static_cast<char>('A' + i));
    }
    return reportShape(true, "VALID", '-');
}

bool NCManager::IsEDMShortFlushRunReadySameThread() const noexcept
{
    // This is readiness for an isolated diagnostic, never motion permission.
    return m_mode == NCOperationMode::MEMORY && m_state == NCState::RUN &&
        m_edmState != EDMState::NOT_READY && m_edmState != EDMState::HOLD &&
        m_edmState != EDMState::ALARM &&
        m_externalReadyInterlock && m_resetContinuationPhase == ResetContinuationPhase::IDLE &&
        !m_resetSafetyOutputHoldActive && !Close_System_Com_flag &&
        !AlarmManager::GetInstance().HasAlarm() && !Homing.IsActive();
}

void NCManager::InvalidateEDMPhysicalSequencePlanSameThread() noexcept
{
    auto& plan = m_edmPhysicalSequencePlan;
    plan.plan.Invalidate();
    plan.run = plan.cache = 0ULL;
    plan.epoch = MOTION_EXECUTION_EPOCH_INVALID;
    plan.lease = MotionOwnerLease{};
    plan.firstPC = -1; plan.next = 0U;
    plan.dispatch = NC_BLOCK_DISPATCH_ID_INVALID; plan.legPending = false;
    plan.terminalPC = -1; plan.terminalDispatch = NC_BLOCK_DISPATCH_ID_INVALID;
}

// Only for a new dispatch rejected before it creates a fixture session. Real
// active/retiring work still takes the existing containment path, including the
// early ExecuteBlock shape check that runs before Start's busy guards.
void NCManager::RejectEDMGapAdmissionSameThread(int alarmCode, int line, const char* reason)
{
    if (m_gapDryRun.active || m_edmZFixture.cleanupPending || m_edmZFixture.resetPending ||
        m_edmZFixture.failureAlarmPending || m_edmPhysicalServoRT.retirePending ||
        m_edmCoordinatedRT.retirePending || (m_edmCoordinatedRT.submitted && !m_edmCoordinatedRT.retired) ||
        m_edmGapServoRT.retirePending || m_edmCNCRetainedRT.retirePending ||
        (m_edmCNCRetainedRT.submitted && !m_edmCNCRetainedRT.retired))
    { RejectGapDryRunSameThread(alarmCode, line, reason); return; }
    InvalidateEDMPhysicalSequencePlanSameThread();
    RtPrintf("[EDM79] event=ADMISSION_REJECT result=REJECT reason=%s run=%llu cache=%llu dispatch=%llu line=%d newSession=0 armIssued=0 discharge=0\n",
        reason ? reason : "UNKNOWN", static_cast<unsigned long long>(m_pathCoreLiveBookkeeping.currentRunToken),
        static_cast<unsigned long long>(GetBaseProgramCache().GetGeneration()),
        static_cast<unsigned long long>(m_currentExecutingBlockDispatchId), line);
    AlarmManager::GetInstance().Trigger(alarmCode, line);
    ChangeState(NCState::ALARM);
}

void NCManager::RejectGapDryRunSameThread(int alarmCode, int line, const char* reason)
{
    if (m_gapDryRun.coordinatedRTTest || m_edmCoordinatedRT.retirePending ||
        (m_edmCoordinatedRT.submitted && !m_edmCoordinatedRT.retired))
    {
        EndEDMCoordinatedRTSameThread(true, reason);
        AlarmManager::GetInstance().Trigger(alarmCode, line);
        ChangeState(NCState::ALARM);
        return;
    }
    if (m_gapDryRun.cncRetainedRTTest || m_edmCNCRetainedRT.retirePending ||
        (m_edmCNCRetainedRT.submitted && !m_edmCNCRetainedRT.retired))
    {
        EndEDMCNCRetainedRTSameThread(true, reason);
        AlarmManager::GetInstance().Trigger(alarmCode, line);
        ChangeState(NCState::ALARM);
        return;
    }
    if (m_gapDryRun.cncRetainedSimulationTest)
    {
        EndEDMCNCRetainedSimulationSameThread(true, reason);
        AlarmManager::GetInstance().Trigger(alarmCode, line);
        ChangeState(NCState::ALARM);
        return;
    }
    if (m_gapDryRun.multiAxisSimulationTest)
    {
        EndEDMMultiAxisSimulationSameThread(true, reason);
        AlarmManager::GetInstance().Trigger(alarmCode, line); ChangeState(NCState::ALARM); return;
    }
    if (m_gapDryRun.physicalServoRTTest)
    {
        EndEDMPhysicalServoRTSameThread(true, reason);
        AlarmManager::GetInstance().Trigger(alarmCode, line); ChangeState(NCState::ALARM); return;
    }
    if (m_gapDryRun.physicalRecipeCaptureTest)
    {
        EndEDMPhysicalRecipeCaptureSameThread(true, reason);
        AlarmManager::GetInstance().Trigger(alarmCode, line); ChangeState(NCState::ALARM); return;
    }
    if (m_gapDryRun.runtimeServoTest)
    {
        EndEDMGapServoRTSameThread(true, reason);
        AlarmManager::GetInstance().Trigger(alarmCode, line);
        ChangeState(NCState::ALARM);
        return;
    }

    if (m_gapDryRun.physicalZFixtureTest)
    {
        EndEDMZFixtureSameThread("FAIL", reason);
        AlarmManager::GetInstance().Trigger(alarmCode, line); ChangeState(NCState::ALARM); return;
    }
    if (m_gapDryRun.motionBridgeTest)
    {
        EndEDMMotionReplaySameThread("FAIL", reason, m_gapDryRun.lastServiceMs);
        AlarmManager::GetInstance().Trigger(alarmCode, line); ChangeState(NCState::ALARM); return;
    }
    if (m_gapDryRun.liveProcessTest)
    {
        EndEDMLiveProcessSameThread("FAIL", reason, m_gapDryRun.lastServiceMs);
        AlarmManager::GetInstance().Trigger(alarmCode, line);
        ChangeState(NCState::ALARM);
        return;
    }
    if (m_gapDryRun.liveAutomaticFlushTest)
    {
        EndEDMLiveAutomaticFlushSameThread("FAIL", reason, m_gapDryRun.lastServiceMs);
        AlarmManager::GetInstance().Trigger(alarmCode, line);
        ChangeState(NCState::ALARM);
        return;
    }
    m_gapDryRun.active = false;
    m_gapDryRun.paused = false;
    m_gapDryRun.result = 3U;
    m_edmProcessTest.Revoke();
    m_edmFeedRetreatTest.Revoke();
    m_edmAcquisitionTest.Revoke();
    m_edmVoltageTest.Revoke();
    m_edmRecipeTest.Revoke();
    m_edmShortFlushTest.Revoke();
    m_edmAutomaticFlushTest.Revoke();
    m_edmProcessCoordinatorTest.Revoke();
    m_edmMotionReplay.Revoke();
    m_edmMotionReplayReceiptPending = false;
    RevokeEDMLiveProcessSameThread();
    m_edmLiveAutomaticFlush.Revoke();
    if (m_gapDryRun.shortFlushTest || m_gapDryRun.automaticFlushTest || m_gapDryRun.liveAutomaticFlushTest || m_gapDryRun.processCoordinatorTest || m_gapDryRun.liveProcessTest || m_gapDryRun.motionBridgeTest)
    {
        char text[512]{};
        const int length = std::snprintf(text, sizeof(text),
            "[%s] event=SUMMARY result=FAIL test=%llu run=%llu dispatch=%llu line=%d case=%u passedMask=%08X reason=%.80s alarm=%d mode=SHADOW_ONLY motion=0 physicalPermit=0 discharge=0\n",
            m_gapDryRun.motionBridgeTest ? "EDM27" : m_gapDryRun.liveProcessTest ? "EDM26" : m_gapDryRun.processCoordinatorTest ? "EDM25" : (m_gapDryRun.liveAutomaticFlushTest ? "EDM24" : (m_gapDryRun.automaticFlushTest ? "EDM23" : "EDM22")),
            static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(m_gapDryRun.run),
            static_cast<unsigned long long>(m_gapDryRun.dispatch), line,
            static_cast<unsigned int>(m_gapDryRun.phase), static_cast<unsigned int>(m_gapDryRun.passedMask),
            reason ? reason : "UNKNOWN", alarmCode);
        if (length >= 0 && static_cast<std::size_t>(length) < sizeof(text)) RtPrintf("%s", text);
        else RtPrintf("%s", m_gapDryRun.motionBridgeTest ? "[EDM27] event=SUMMARY result=FAIL reason=LOG_FORMAT motion=0 physicalPermit=0 discharge=0\n" : m_gapDryRun.liveProcessTest ? "[EDM26] event=SUMMARY result=FAIL reason=LOG_FORMAT motion=0 physicalPermit=0 discharge=0\n" : m_gapDryRun.processCoordinatorTest ? "[EDM25] event=SUMMARY result=FAIL reason=LOG_FORMAT motion=0 physicalPermit=0 discharge=0\n" : m_gapDryRun.automaticFlushTest ? "[EDM23] event=SUMMARY result=FAIL reason=LOG_FORMAT physicalPermit=0 discharge=0\n" : "[EDM22] event=SUMMARY result=FAIL reason=LOG_FORMAT physicalPermit=0 discharge=0\n");
    }
    else if (m_gapDryRun.adcShadowTest || m_gapDryRun.voltageTest || m_gapDryRun.recipeTest)
    {
        RtPrintf("[%s] event=SUMMARY result=FAIL test=%llu run=%llu dispatch=%llu line=%d case=%u passedMask=%08X reason=%s alarm=%d motion=0 physicalPermit=0 discharge=0\n",
            m_gapDryRun.recipeTest ? "EDM16" : (m_gapDryRun.voltageTest ? "EDM15" : "EDM14"),
            static_cast<unsigned long long>(m_gapDryRun.test),
            static_cast<unsigned long long>(m_gapDryRun.run),
            static_cast<unsigned long long>(m_gapDryRun.dispatch), line,
            static_cast<unsigned int>(m_gapDryRun.phase),
            static_cast<unsigned int>(m_gapDryRun.passedMask), reason, alarmCode);
    }
    else
    RtPrintf("[%s] event=SUMMARY result=FAIL test=%llu run=%llu dispatch=%llu line=%d case=%u passedMask=%04X reason=%s alarm=%d\n",
        GapDryRunTag(m_gapDryRun.processTest, m_gapDryRun.feedRetreatTest),
        static_cast<unsigned long long>(m_gapDryRun.test),
        static_cast<unsigned long long>(m_gapDryRun.run),
        static_cast<unsigned long long>(m_gapDryRun.dispatch), line,
        static_cast<unsigned int>(m_gapDryRun.phase),
        static_cast<unsigned int>(m_gapDryRun.passedMask), reason, alarmCode);
    if (!m_gapDryRun.shortFlushTest && !m_gapDryRun.automaticFlushTest && !m_gapDryRun.liveAutomaticFlushTest && !m_gapDryRun.processCoordinatorTest && !m_gapDryRun.liveProcessTest && !m_gapDryRun.motionBridgeTest) m_gapInput.Reset();
    AlarmManager::GetInstance().Trigger(alarmCode, line);
    ChangeState(NCState::ALARM);
}

bool NCManager::ReadGapDryRunClockSameThread(std::uint64_t& nowMs) noexcept
{
    LARGE_INTEGER counter{};
    const char* fault = nullptr;
    if (m_gapDryRun.frequency == 0ULL) fault = "NO_FREQUENCY";
    else if (!RtQueryPerformanceCounter(&counter)) fault = "COUNTER_READ";
    else if (counter.QuadPart < 0) fault = "NEGATIVE_COUNTER";
    else if (static_cast<std::uint64_t>(counter.QuadPart) < m_gapDryRun.lastTicks)
        fault = "COUNTER_REGRESSION";
    if (fault != nullptr)
    {
        if (!m_gapDryRun.adcShadowTest && !m_gapDryRun.voltageTest && !m_gapDryRun.recipeTest && !m_gapDryRun.shortFlushTest && !m_gapDryRun.automaticFlushTest && !m_gapDryRun.liveAutomaticFlushTest && !m_gapDryRun.processCoordinatorTest && !m_gapDryRun.liveProcessTest && !m_gapDryRun.motionBridgeTest && !m_gapDryRun.physicalZFixtureTest)
        RtPrintf("[%s] event=CLOCK_FAULT build=%s reason=%s rawTicks=%lld lastTicks=%llu frequency=%llu test=%llu run=%llu dispatch=%llu\n",
            GapDryRunTag(m_gapDryRun.processTest, m_gapDryRun.feedRetreatTest),
            m_gapDryRun.feedRetreatTest ? "EDM02" : "EDM01_FIX1", fault,
            static_cast<long long>(counter.QuadPart),
            static_cast<unsigned long long>(m_gapDryRun.lastTicks),
            static_cast<unsigned long long>(m_gapDryRun.frequency),
            static_cast<unsigned long long>(m_gapDryRun.test),
            static_cast<unsigned long long>(m_gapDryRun.run),
            static_cast<unsigned long long>(m_gapDryRun.dispatch));
        return false;
    }
    const std::uint64_t ticks = static_cast<std::uint64_t>(counter.QuadPart);
    const std::uint64_t seconds = ticks / m_gapDryRun.frequency;
    const std::uint64_t fraction = (ticks % m_gapDryRun.frequency) * 1000ULL /
        m_gapDryRun.frequency;
    if (seconds > ((std::numeric_limits<std::uint64_t>::max)() - fraction) / 1000ULL)
    {
        if (!m_gapDryRun.adcShadowTest && !m_gapDryRun.voltageTest && !m_gapDryRun.recipeTest && !m_gapDryRun.shortFlushTest && !m_gapDryRun.automaticFlushTest && !m_gapDryRun.liveAutomaticFlushTest && !m_gapDryRun.processCoordinatorTest && !m_gapDryRun.liveProcessTest && !m_gapDryRun.motionBridgeTest && !m_gapDryRun.physicalZFixtureTest)
        RtPrintf("[%s] event=CLOCK_FAULT build=%s reason=MILLISECOND_OVERFLOW test=%llu\n",
            GapDryRunTag(m_gapDryRun.processTest, m_gapDryRun.feedRetreatTest),
            m_gapDryRun.feedRetreatTest ? "EDM02" : "EDM01_FIX1",
            static_cast<unsigned long long>(m_gapDryRun.test));
        return false;
    }
    nowMs = seconds * 1000ULL + fraction;
    m_gapDryRun.lastTicks = ticks;
    return true;
}

bool NCManager::RestartGapDryRunSameThread()
{
    if (m_gapDryRun.physicalServoRTTest)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "EDM69_EXPLICIT_RESTART_REQUIRED"); return false;
    }
    if (m_gapDryRun.physicalRecipeCaptureTest)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, m_gapDryRun.physicalRecipeWindowTest ? "EDM68_EXPLICIT_RECAPTURE_REQUIRED" : "EDM67_EXPLICIT_RECAPTURE_REQUIRED"); return false;
    }
    if ((m_gapDryRun.shortFlushTest || m_gapDryRun.automaticFlushTest || m_gapDryRun.liveAutomaticFlushTest || m_gapDryRun.processCoordinatorTest || m_gapDryRun.liveProcessTest || m_gapDryRun.motionBridgeTest) && !IsEDMShortFlushRunReadySameThread())
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, m_gapDryRun.motionBridgeTest ? "EDM27_NOT_READY" : m_gapDryRun.liveProcessTest ? "EDM26_NOT_READY" : m_gapDryRun.processCoordinatorTest ? "EDM25_NOT_READY" : (m_gapDryRun.liveAutomaticFlushTest ? "EDM24_NOT_READY" : (m_gapDryRun.automaticFlushTest ? "EDM23_NOT_READY" : "EDM22_NOT_READY")));
        return false;
    }
    if (m_gapDryRun.recipeTest && !m_edmRecipeTest.Ready())
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "EDM16_CATALOG_OR_FIXTURE");
        return false;
    }
    if (m_gapDryRun.voltageTest && !m_edmVoltageTest.Ready())
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "EDM15_PROFILE_UNAVAILABLE");
        return false;
    }
    LARGE_INTEGER frequency{};
    if (!RtQueryPerformanceFrequency(&frequency) || frequency.QuadPart <= 0 ||
        static_cast<std::uint64_t>(frequency.QuadPart) >
        (std::numeric_limits<std::uint64_t>::max)() / 1000ULL)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "CLOCK_FREQUENCY");
        return false;
    }
    m_gapDryRun.frequency = static_cast<std::uint64_t>(frequency.QuadPart);
    m_gapDryRun.lastTicks = 0ULL;
    std::uint64_t nowMs = 0ULL;
    if (!ReadGapDryRunClockSameThread(nowMs))
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "CLOCK_READ");
        return false;
    }
    // These thresholds belong only to this synthetic test profile.
    if (!m_gapDryRun.shortFlushTest && !m_gapDryRun.automaticFlushTest && !m_gapDryRun.liveAutomaticFlushTest && !m_gapDryRun.processCoordinatorTest && !m_gapDryRun.liveProcessTest && !m_gapDryRun.motionBridgeTest &&
        !m_gapInput.Configure(EDMGap::Config{}, EDMGap::Source::SIMULATED))
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "SIM_CONFIG");
        return false;
    }
    m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch();
    m_gapDryRun.lease = m_programMotionLease;
    m_gapDryRun.phase = 0U;
    m_gapDryRun.passedMask = 0U;
    m_gapDryRun.observations = 0U;
    m_gapDryRun.phaseSamples = 0U;
    m_gapDryRun.sequence = 0ULL;
    m_gapDryRun.sample = EDMGap::Sample{};
    m_gapDryRun.phaseStartMs = nowMs;
    m_gapDryRun.lastServiceMs = nowMs;
    m_gapDryRun.paused = false;
    m_gapDryRun.result = 0U;
    m_edmProcessTest.Reset();
    m_edmFeedRetreatTest.Reset();
    m_edmShortFlushTest.Revoke();
    m_edmAutomaticFlushTest.Revoke();
    m_edmProcessCoordinatorTest.Revoke();
    m_edmMotionReplay.Revoke();
    m_edmMotionReplayReceiptPending = false;
    RevokeEDMLiveProcessSameThread();
    m_edmLiveAutomaticFlush.Revoke();
    if (m_gapDryRun.motionBridgeTest) return RestartEDMMotionReplaySameThread(nowMs);
    if (m_gapDryRun.liveProcessTest) return RestartEDMLiveProcessSameThread(nowMs);
    if (m_gapDryRun.liveAutomaticFlushTest) return RestartEDMLiveAutomaticFlushSameThread(nowMs);
    if (m_gapDryRun.processCoordinatorTest)
    {
        // Initial dispatch is latched by the caller after this BEGIN. Do not
        // compare against the previous waiting callback during admission.
        m_edmProcessCoordinatorTest.Reset();
        char line[512]{};
        const int length = std::snprintf(line, sizeof(line),
            "[EDM25] event=BEGIN test=%llu run=%llu dispatch=%llu line=%d restart=%u cases=%u caseMs=500 clock=VIRTUAL_CASES fixture=INDEPENDENT mode=SHADOW_ONLY motion=0 physicalPermit=0 discharge=0\n",
            static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(m_gapDryRun.run),
            static_cast<unsigned long long>(m_gapDryRun.dispatch), m_gapDryRun.sourceLine,
            static_cast<unsigned int>(m_gapDryRun.restarts), static_cast<unsigned int>(EDMProcessCoordinatorSimulationTest::CaseCount));
        if (length < 0 || static_cast<std::size_t>(length) >= sizeof(line))
        {
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM25_LOG_FORMAT");
            return false;
        }
        RtPrintf("%s", line);
        return true;
    }
    if (m_gapDryRun.automaticFlushTest)
    {
        m_edmAutomaticFlushTest.Reset();
        char line[512]{};
        int length = std::snprintf(line, sizeof(line),
            "[EDM23] event=BEGIN test=%llu run=%llu dispatch=%llu line=%d restart=%u cases=%u caseMs=500 clock=VIRTUAL_CASES fixture=INDEPENDENT mode=SHADOW_ONLY motion=0 physicalPermit=0 discharge=0\n",
            static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(m_gapDryRun.run),
            static_cast<unsigned long long>(m_gapDryRun.dispatch), m_gapDryRun.sourceLine,
            static_cast<unsigned int>(m_gapDryRun.restarts), static_cast<unsigned int>(EDMAutomaticFlushSimulationTest::CaseCount));
        if (length < 0 || static_cast<std::size_t>(length) >= sizeof(line))
        {
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM23_LOG_FORMAT");
            return false;
        }
        RtPrintf("%s", line);
        // Read-only metadata: the independent fixture values below do not use
        // or mutate this active COND. Unit/zero-work problems block only binding.
        const auto binding = EDM23::BindRecipe(m_recipe, m_edmProcessProfile.machineProfileId);
        length = std::snprintf(line, sizeof(line),
            "[EDM23] event=RECIPE test=%llu result=%s reason=%.64s field=%u cond=%u E=%u generation=%llu workMs=%llu heightMm=%.9g speedPct=%.9g deepEvery=%u deepMultiplier=%.9g fixture=INDEPENDENT automaticActive=0 mode=SHADOW_ONLY motion=0 physicalPermit=0 discharge=0\n",
            static_cast<unsigned long long>(m_gapDryRun.test), binding.ready ? "READY" : "BLOCKED",
            EDM23::RecipeBindingErrorName(binding.error), static_cast<unsigned int>(binding.failedFieldId),
            static_cast<unsigned int>(binding.tableId), static_cast<unsigned int>(binding.eCode),
            static_cast<unsigned long long>(binding.generation), static_cast<unsigned long long>(binding.workTimeMs),
            binding.jumpHeightMm, binding.speedOverridePercent, static_cast<unsigned int>(binding.deepFlushCycleInterval),
            binding.deepFlushHeightMultiplier);
        if (length < 0 || static_cast<std::size_t>(length) >= sizeof(line))
        {
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM23_LOG_FORMAT");
            return false;
        }
        RtPrintf("%s", line);
        // The virtual B0 frame confirms only a local model plan. Start/Revoke
        // exercises bound values plus the installed profile without one Step.
        EDM23::AutomaticFlushCycle currentRecipePlan{};
        EDM23::AutomaticFlushCycleConfig currentConfig{};
        EDM20::FlushRequest virtualRequest{};
        virtualRequest.mode = EDM20::FlushMode::B0;
        virtualRequest.jumpHeightConfirmed = true;
        virtualRequest.frame.machiningDirectionConfirmed = true;
        const bool mapped = m_edmProcessReady && EDM23::MakeAutomaticFlushCycleConfig(binding, m_edmProcessProfile.flush, m_edmProcessProfile.revision, virtualRequest, currentConfig);
        const bool planReady = mapped && currentRecipePlan.Start(currentConfig, nowMs);
        const auto planError = currentRecipePlan.Snapshot().error;
        currentRecipePlan.Revoke();
        length = std::snprintf(line, sizeof(line),
            "[EDM23] event=RECIPE_PLAN test=%llu result=%s reason=%.64s origin=VIRTUAL_B0 automaticActive=0 config=%llu recipeGeneration=%llu stepped=0 mode=SHADOW_ONLY motion=0 physicalPermit=0 discharge=0\n",
            static_cast<unsigned long long>(m_gapDryRun.test), planReady ? "READY" : "BLOCKED",
            mapped ? EDM23::AutomaticFlushCycleErrorName(planError) : (!m_edmProcessReady ? "CONFIG_UNAVAILABLE" : (binding.ready ? "PLAN_ADMISSION_BLOCKED" : EDM23::RecipeBindingErrorName(binding.error))),
            static_cast<unsigned long long>(m_edmProcessProfile.revision), static_cast<unsigned long long>(binding.generation));
        if (length < 0 || static_cast<std::size_t>(length) >= sizeof(line))
        {
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM23_LOG_FORMAT");
            return false;
        }
        RtPrintf("%s", line);
        return true;
    }
    if (m_gapDryRun.shortFlushTest)
    {
        m_edmShortFlushTest.Reset();
        char line[512]{};
        const int length = std::snprintf(line, sizeof(line),
            "[EDM22] event=BEGIN test=%llu run=%llu dispatch=%llu line=%d restart=%u cases=%u caseMs=500 clock=VIRTUAL_CASES mode=SHADOW_ONLY motion=0 physicalPermit=0 discharge=0\n",
            static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(m_gapDryRun.run),
            static_cast<unsigned long long>(m_gapDryRun.dispatch), m_gapDryRun.sourceLine,
            static_cast<unsigned int>(m_gapDryRun.restarts), static_cast<unsigned int>(EDMShortFlushSimulationTest::CaseCount));
        if (length < 0 || static_cast<std::size_t>(length) >= sizeof(line))
        {
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
                m_gapDryRun.sourceLine, "EDM22_LOG_FORMAT");
            return false;
        }
        RtPrintf("%s", line);
        return true;
    }
    if (m_gapDryRun.recipeTest)
    {
        ClearEDMRecipeSelectionSameThread();
        if (!m_edmRecipeTest.Restart())
        {
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
                m_gapDryRun.sourceLine, "EDM16_CATALOG_OR_FIXTURE");
            return false;
        }
        RtPrintf("[EDM16] event=BEGIN build=EDM16 test=%llu run=%llu dispatch=%llu restart=%u cases=%u caseMs=500 mode=SHADOW_ONLY motion=0 physicalPermit=0 discharge=0\n",
            static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(m_gapDryRun.run),
            static_cast<unsigned long long>(m_gapDryRun.dispatch), static_cast<unsigned int>(m_gapDryRun.restarts),
            static_cast<unsigned int>(EDMRecipeSimulationTest::CaseCount));
        return true;
    }
    if (m_gapDryRun.voltageTest)
    {
        if (!m_edmVoltageTest.Restart())
        {
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
                m_gapDryRun.sourceLine, "EDM15_PROFILE_UNAVAILABLE");
            return false;
        }
        RtPrintf("[EDM15] event=BEGIN build=EDM15 mode=SYNTHETIC_SHADOW test=%llu run=%llu dispatch=%llu line=%d restart=%u profile=%u cases=%u caseMs=500 clock=VIRTUAL_CASES syntheticOnly=1 motion=0 physicalPermit=0 discharge=0\n",
            static_cast<unsigned long long>(m_gapDryRun.test),
            static_cast<unsigned long long>(m_gapDryRun.run),
            static_cast<unsigned long long>(m_gapDryRun.dispatch), m_gapDryRun.sourceLine,
            static_cast<unsigned int>(m_gapDryRun.restarts),
            static_cast<unsigned int>(m_edmVoltageTest.Profile().profileRevision),
            static_cast<unsigned int>(EDMVoltageSimulationTest::CaseCount));
        return true;
    }
    if (m_gapDryRun.adcShadowTest)
    {
        m_edmAcquisitionTest.Restart();
        RtPrintf("[EDM14] event=BEGIN build=EDM14 mode=SYNTHETIC_ADC_SHADOW test=%llu run=%llu dispatch=%llu line=%d restart=%u cases=%u caseMs=500 clock=VIRTUAL_CASES motion=0 physicalPermit=0 discharge=0\n",
            static_cast<unsigned long long>(m_gapDryRun.test),
            static_cast<unsigned long long>(m_gapDryRun.run),
            static_cast<unsigned long long>(m_gapDryRun.dispatch), m_gapDryRun.sourceLine,
            static_cast<unsigned int>(m_gapDryRun.restarts),
            static_cast<unsigned int>(EDMGapAcquisitionSimulationTest::CaseCount));
        return true;
    }
    if (m_gapDryRun.feedRetreatTest)
    {
        RtPrintf("[EDM02] event=BEGIN build=EDM02 mode=SIMULATED test=%llu run=%llu dispatch=%llu line=%d restart=%u cases=%u caseMs=%llu clock=VIRTUAL_CONTINUOUS consumer=VIRTUAL motion=0 discharge=0\n",
            static_cast<unsigned long long>(m_gapDryRun.test),
            static_cast<unsigned long long>(m_gapDryRun.run),
            static_cast<unsigned long long>(m_gapDryRun.dispatch), m_gapDryRun.sourceLine,
            static_cast<unsigned int>(m_gapDryRun.restarts),
            static_cast<unsigned int>(EDMFeedRetreatSimulationTest::CaseCount),
            static_cast<unsigned long long>(EDMProcessCaseMs));
        return true;
    }
    if (m_gapDryRun.processTest)
    {
        RtPrintf("[EDM01] event=BEGIN build=EDM01_FIX1 mode=SIMULATED test=%llu run=%llu dispatch=%llu line=%d restart=%u cases=%u caseMs=%llu clock=VIRTUAL_CASES motion=0 discharge=0\n",
            static_cast<unsigned long long>(m_gapDryRun.test),
            static_cast<unsigned long long>(m_gapDryRun.run),
            static_cast<unsigned long long>(m_gapDryRun.dispatch), m_gapDryRun.sourceLine,
            static_cast<unsigned int>(m_gapDryRun.restarts),
            static_cast<unsigned int>(EDMProcessSimulationTest::CaseCount),
            static_cast<unsigned long long>(EDMProcessCaseMs));
        return true;
    }
    RtPrintf("[GAP-CG] event=BEGIN mode=SIMULATED test=%llu run=%llu dispatch=%llu line=%d restart=%u cases=%u caseMs=%llu dwellMs=30 staleMs=100 lowEnterMv=30000 lowExitMv=35000 highExitMv=75000 highEnterMv=80000 motion=0 discharge=0\n",
        static_cast<unsigned long long>(m_gapDryRun.test),
        static_cast<unsigned long long>(m_gapDryRun.run),
        static_cast<unsigned long long>(m_gapDryRun.dispatch), m_gapDryRun.sourceLine,
        static_cast<unsigned int>(m_gapDryRun.restarts),
        static_cast<unsigned int>(GapCaseCount), static_cast<unsigned long long>(GapCaseMs));
    return true;
}

WaitConditionFunc NCManager::StartGapDryRunSameThread(const NCBlock& block)
{
    if (block.has('P') && (block.val('P')==292.0 || block.val('P')==293.0 || block.val('P')==294.0 || block.val('P')==295.0 || block.val('P')==296.0 || block.val('P')==297.0 || block.val('P')==298.0 || block.val('P')==299.0 || block.val('P')==300.0 || block.val('P')==301.0)) return StartEDM173SameThread(block);
    if (block.has('P') && (block.val('P') == 85.0 || block.val('P') == 86.0 ||
        block.val('P') == 87.0 || block.val('P') == 88.0 ||
        block.val('P') == 89.0 || block.val('P') == 90.0 ||
        block.val('P') == 91.0 || block.val('P') == 92.0 ||
        block.val('P') == 93.0 || block.val('P') == 94.0 ||
        block.val('P') == 95.0 || block.val('P') == 96.0 ||
        block.val('P') == 97.0 || block.val('P') == 98.0 ||
        block.val('P') == 99.0 || block.val('P') == 100.0 ||
        block.val('P') == 101.0 || block.val('P') == 102.0 ||
        block.val('P') == 103.0 || block.val('P') == 104.0 ||
        block.val('P') == 105.0 || block.val('P') == 106.0 ||
        block.val('P') == 107.0 || block.val('P') == 108.0 ||
        block.val('P') == 109.0 || block.val('P') == 110.0 ||
        block.val('P') == 111.0 || block.val('P') == 112.0 ||
        block.val('P') == 113.0 || block.val('P') == 114.0 || block.val('P') == 115.0 || block.val('P') == 116.0 || block.val('P') == 117.0 || block.val('P') == 118.0 || block.val('P') == 119.0 || block.val('P') == 120.0 || block.val('P') == 121.0 || block.val('P') == 122.0 || block.val('P') == 123.0 || block.val('P') == 124.0 || block.val('P') == 125.0 || block.val('P') == 126.0 || block.val('P') == 127.0 || block.val('P') == 128.0 || block.val('P') == 129.0 || block.val('P') == 130.0 || block.val('P') == 131.0 || block.val('P') == 132.0 || block.val('P') == 133.0 || block.val('P') == 134.0 || block.val('P') == 135.0 || block.val('P') == 136.0 || block.val('P') == 137.0 || block.val('P') == 138.0 || block.val('P') == 139.0 || block.val('P') == 140.0 || block.val('P') == 141.0 || block.val('P') == 142.0 || block.val('P') == 143.0 || block.val('P') == 144.0 || block.val('P') == 145.0 || block.val('P') == 146.0 || block.val('P') == 147.0 || block.val('P') == 148.0 || block.val('P') == 149.0 || block.val('P') == 150.0 || block.val('P') == 151.0 || block.val('P') == 152.0 || block.val('P') == 153.0 || block.val('P') == 154.0 || block.val('P') == 155.0 || block.val('P') == 156.0 || block.val('P') == 157.0 || block.val('P') == 158.0 || block.val('P') == 159.0 || block.val('P') == 160.0 || block.val('P') == 161.0 || block.val('P') == 162.0 || block.val('P') == 163.0 || block.val('P') == 164.0 || block.val('P') == 165.0 || block.val('P') == 166.0 || block.val('P') == 167.0 || block.val('P') == 168.0 || block.val('P') == 169.0 || block.val('P') == 170.0 || block.val('P') == 171.0 || block.val('P') == 172.0 || block.val('P') == 173.0 || block.val('P') == 174.0 || block.val('P') == 175.0 || block.val('P') == 176.0 || block.val('P') == 177.0 || block.val('P') == 178.0 || block.val('P') == 179.0 || block.val('P') == 180.0 || block.val('P') == 181.0 || block.val('P') == 182.0 || block.val('P') == 183.0 || block.val('P') == 184.0 || block.val('P') == 185.0 || block.val('P') == 186.0 || block.val('P') == 187.0 || block.val('P') == 188.0 || block.val('P') == 189.0 || block.val('P') == 190.0 || block.val('P') == 191.0 || block.val('P') == 192.0 || block.val('P') == 193.0 || block.val('P') == 194.0 || block.val('P') == 195.0 || block.val('P') == 196.0 || block.val('P') == 197.0 || block.val('P') == 198.0 || block.val('P') == 199.0 || block.val('P') == 200.0 || block.val('P') == 201.0 || block.val('P') == 202.0 || block.val('P') == 203.0 || block.val('P') == 204.0 || block.val('P') == 205.0 || block.val('P') == 206.0 || block.val('P') == 207.0 || block.val('P') == 208.0 || block.val('P') == 209.0 || block.val('P') == 210.0 || block.val('P') == 211.0 || block.val('P') == 212.0 || block.val('P') == 213.0 || block.val('P') == 214.0 || block.val('P') == 215.0 || block.val('P') == 216.0 || block.val('P') == 217.0 || block.val('P') == 218.0 || block.val('P') == 219.0 || block.val('P') == 220.0 || block.val('P') == 221.0 || block.val('P') == 222.0 || block.val('P') == 223.0 || block.val('P') == 224.0 || block.val('P') == 225.0 || block.val('P') == 226.0 || block.val('P') == 227.0 || block.val('P') == 228.0 || block.val('P') == 229.0 || block.val('P') == 230.0 || block.val('P') == 231.0 || block.val('P') == 232.0 || block.val('P') == 233.0 || block.val('P') == 234.0 || block.val('P') == 236.0 || block.val('P') == 238.0 || block.val('P') == 240.0 || block.val('P') == 242.0 || block.val('P') == 244.0 || block.val('P') == 246.0 || block.val('P') == 248.0 || block.val('P') == 250.0 || block.val('P') == 252.0 || block.val('P') == 235.0 || block.val('P') == 237.0 || block.val('P') == 239.0 || block.val('P') == 241.0 || block.val('P') == 243.0 || block.val('P') == 245.0 || block.val('P') == 247.0 || block.val('P') == 249.0 || block.val('P') == 251.0 || block.val('P') == 253.0 || block.val('P') == 254.0 || block.val('P') == 255.0 || block.val('P') == 256.0 || block.val('P') == 257.0 || block.val('P') == 258.0 || block.val('P') == 259.0 || block.val('P') == 260.0 || block.val('P') == 261.0 || block.val('P') == 262.0 || block.val('P') == 263.0 || block.val('P') == 264.0 || block.val('P') == 265.0 || block.val('P') == 266.0 || block.val('P') == 267.0 || block.val('P') == 268.0 || block.val('P') == 269.0 || block.val('P') == 270.0 || block.val('P') == 271.0 || block.val('P') == 272.0 || block.val('P') == 273.0 || block.val('P') == 274.0 || block.val('P') == 275.0 || block.val('P') == 276.0 || block.val('P') == 277.0 || block.val('P') == 278.0 || block.val('P') == 279.0 || block.val('P') == 280.0 || block.val('P') == 281.0 || block.val('P') == 282.0 || block.val('P') == 283.0 || block.val('P') == 284.0 || (block.val('P') == 286.0 || block.val('P') == 288.0 || block.val('P') == 290.0) || block.val('P') == 285.0 || (block.val('P') == 287.0 || block.val('P') == 289.0 || block.val('P') == 291.0))) return StartEDM94SameThread(block);
    ServiceEDMCoordinatedRTRetirementSameThread();
    if (m_edmCoordinatedRT.retirePending || (m_edmCoordinatedRT.submitted && !m_edmCoordinatedRT.retired))
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, m_gapDryRun.sourceLine, "EDM84_PREVIOUS_SESSION_BUSY"); return nullptr; }
    ServiceEDMCNCRetainedRTRetirementSameThread();
    if (m_edmCNCRetainedRT.retirePending || (m_edmCNCRetainedRT.submitted && !m_edmCNCRetainedRT.retired))
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, m_gapDryRun.sourceLine, "EDM83_PREVIOUS_SESSION_BUSY"); return nullptr; }
    if (block.has('P') && (block.val('P') == 69.0 || block.val('P') == 70.0 || block.val('P') == 71.0 || block.val('P') == 72.0 || block.val('P') == 73.0 || block.val('P') == 74.0 || block.val('P') == 75.0 || block.val('P') == 76.0 || block.val('P') == 77.0 || block.val('P') == 78.0 || block.val('P') == 79.0 || block.val('P') == 80.0 || block.val('P') == 81.0 || block.val('P') == 82.0 || block.val('P') == 83.0 || block.val('P') == 84.0))
        return StartEDMCoordinatedRTSameThread(block);
    if (block.has('P') && (block.val('P') == 67.0 || block.val('P') == 68.0))
        return StartEDMCNCRetainedRTSameThread(block);
    if (block.has('P') && (block.val('P') == 65.0 || block.val('P') == 66.0))
        return StartEDMCNCRetainedSimulationSameThread(block);
    if (block.has('P') && (block.val('P') == 63.0 || block.val('P') == 64.0))
        return StartEDMMultiAxisSimulationSameThread(block);
    ServiceEDMGapServoRetirementSameThread();
    if (m_edmPhysicalServoRT.retirePending)
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, m_gapDryRun.sourceLine, "EDM69_RETIREMENT_PENDING"); return nullptr; }
    if (block.has('P') && block.val('P') == 55.0) return StartEDMPhysicalServoRTSameThread(block);
    if (block.has('P') && (block.val('P') == 53.0 || block.val('P') == 54.0))
        return StartEDMPhysicalRecipeCaptureSameThread(block);
    if (block.has('P') && (block.val('P') == 35.0 || block.val('P') == 36.0 || block.val('P') == 37.0 || block.val('P') == 38.0 || block.val('P') == 39.0 || block.val('P') == 40.0 || block.val('P') == 41.0 || block.val('P') == 42.0 || block.val('P') == 43.0 || block.val('P') == 44.0 || block.val('P') == 45.0 || block.val('P') == 46.0 || block.val('P') == 47.0 || block.val('P') == 48.0 || block.val('P') == 49.0 || block.val('P') == 50.0 || block.val('P') == 51.0 || block.val('P') == 52.0 || block.val('P') == 56.0 || block.val('P') == 57.0 || block.val('P') == 58.0 || block.val('P') == 59.0 || block.val('P') == 60.0 || block.val('P') == 61.0 || block.val('P') == 62.0))
        return StartEDMGapDynamicSameThread(block);
    if (block.has('P') && (block.val('P') == 33.0 || block.val('P') == 34.0))
        return StartEDMGapServoRTSameThread(block);

    if (m_edmZFixture.cleanupPending || m_edmZFixture.resetPending)
    {
        RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, m_gapDryRun.sourceLine, "EDM28_CLEANUP_PENDING");
        return nullptr;
    }
    if (block.has('P') && (block.val('P') == 13.0 || block.val('P') == 14.0 || block.val('P') == 15.0 || block.val('P') == 16.0 || block.val('P') == 17.0 || block.val('P') == 18.0 || block.val('P') == 19.0 || block.val('P') == 20.0 || block.val('P') == 21.0 || block.val('P') == 22.0 || block.val('P') == 23.0 || block.val('P') == 24.0 || block.val('P') == 25.0 || block.val('P') == 26.0 || block.val('P') == 27.0 || block.val('P') == 28.0 || block.val('P') == 29.0 || block.val('P') == 30.0 || block.val('P') == 31.0 || block.val('P') == 32.0) && !m_gapDryRun.active)
        return StartEDMZFixtureSameThread(block);
    const int sourceLine = m_gapDryRun.sourceLine;
    // Attribute malformed isolated requests without relabeling an active test.
    if (!m_gapDryRun.active)
    {
        m_gapDryRun.shortFlushTest = block.has('P') && block.val('P') == 7.0;
        m_gapDryRun.automaticFlushTest = block.has('P') && block.val('P') == 8.0;
        m_gapDryRun.liveAutomaticFlushTest = block.has('P') && block.val('P') == 9.0;
        m_gapDryRun.processCoordinatorTest = block.has('P') && block.val('P') == 10.0;
        m_gapDryRun.liveProcessTest = block.has('P') && block.val('P') == 11.0;
        m_gapDryRun.motionBridgeTest = block.has('P') && block.val('P') == 12.0;
    }
    if (!IsGapDryRunBlockShapeValid(block))
    {
        RejectGapDryRunSameThread(AlarmManager::G_Code_Invalid_parameter,
            sourceLine, "BLOCK_SHAPE");
        return nullptr;
    }
    if (m_gapDryRun.active || m_mode != NCOperationMode::MEMORY || m_state != NCState::RUN ||
        AlarmManager::GetInstance().HasAlarm() || Close_System_Com_flag ||
        m_edmState == EDMState::NOT_READY || Homing.IsActive() ||
        ((block.val('P') == 7.0 || block.val('P') == 8.0 || block.val('P') == 9.0 || block.val('P') == 10.0 || block.val('P') == 11.0 || block.val('P') == 12.0) && !IsEDMShortFlushRunReadySameThread()) ||
        m_isG66Active || !m_macroStack.empty() || m_pathFeed.pending || m_pathArc.pending ||
        m_pathReplay.pending || m_pathHold.armed || m_pathHold.bound ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) ||
        m_motion.HasPendingSafetyOrRecoveryRequests() || !m_motion.IsGroupDone() ||
        m_motion.GetCommandIngressSize() != 0U || m_motion.GetCommandReplaySize() != 0U ||
        m_currentExecutingBlockDispatchId == NC_BLOCK_DISPATCH_ID_INVALID ||
        m_pathCoreLiveBookkeeping.currentRunToken == 0ULL ||
        m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)())
    {
        RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY,
            sourceLine, "NOT_READY");
        return nullptr;
    }
    m_gapDryRun = GapDryRunState{};
    m_edmLiveAutomaticFlushResume = EDMLiveAutomaticFlushResumeState{};
    m_edmLiveProcessResume = EDMLiveAutomaticFlushResumeState{};
    m_edmLiveProcessReceiptWait = EDMLiveAutomaticFlushResumeState{};
    m_gapDryRun.active = true;
    m_gapDryRun.processTest = block.val('P') == 2.0;
    m_gapDryRun.feedRetreatTest = block.val('P') == 3.0;
    m_gapDryRun.adcShadowTest = block.val('P') == 4.0;
    m_gapDryRun.voltageTest = block.val('P') == 5.0;
    m_gapDryRun.recipeTest = block.val('P') == 6.0;
    m_gapDryRun.shortFlushTest = block.val('P') == 7.0;
    m_gapDryRun.automaticFlushTest = block.val('P') == 8.0;
    m_gapDryRun.liveAutomaticFlushTest = block.val('P') == 9.0;
    m_gapDryRun.processCoordinatorTest = block.val('P') == 10.0;
    m_gapDryRun.liveProcessTest = block.val('P') == 11.0;
    m_gapDryRun.motionBridgeTest = block.val('P') == 12.0;
    m_gapDryRun.test = ++m_gapDryRunSerial;
    m_gapDryRun.run = m_pathCoreLiveBookkeeping.currentRunToken;
    m_gapDryRun.cache = GetBaseProgramCache().GetGeneration();
    m_gapDryRun.dispatch = m_currentExecutingBlockDispatchId;
    m_gapDryRun.sourceLine = sourceLine;
    if (!RestartGapDryRunSameThread()) return nullptr;
    return WaitForGapDryRunCallback;
}

void NCManager::PauseGapDryRunSameThread(const char* reason) noexcept
{
    CancelEDM94SameThread(reason);
    if (m_gapDryRun.coordinatedRTTest) { EndEDMCoordinatedRTSameThread(false, reason); return; }
    if (m_gapDryRun.cncRetainedRTTest) { EndEDMCNCRetainedRTSameThread(false, reason); return; }
    if (m_gapDryRun.cncRetainedSimulationTest) { EndEDMCNCRetainedSimulationSameThread(false, reason); return; }
    if (m_gapDryRun.multiAxisSimulationTest) { EndEDMMultiAxisSimulationSameThread(false, reason); return; }
    if (m_gapDryRun.physicalServoRTTest) { EndEDMPhysicalServoRTSameThread(false, reason); return; }
    // P53/P54 HOLD cancels only the NC wait; RESET and an explicit
    // new command are required. No silent recapture or retained window credit.
    if (m_gapDryRun.physicalRecipeCaptureTest)
    { EndEDMPhysicalRecipeCaptureSameThread(false, reason); return; }
    if (m_gapDryRun.runtimeServoTest) { PauseEDMGapServoRTSameThread(reason); return; }
    if (m_gapDryRun.physicalZFixtureTest) { PauseEDMZFixtureSameThread(reason); return; }
    // A second HOLD supersedes the pending START, even while already paused.
    m_edmLiveAutomaticFlushResume = EDMLiveAutomaticFlushResumeState{};
    m_edmLiveProcessResume = EDMLiveAutomaticFlushResumeState{};
    m_edmLiveProcessReceiptWait = EDMLiveAutomaticFlushResumeState{};
    if (!m_gapDryRun.active || m_gapDryRun.paused)
    {
        if ((m_gapDryRun.shortFlushTest || m_gapDryRun.automaticFlushTest || m_gapDryRun.liveAutomaticFlushTest || m_gapDryRun.processCoordinatorTest || m_gapDryRun.liveProcessTest || m_gapDryRun.motionBridgeTest) && m_gapDryRun.result == 1U)
        {
            m_edmShortFlushTest.Revoke();
            m_edmAutomaticFlushTest.Revoke();
            m_edmProcessCoordinatorTest.Revoke();
            m_edmMotionReplay.Revoke();
            m_edmMotionReplayReceiptPending = false;
            RevokeEDMLiveProcessSameThread();
            m_edmLiveAutomaticFlush.Revoke();
            m_gapDryRun.result = 2U; // A completed receipt cannot survive a later HOLD.
        }
        return;
    }
    m_gapDryRun.paused = true;
    m_edmProcessTest.Revoke();
    m_edmFeedRetreatTest.Revoke();
    m_edmAcquisitionTest.Revoke();
    m_edmVoltageTest.Revoke();
    m_edmRecipeTest.Revoke();
    m_edmShortFlushTest.Revoke();
    m_edmAutomaticFlushTest.Revoke();
    m_edmProcessCoordinatorTest.Revoke();
    m_edmMotionReplay.Revoke();
    m_edmMotionReplayReceiptPending = false;
    RevokeEDMLiveProcessSameThread();
    m_edmLiveAutomaticFlush.Revoke();
    if (!m_gapDryRun.shortFlushTest && !m_gapDryRun.automaticFlushTest && !m_gapDryRun.liveAutomaticFlushTest && !m_gapDryRun.processCoordinatorTest && !m_gapDryRun.liveProcessTest && !m_gapDryRun.motionBridgeTest) m_gapInput.Reset();
    if (m_gapDryRun.shortFlushTest || m_gapDryRun.automaticFlushTest || m_gapDryRun.liveAutomaticFlushTest || m_gapDryRun.processCoordinatorTest || m_gapDryRun.liveProcessTest || m_gapDryRun.motionBridgeTest)
    {
        char text[512]{};
        const int length = std::snprintf(text, sizeof(text),
            "[%s] event=PAUSED test=%llu run=%llu dispatch=%llu line=%d case=%u passedMask=%08X reason=%.80s restart=REQUIRED mode=SHADOW_ONLY motion=0 physicalPermit=0 discharge=0\n",
            m_gapDryRun.motionBridgeTest ? "EDM27" : m_gapDryRun.liveProcessTest ? "EDM26" : m_gapDryRun.processCoordinatorTest ? "EDM25" : (m_gapDryRun.liveAutomaticFlushTest ? "EDM24" : (m_gapDryRun.automaticFlushTest ? "EDM23" : "EDM22")),
            static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(m_gapDryRun.run),
            static_cast<unsigned long long>(m_gapDryRun.dispatch), m_gapDryRun.sourceLine,
            static_cast<unsigned int>(m_gapDryRun.phase), static_cast<unsigned int>(m_gapDryRun.passedMask),
            reason ? reason : "UNKNOWN");
        if (length >= 0 && static_cast<std::size_t>(length) < sizeof(text)) RtPrintf("%s", text);
        else RtPrintf("%s", m_gapDryRun.motionBridgeTest ? "[EDM27] event=PAUSED reason=LOG_FORMAT motion=0 physicalPermit=0 discharge=0\n" : m_gapDryRun.liveProcessTest ? "[EDM26] event=PAUSED reason=LOG_FORMAT motion=0 physicalPermit=0 discharge=0\n" : m_gapDryRun.processCoordinatorTest ? "[EDM25] event=PAUSED reason=LOG_FORMAT motion=0 physicalPermit=0 discharge=0\n" : m_gapDryRun.automaticFlushTest ? "[EDM23] event=PAUSED reason=LOG_FORMAT physicalPermit=0 discharge=0\n" : "[EDM22] event=PAUSED reason=LOG_FORMAT physicalPermit=0 discharge=0\n");
        return;
    }
    RtPrintf((m_gapDryRun.adcShadowTest || m_gapDryRun.voltageTest || m_gapDryRun.recipeTest || m_gapDryRun.shortFlushTest || m_gapDryRun.automaticFlushTest || m_gapDryRun.liveAutomaticFlushTest || m_gapDryRun.processCoordinatorTest || m_gapDryRun.liveProcessTest || m_gapDryRun.motionBridgeTest) ?
        "[%s] event=PAUSED test=%llu run=%llu dispatch=%llu case=%u reason=%s restart=REQUIRED simPermit=0 motion=0 physicalPermit=0 discharge=0\n" :
        "[%s] event=PAUSED test=%llu run=%llu dispatch=%llu case=%u reason=%s restart=REQUIRED simPermit=0 motion=0 discharge=0\n",
        GapDryRunTag(m_gapDryRun.processTest, m_gapDryRun.feedRetreatTest, m_gapDryRun.adcShadowTest, m_gapDryRun.voltageTest, m_gapDryRun.recipeTest, m_gapDryRun.shortFlushTest, m_gapDryRun.automaticFlushTest, m_gapDryRun.liveAutomaticFlushTest, m_gapDryRun.processCoordinatorTest, m_gapDryRun.liveProcessTest, m_gapDryRun.motionBridgeTest),
        static_cast<unsigned long long>(m_gapDryRun.test),
        static_cast<unsigned long long>(m_gapDryRun.run),
        static_cast<unsigned long long>(m_gapDryRun.dispatch),
        static_cast<unsigned int>(m_gapDryRun.phase), reason);
}

void NCManager::CancelGapDryRunSameThread(const char* reason, bool preserveCompletedEDM94) noexcept
{
    // The caller's completed-session LOAD admission preserves only the old
    // EDM94 zero latch. It does not suppress any other GAP cleanup.
    if (!preserveCompletedEDM94) CancelEDM94SameThread(reason);
    if (m_gapDryRun.coordinatedRTTest) { EndEDMCoordinatedRTSameThread(false, reason); return; }
    if (m_edmCoordinatedRT.retirePending || (m_edmCoordinatedRT.submitted && !m_edmCoordinatedRT.retired))
        EndEDMCoordinatedRTSameThread(false, reason);
    if (m_gapDryRun.cncRetainedRTTest) { EndEDMCNCRetainedRTSameThread(false, reason); return; }
    if (m_edmCNCRetainedRT.retirePending || (m_edmCNCRetainedRT.submitted && !m_edmCNCRetainedRT.retired))
        EndEDMCNCRetainedRTSameThread(false, reason);
    InvalidateEDMPhysicalSequencePlanSameThread();
    if (m_gapDryRun.cncRetainedSimulationTest) { EndEDMCNCRetainedSimulationSameThread(false, reason); return; }
    if (m_gapDryRun.multiAxisSimulationTest) { EndEDMMultiAxisSimulationSameThread(false, reason); return; }
    if (m_gapDryRun.physicalServoRTTest) { EndEDMPhysicalServoRTSameThread(false, reason); return; }
    if (m_gapDryRun.physicalRecipeCaptureTest)
    { EndEDMPhysicalRecipeCaptureSameThread(false, reason); return; }
    if (m_gapDryRun.runtimeServoTest) { EndEDMGapServoRTSameThread(false, reason); return; }
    if (m_gapDryRun.physicalZFixtureTest)
    {
        if (m_gapDryRun.active || m_gapDryRun.result == 1U) EndEDMZFixtureSameThread("CANCELLED", reason);
        return;
    }
    m_edmLiveAutomaticFlushResume = EDMLiveAutomaticFlushResumeState{};
    m_edmLiveProcessResume = EDMLiveAutomaticFlushResumeState{};
    m_edmLiveProcessReceiptWait = EDMLiveAutomaticFlushResumeState{};
    if (!m_gapDryRun.active)
    {
        if (m_gapDryRun.shortFlushTest || m_gapDryRun.automaticFlushTest || m_gapDryRun.liveAutomaticFlushTest || m_gapDryRun.processCoordinatorTest || m_gapDryRun.liveProcessTest || m_gapDryRun.motionBridgeTest)
        {
            m_edmShortFlushTest.Revoke();
            m_edmAutomaticFlushTest.Revoke();
            m_edmProcessCoordinatorTest.Revoke();
            m_edmMotionReplay.Revoke();
            m_edmMotionReplayReceiptPending = false;
            RevokeEDMLiveProcessSameThread();
            m_edmLiveAutomaticFlush.Revoke();
            if (m_gapDryRun.result == 1U) m_gapDryRun.result = 2U;
        }
        return;
    }
    if (m_gapDryRun.motionBridgeTest)
    {
        EndEDMMotionReplaySameThread("CANCELLED", reason, m_gapDryRun.lastServiceMs); return;
    }
    if (m_gapDryRun.liveProcessTest)
    {
        EndEDMLiveProcessSameThread("CANCELLED", reason, m_gapDryRun.lastServiceMs);
        return;
    }
    if (m_gapDryRun.liveAutomaticFlushTest)
    {
        EndEDMLiveAutomaticFlushSameThread("CANCELLED", reason, m_gapDryRun.lastServiceMs);
        return;
    }
    m_gapDryRun.active = false;
    m_gapDryRun.paused = false;
    m_gapDryRun.result = 2U;
    m_edmProcessTest.Revoke();
    m_edmFeedRetreatTest.Revoke();
    m_edmAcquisitionTest.Revoke();
    m_edmVoltageTest.Revoke();
    m_edmRecipeTest.Revoke();
    m_edmShortFlushTest.Revoke();
    m_edmAutomaticFlushTest.Revoke();
    m_edmProcessCoordinatorTest.Revoke();
    m_edmMotionReplay.Revoke();
    m_edmMotionReplayReceiptPending = false;
    RevokeEDMLiveProcessSameThread();
    m_edmLiveAutomaticFlush.Revoke();
    if (!m_gapDryRun.shortFlushTest && !m_gapDryRun.automaticFlushTest && !m_gapDryRun.liveAutomaticFlushTest && !m_gapDryRun.processCoordinatorTest && !m_gapDryRun.liveProcessTest && !m_gapDryRun.motionBridgeTest) m_gapInput.Reset();
    if (m_gapDryRun.shortFlushTest || m_gapDryRun.automaticFlushTest || m_gapDryRun.liveAutomaticFlushTest || m_gapDryRun.processCoordinatorTest || m_gapDryRun.liveProcessTest || m_gapDryRun.motionBridgeTest)
    {
        char text[512]{};
        const int length = std::snprintf(text, sizeof(text),
            "[%s] event=SUMMARY result=CANCELLED test=%llu run=%llu dispatch=%llu line=%d case=%u passedMask=%08X reason=%.80s restart=REQUIRED mode=SHADOW_ONLY motion=0 physicalPermit=0 discharge=0\n",
            m_gapDryRun.motionBridgeTest ? "EDM27" : m_gapDryRun.liveProcessTest ? "EDM26" : m_gapDryRun.processCoordinatorTest ? "EDM25" : (m_gapDryRun.liveAutomaticFlushTest ? "EDM24" : (m_gapDryRun.automaticFlushTest ? "EDM23" : "EDM22")),
            static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(m_gapDryRun.run),
            static_cast<unsigned long long>(m_gapDryRun.dispatch), m_gapDryRun.sourceLine,
            static_cast<unsigned int>(m_gapDryRun.phase), static_cast<unsigned int>(m_gapDryRun.passedMask),
            reason ? reason : "UNKNOWN");
        if (length >= 0 && static_cast<std::size_t>(length) < sizeof(text)) RtPrintf("%s", text);
        else RtPrintf("%s", m_gapDryRun.motionBridgeTest ? "[EDM27] event=SUMMARY result=CANCELLED reason=LOG_FORMAT motion=0 physicalPermit=0 discharge=0\n" : m_gapDryRun.liveProcessTest ? "[EDM26] event=SUMMARY result=CANCELLED reason=LOG_FORMAT motion=0 physicalPermit=0 discharge=0\n" : m_gapDryRun.processCoordinatorTest ? "[EDM25] event=SUMMARY result=CANCELLED reason=LOG_FORMAT motion=0 physicalPermit=0 discharge=0\n" : m_gapDryRun.automaticFlushTest ? "[EDM23] event=SUMMARY result=CANCELLED reason=LOG_FORMAT physicalPermit=0 discharge=0\n" : "[EDM22] event=SUMMARY result=CANCELLED reason=LOG_FORMAT physicalPermit=0 discharge=0\n");
        return;
    }
    RtPrintf((m_gapDryRun.adcShadowTest || m_gapDryRun.voltageTest || m_gapDryRun.recipeTest || m_gapDryRun.shortFlushTest || m_gapDryRun.automaticFlushTest || m_gapDryRun.liveAutomaticFlushTest || m_gapDryRun.processCoordinatorTest || m_gapDryRun.liveProcessTest || m_gapDryRun.motionBridgeTest) ?
        "[%s] event=SUMMARY result=CANCELLED test=%llu run=%llu dispatch=%llu line=%d case=%u passedMask=%04X reason=%s simPermit=0 motion=0 physicalPermit=0 discharge=0\n" :
        "[%s] event=SUMMARY result=CANCELLED test=%llu run=%llu dispatch=%llu line=%d case=%u passedMask=%04X reason=%s simPermit=0 motion=0 discharge=0\n",
        GapDryRunTag(m_gapDryRun.processTest, m_gapDryRun.feedRetreatTest, m_gapDryRun.adcShadowTest, m_gapDryRun.voltageTest, m_gapDryRun.recipeTest, m_gapDryRun.shortFlushTest, m_gapDryRun.automaticFlushTest, m_gapDryRun.liveAutomaticFlushTest, m_gapDryRun.processCoordinatorTest, m_gapDryRun.liveProcessTest, m_gapDryRun.motionBridgeTest),
        static_cast<unsigned long long>(m_gapDryRun.test),
        static_cast<unsigned long long>(m_gapDryRun.run),
        static_cast<unsigned long long>(m_gapDryRun.dispatch), m_gapDryRun.sourceLine,
        static_cast<unsigned int>(m_gapDryRun.phase),
        static_cast<unsigned int>(m_gapDryRun.passedMask), reason);
}

void NCManager::ValidateGapDryRunSameThread()
{
    ServiceEDMCoordinatedRTRetirementSameThread();
    if (m_gapDryRun.coordinatedRTTest) { ValidateEDMCoordinatedRTSameThread(); return; }
    // Deliberately independent of the flag RESET may have already cleared.
    ServiceEDMCNCRetainedRTRetirementSameThread();
    if (m_gapDryRun.cncRetainedRTTest) { ValidateEDMCNCRetainedRTSameThread(); return; }
    if (m_gapDryRun.cncRetainedSimulationTest) { ValidateEDMCNCRetainedSimulationSameThread(); return; }
    if (m_gapDryRun.multiAxisSimulationTest) { ValidateEDMMultiAxisSimulationSameThread(); return; }
    ServiceEDMPhysicalServoRetirementSameThread();
    if (m_gapDryRun.physicalServoRTTest) { ValidateEDMPhysicalServoRTSameThread(); return; }
    if (m_gapDryRun.runtimeServoTest) { ValidateEDMGapServoRTSameThread(); return; }
    ServiceEDMGapServoRetirementSameThread();
    // Read-only cancellation acknowledgement. This never submits or services
    // runtime work before the latest NC input/safety observation.
    ServiceEDMRTShadowCancelSameThread();
    if (m_gapDryRun.physicalRecipeCaptureTest)
    { ValidateEDMPhysicalRecipeCaptureSameThread(); return; }
    if (m_gapDryRun.physicalZFixtureTest)
    {
        if (EDM28::IsGapDynamicProfile(m_edmZFixture.config.profile))
        { ValidateEDMGapDynamicSameThread(); return; }
        if (m_edmZFixture.cleanupPending) ServiceEDMZFixtureCleanupSameThread();
        ServiceEDMZFixtureFailureAlarmSameThread();
        if (!m_gapDryRun.active) return;
        if (!IsEDMZFixtureAuthorityCurrentSameThread(true)) EndEDMZFixtureSameThread("CANCELLED", "AUTHORITY_CHANGED");
        else if (m_state == NCState::HOLD || m_edmState == EDMState::HOLD)
        {
            // A coherent sticky RT fault retires the fixture intent in HOLD
            // just as it does in RUN. Recovery may prove Stop for cleanup;
            // it must never silently turn a faulted HOLD into a new Arm.
            if (m_edmZFixture.gapCurveProfile && !ReadEDMZFixtureFeedbackSameThread(false))
                EndEDMZFixtureSameThread("FAIL", m_edmGapRapid.shortState.active ? "P29_P30_HELD_SOURCE_INVALID" : m_edmZFixture.gapCurveReplanProfile ? "P24_HELD_SOURCE_INVALID" : "P23_HELD_SOURCE_INVALID");
            else if (ReadEDMZFixtureFeedbackSameThread(false) && m_edmZFixture.feedback.capHeld && m_edmZFixture.feedback.latchedFault)
            {
                LogEDMZFixtureSameThread("RT_STOP", "FAULT", "HOLD_RT_LATCHED_FAULT");
                EndEDMZFixtureSameThread("FAIL", "RT_LATCHED_FAULT");
            }
            else PauseEDMZFixtureSameThread("HOLD");
        }
        return;
    }
    if (!m_gapDryRun.active) return;
    if (AlarmManager::GetInstance().HasAlarm() || m_state == NCState::ALARM ||
        ((m_gapDryRun.shortFlushTest || m_gapDryRun.automaticFlushTest || m_gapDryRun.liveAutomaticFlushTest || m_gapDryRun.processCoordinatorTest || m_gapDryRun.liveProcessTest || m_gapDryRun.motionBridgeTest) && m_edmState == EDMState::ALARM))
        CancelGapDryRunSameThread("ALARM");
    else if (Close_System_Com_flag)
        CancelGapDryRunSameThread("SHUTDOWN");
    else if ((m_gapDryRun.shortFlushTest || m_gapDryRun.automaticFlushTest || m_gapDryRun.liveAutomaticFlushTest || m_gapDryRun.processCoordinatorTest || m_gapDryRun.liveProcessTest || m_gapDryRun.motionBridgeTest) &&
        (m_resetContinuationPhase != ResetContinuationPhase::IDLE || m_resetSafetyOutputHoldActive))
        CancelGapDryRunSameThread("RESET");
    else if (m_mode != NCOperationMode::MEMORY ||
        (m_state != NCState::RUN && m_state != NCState::HOLD) ||
        m_gapDryRun.run != m_pathCoreLiveBookkeeping.currentRunToken ||
        m_gapDryRun.cache != GetBaseProgramCache().GetGeneration() ||
        (m_gapDryRun.motionBridgeTest && m_gapDryRun.dispatch != m_waitingBlockDispatchId) ||
        m_isG66Active || !m_macroStack.empty())
        CancelGapDryRunSameThread("SCOPE_CHANGED");
    else if (m_gapDryRun.liveProcessTest &&
        (m_recipe.Current().generation != m_edmLiveProcessBinding.generation ||
         m_edmProcessProfile.revision != m_edmLiveProcessBinding.config.flush.configRevision))
        CancelGapDryRunSameThread("COND_OR_CONFIG_CHANGED");
    else if ((m_gapDryRun.processCoordinatorTest || m_gapDryRun.liveProcessTest || m_gapDryRun.motionBridgeTest) &&
        (m_motion.HasPendingSafetyOrRecoveryRequests() || Homing.IsActive()))
        CancelGapDryRunSameThread(m_gapDryRun.motionBridgeTest ? "EDM27_SAFETY_CHANGED" : m_gapDryRun.liveProcessTest ? "EDM26_SAFETY_CHANGED" : "EDM25_SAFETY_CHANGED");
    else if ((m_gapDryRun.liveAutomaticFlushTest || m_gapDryRun.processCoordinatorTest || m_gapDryRun.liveProcessTest || m_gapDryRun.motionBridgeTest) &&
        (!m_externalReadyInterlock || (m_state != NCState::HOLD && m_edmState != EDMState::HOLD && m_edmState == EDMState::NOT_READY)))
        CancelGapDryRunSameThread("INTERLOCK_LOST");
    else if (m_state == NCState::HOLD || m_edmState == EDMState::NOT_READY ||
        ((m_gapDryRun.shortFlushTest || m_gapDryRun.automaticFlushTest || m_gapDryRun.liveAutomaticFlushTest || m_gapDryRun.processCoordinatorTest || m_gapDryRun.liveProcessTest || m_gapDryRun.motionBridgeTest) && (m_edmState == EDMState::HOLD || !m_externalReadyInterlock)))
        PauseGapDryRunSameThread("HOLD_OR_INTERLOCK");
    else if ((!m_gapDryRun.paused || m_edmLiveAutomaticFlushResume.pending || m_edmLiveProcessResume.pending) &&
        (m_gapDryRun.epoch != m_motion.GetCurrentExecutionEpoch() ||
            !m_gapDryRun.lease.Matches(m_programMotionLease) ||
            !m_motion.IsMotionOwnerLeaseCurrent(m_gapDryRun.lease)))
    {
        if (m_gapDryRun.processCoordinatorTest || m_gapDryRun.liveProcessTest || m_gapDryRun.motionBridgeTest) CancelGapDryRunSameThread("IDENTITY_CHANGED");
        else RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "IDENTITY_CHANGED");
    }
}

bool NCManager::WaitForGapDryRunCallback(NCManager* nc)
{
    return nc != nullptr && nc->ProcessGapDryRunSameThread();
}

bool NCManager::ProcessGapDryRunSameThread()
{
    if (m_gapDryRun.coordinatedRTTest) return ProcessEDMCoordinatedRTSameThread();
    if (m_gapDryRun.cncRetainedRTTest) return ProcessEDMCNCRetainedRTSameThread();
    if (m_gapDryRun.cncRetainedSimulationTest) return ProcessEDMCNCRetainedSimulationSameThread();
    if (m_gapDryRun.multiAxisSimulationTest) return ProcessEDMMultiAxisSimulationSameThread();
    if (m_gapDryRun.physicalServoRTTest) return ProcessEDMPhysicalServoRTSameThread();
    if (m_gapDryRun.physicalRecipeCaptureTest) return ProcessEDMPhysicalRecipeCaptureSameThread();
    if (m_gapDryRun.runtimeServoTest) return ProcessEDMGapServoRTSameThread();
    if (m_gapDryRun.physicalZFixtureTest) return ProcessEDMZFixtureSameThread();
    const bool adcWasActive = (m_gapDryRun.adcShadowTest || m_gapDryRun.voltageTest || m_gapDryRun.recipeTest || m_gapDryRun.shortFlushTest || m_gapDryRun.automaticFlushTest || m_gapDryRun.liveAutomaticFlushTest || m_gapDryRun.processCoordinatorTest || m_gapDryRun.liveProcessTest || m_gapDryRun.motionBridgeTest) && m_gapDryRun.active;
    ValidateGapDryRunSameThread();
    // Stop this callback after cancellation. P11 also emits bounded EDM46/47
    // diagnostic pairs at existing progress/lifecycle boundaries, never per RT tick.
    // P8 restart BEGIN also reports read-only current recipe binding/plan metadata.
    if (adcWasActive && !m_gapDryRun.active) return false;
    if (!m_gapDryRun.active)
    {
        if (m_gapDryRun.motionBridgeTest && m_gapDryRun.result == 1U)
        {
            std::uint64_t receiptNowMs = 0ULL;
            const auto status = ReadGapDryRunClockSameThread(receiptNowMs) ?
                CheckEDMMotionReplayReceiptSameThread(receiptNowMs) : EDMLiveProcessReceiptStatus::Invalid;
            if (receiptNowMs != 0ULL) m_gapDryRun.lastServiceMs = receiptNowMs;
            if (status == EDMLiveProcessReceiptStatus::Wait) return false;
            if (status == EDMLiveProcessReceiptStatus::Invalid)
            {
                RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine,
                    "EDM27_RECEIPT_NOT_CURRENT");
                return false;
            }
        }
        if (m_gapDryRun.liveProcessTest && m_gapDryRun.result == 1U)
        {
            std::uint64_t receiptNowMs = 0ULL;
            const auto status = ReadGapDryRunClockSameThread(receiptNowMs) ?
                CheckEDMLiveProcessReceiptSameThread(receiptNowMs) : EDMLiveProcessReceiptStatus::Invalid;
            if (receiptNowMs != 0ULL) m_gapDryRun.lastServiceMs = receiptNowMs;
            if (status == EDMLiveProcessReceiptStatus::Wait) return false;
            if (status == EDMLiveProcessReceiptStatus::Invalid)
            {
                RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine,
                    "EDM26_RECEIPT_NOT_CURRENT");
                return false;
            }
        }
        if (m_gapDryRun.liveAutomaticFlushTest && m_gapDryRun.result == 1U)
        {
            std::uint64_t receiptNowMs = 0ULL;
            if (!ReadGapDryRunClockSameThread(receiptNowMs) || !IsEDMLiveAutomaticFlushReceiptCurrentSameThread(receiptNowMs))
            {
                const auto& receipt = m_edmLiveAutomaticFlush.Snapshot();
                const bool duplicate = m_edmProcessObservedGap.sampleSequence <= receipt.lastSampleSeq ||
                    m_edmProcessObservedGap.sampledAtMs <= receipt.lastSampleTimeMs;
                RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine,
                    duplicate ? "DUPLICATE_SAMPLE" : "EDM24_RECEIPT_NOT_CURRENT");
                return false;
            }
        }
        if (m_gapDryRun.result == 1U && m_state == NCState::RUN &&
            m_mode == NCOperationMode::MEMORY && !AlarmManager::GetInstance().HasAlarm() &&
            m_gapDryRun.run == m_pathCoreLiveBookkeeping.currentRunToken &&
            m_gapDryRun.cache == GetBaseProgramCache().GetGeneration() &&
            m_gapDryRun.dispatch == m_waitingBlockDispatchId &&
            m_gapDryRun.epoch == m_motion.GetCurrentExecutionEpoch() &&
            m_gapDryRun.lease.Matches(m_programMotionLease) &&
            m_motion.IsMotionOwnerLeaseCurrent(m_gapDryRun.lease) &&
            ((!m_gapDryRun.shortFlushTest && !m_gapDryRun.automaticFlushTest && !m_gapDryRun.liveAutomaticFlushTest && !m_gapDryRun.processCoordinatorTest && !m_gapDryRun.liveProcessTest && !m_gapDryRun.motionBridgeTest) || (IsEDMShortFlushRunReadySameThread() &&
                !m_isG66Active && m_macroStack.empty() &&
                !m_motion.HasPendingSafetyOrRecoveryRequests())) &&
            ((!m_gapDryRun.processCoordinatorTest && !m_gapDryRun.liveProcessTest && !m_gapDryRun.motionBridgeTest) || IsEDMCoordinatorAuthorityCurrentSameThread()))
        {
            if (m_gapDryRun.motionBridgeTest)
            {
                char line[512]{};
                const auto& receipt = m_edmMotionReplay.Current();
                const int length = std::snprintf(line, sizeof(line),
                    "[EDM27] event=RECEIPT result=READY test=%llu run=%llu dispatch=%llu receiptSeq=%llu proof=SYNTHETIC fixture=INDEPENDENT source=SIMULATED motion=0 physicalPermit=0 discharge=0\n",
                    static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(m_gapDryRun.run),
                    static_cast<unsigned long long>(m_gapDryRun.dispatch), static_cast<unsigned long long>(receipt.receiptSequence));
                if (length < 0 || static_cast<std::size_t>(length) >= sizeof(line))
                { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM27_LOG_FORMAT"); return false; }
                RtPrintf("%s", line);
            }
            if (m_gapDryRun.liveProcessTest)
            {
                char line[512]{};
                const auto& gap = m_edmProcessObservedGap;
                const int length = std::snprintf(line, sizeof(line),
                    "[EDM26] event=RECEIPT result=READY test=%llu run=%llu dispatch=%llu seq=%llu sampleMs=%llu source=%s AD=%u mode=LIVE_COND_SHADOW motion=0 physicalPermit=0 discharge=0\n",
                    static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(m_gapDryRun.run),
                    static_cast<unsigned long long>(m_gapDryRun.dispatch), static_cast<unsigned long long>(gap.sampleSequence),
                    static_cast<unsigned long long>(gap.sampledAtMs), EDMGap::SourceName(gap.configuredSource), static_cast<unsigned>(gap.adIndex));
                if (length < 0 || static_cast<std::size_t>(length) >= sizeof(line))
                { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM26_LOG_FORMAT"); return false; }
                RtPrintf("%s", line);
            }
            return true;
        }
        if ((m_gapDryRun.shortFlushTest || m_gapDryRun.automaticFlushTest || m_gapDryRun.liveAutomaticFlushTest || m_gapDryRun.processCoordinatorTest || m_gapDryRun.liveProcessTest || m_gapDryRun.motionBridgeTest) && m_gapDryRun.result == 1U)
        {
            m_edmShortFlushTest.Revoke();
            m_edmAutomaticFlushTest.Revoke();
            m_edmProcessCoordinatorTest.Revoke();
            m_edmMotionReplay.Revoke();
            m_edmMotionReplayReceiptPending = false;
            RevokeEDMLiveProcessSameThread();
            m_edmLiveAutomaticFlush.Revoke();
            m_gapDryRun.result = 2U;
        }
        if (m_state == NCState::RUN && !AlarmManager::GetInstance().HasAlarm())
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
                m_gapDryRun.sourceLine, "CANCELLED_CALLBACK");
        return false;
    }
    if (m_state != NCState::RUN || m_edmState == EDMState::NOT_READY ||
        ((m_gapDryRun.shortFlushTest || m_gapDryRun.automaticFlushTest || m_gapDryRun.liveAutomaticFlushTest || m_gapDryRun.processCoordinatorTest || m_gapDryRun.liveProcessTest || m_gapDryRun.motionBridgeTest) && (m_edmState == EDMState::HOLD || !m_externalReadyInterlock))) return false;
    if (m_gapDryRun.dispatch != m_waitingBlockDispatchId ||
        !m_motion.IsMotionOwnerLeaseCurrent(m_programMotionLease) ||
        m_motion.HasPendingSafetyOrRecoveryRequests() ||
        ((m_gapDryRun.shortFlushTest || m_gapDryRun.automaticFlushTest || m_gapDryRun.liveAutomaticFlushTest || m_gapDryRun.processCoordinatorTest || m_gapDryRun.liveProcessTest || m_gapDryRun.motionBridgeTest) && Homing.IsActive()))
    {
        if (m_gapDryRun.processCoordinatorTest || m_gapDryRun.liveProcessTest || m_gapDryRun.motionBridgeTest) CancelGapDryRunSameThread("CALLBACK_SCOPE");
        else RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "CALLBACK_SCOPE");
        return false;
    }
    if (m_gapDryRun.paused)
    {
        if (m_gapDryRun.liveProcessTest)
            return ProcessEDMLiveProcessResumeSameThread();
        if (m_gapDryRun.liveAutomaticFlushTest)
            return ProcessEDMLiveAutomaticFlushResumeSameThread();
        if (m_gapDryRun.restarts == (std::numeric_limits<std::uint32_t>::max)())
        {
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
                m_gapDryRun.sourceLine, "RESTART_EXHAUSTED");
            return false;
        }
        ++m_gapDryRun.restarts;
        if (m_gapDryRun.adcShadowTest || m_gapDryRun.voltageTest || m_gapDryRun.recipeTest || m_gapDryRun.shortFlushTest || m_gapDryRun.automaticFlushTest || m_gapDryRun.liveAutomaticFlushTest || m_gapDryRun.processCoordinatorTest || m_gapDryRun.liveProcessTest || m_gapDryRun.motionBridgeTest)
        {
            if (m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)())
            {
                RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
                    m_gapDryRun.sourceLine, "TEST_SERIAL_EXHAUSTED");
                return false;
            }
            m_gapDryRun.test = ++m_gapDryRunSerial;
        }
        if (!RestartGapDryRunSameThread()) return false;
        if (m_gapDryRun.adcShadowTest || m_gapDryRun.voltageTest || m_gapDryRun.recipeTest || m_gapDryRun.shortFlushTest || m_gapDryRun.automaticFlushTest || m_gapDryRun.liveAutomaticFlushTest || m_gapDryRun.processCoordinatorTest || m_gapDryRun.liveProcessTest || m_gapDryRun.motionBridgeTest) return false;
    }
    if ((m_gapDryRun.processCoordinatorTest || m_gapDryRun.liveProcessTest || m_gapDryRun.motionBridgeTest) && !IsEDMCoordinatorAuthorityCurrentSameThread())
    {
        CancelGapDryRunSameThread(m_gapDryRun.motionBridgeTest ? "EDM27_AUTHORITY_CHANGED" : m_gapDryRun.liveProcessTest ? "EDM26_AUTHORITY_CHANGED" : "EDM25_AUTHORITY_CHANGED");
        return false;
    }
    std::uint64_t nowMs = 0ULL;
    if (!ReadGapDryRunClockSameThread(nowMs))
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "CLOCK_READ_FAILED");
        return false;
    }
    if (nowMs < m_gapDryRun.lastServiceMs)
    {
        if (!m_gapDryRun.adcShadowTest && !m_gapDryRun.voltageTest && !m_gapDryRun.recipeTest && !m_gapDryRun.shortFlushTest && !m_gapDryRun.automaticFlushTest && !m_gapDryRun.liveAutomaticFlushTest && !m_gapDryRun.processCoordinatorTest && !m_gapDryRun.liveProcessTest && !m_gapDryRun.motionBridgeTest)
        RtPrintf("[%s] event=CLOCK_FAULT build=%s reason=MILLISECOND_REGRESSION nowMs=%llu lastServiceMs=%llu test=%llu\n",
            GapDryRunTag(m_gapDryRun.processTest, m_gapDryRun.feedRetreatTest),
            m_gapDryRun.feedRetreatTest ? "EDM02" : "EDM01_FIX1",
            static_cast<unsigned long long>(nowMs),
            static_cast<unsigned long long>(m_gapDryRun.lastServiceMs),
            static_cast<unsigned long long>(m_gapDryRun.test));
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "CLOCK_REGRESSION");
        return false;
    }
    const std::uint64_t serviceGapMs = nowMs - m_gapDryRun.lastServiceMs;
    if (serviceGapMs > GapMaxServiceGapMs)
    {
        if (m_gapDryRun.motionBridgeTest || m_gapDryRun.liveProcessTest || m_gapDryRun.liveAutomaticFlushTest)
        {
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine,
                m_gapDryRun.motionBridgeTest ? "EDM27_SERVICE_GAP" : m_gapDryRun.liveProcessTest ? "EDM26_SERVICE_GAP" : "EDM24_SERVICE_GAP");
            return false;
        }
        if (m_gapDryRun.adcShadowTest || m_gapDryRun.voltageTest || m_gapDryRun.recipeTest || m_gapDryRun.shortFlushTest || m_gapDryRun.automaticFlushTest || m_gapDryRun.liveAutomaticFlushTest || m_gapDryRun.processCoordinatorTest || m_gapDryRun.liveProcessTest || m_gapDryRun.motionBridgeTest)
        {
            m_edmAcquisitionTest.Revoke();
            m_edmVoltageTest.Revoke();
            m_edmRecipeTest.Revoke();
            m_edmShortFlushTest.Revoke();
            m_edmAutomaticFlushTest.Revoke();
            m_edmProcessCoordinatorTest.Revoke();
            m_edmMotionReplay.Revoke();
            m_edmMotionReplayReceiptPending = false;
            RevokeEDMLiveProcessSameThread();
            m_edmLiveAutomaticFlush.Revoke();
            if (m_gapDryRun.restarts == (std::numeric_limits<std::uint32_t>::max)() ||
                m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)())
            {
                RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
                    m_gapDryRun.sourceLine, "RESTART_EXHAUSTED");
                return false;
            }
            ++m_gapDryRun.restarts;
            m_gapDryRun.test = ++m_gapDryRunSerial;
            (void)RestartGapDryRunSameThread(); // One BEGIN, no extra active print.
            return false;
        }
        if (m_gapDryRun.feedRetreatTest)
        {
            // P3 has one continuous simulated consumer across its cases. A
            // scheduling break invalidates the whole session, including all
            // ACKs, virtual position and coverage. Never retain partial P3
            // progress or integrate a large real-time delta into virtual motion.
            m_edmFeedRetreatTest.Revoke();
            RtPrintf("[EDM02] event=SCHEDULER_DELAY build=EDM02 test=%llu run=%llu dispatch=%llu gapMs=%llu thresholdMs=%llu action=REVOKE_RESTART clock=VIRTUAL_CONTINUOUS simPermit=0 virtualVelocity=0 motion=0 discharge=0\n",
                static_cast<unsigned long long>(m_gapDryRun.test),
                static_cast<unsigned long long>(m_gapDryRun.run),
                static_cast<unsigned long long>(m_gapDryRun.dispatch),
                static_cast<unsigned long long>(serviceGapMs),
                static_cast<unsigned long long>(GapMaxServiceGapMs));
            if (m_gapDryRun.restarts == (std::numeric_limits<std::uint32_t>::max)())
            {
                RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
                    m_gapDryRun.sourceLine, "RESTART_EXHAUSTED");
                return false;
            }
            ++m_gapDryRun.restarts;
            (void)RestartGapDryRunSameThread();
            return false;
        }
        if (m_gapDryRun.processTest)
        {
            // EDM01_FIX1: P2 cases have independent virtual GAP clocks. Wall
            // scheduling delay is not a stale simulated acquisition. Revoke
            // the prior observation, retain only completed cases, and wait a
            // fresh 500 ms before executing the next case. Never skip cases,
            // publish a sample, grant physical permission, or advance Motion.
            m_edmProcessTest.Revoke();
            m_gapDryRun.lastServiceMs = nowMs;
            m_gapDryRun.phaseStartMs = nowMs;
            m_gapDryRun.observations = 0U;
            RtPrintf("[EDM01] event=SCHEDULER_DELAY build=EDM01_FIX1 test=%llu run=%llu dispatch=%llu nextCase=%u passedMask=%08X gapMs=%llu thresholdMs=%llu action=REVOKE_DEFER clock=VIRTUAL_CASES simPermit=0 motion=0 discharge=0\n",
                static_cast<unsigned long long>(m_gapDryRun.test),
                static_cast<unsigned long long>(m_gapDryRun.run),
                static_cast<unsigned long long>(m_gapDryRun.dispatch),
                static_cast<unsigned int>(m_gapDryRun.phase + 1U),
                static_cast<unsigned int>(m_gapDryRun.passedMask),
                static_cast<unsigned long long>(serviceGapMs),
                static_cast<unsigned long long>(GapMaxServiceGapMs));
            return false;
        }
        // P1 services Monitor on the real clock; its original bound remains.
        RtPrintf("[GAP-CG] event=SERVICE_GAP build=EDM01_FIX1 test=%llu nowMs=%llu lastServiceMs=%llu gapMs=%llu limitMs=%llu\n",
            static_cast<unsigned long long>(m_gapDryRun.test),
            static_cast<unsigned long long>(nowMs),
            static_cast<unsigned long long>(m_gapDryRun.lastServiceMs),
            static_cast<unsigned long long>(serviceGapMs),
            static_cast<unsigned long long>(GapMaxServiceGapMs));
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "SERVICE_GAP");
        return false;
    }
    m_gapDryRun.lastServiceMs = nowMs;
    if (m_gapDryRun.motionBridgeTest) return ProcessEDMMotionReplaySameThread(nowMs);
    if (m_gapDryRun.liveProcessTest) return ProcessEDMLiveProcessSameThread(nowMs);
    if (m_gapDryRun.processCoordinatorTest) return ProcessEDMCoordinatorDryRunCaseSameThread(nowMs);
    if (m_gapDryRun.liveAutomaticFlushTest) return ProcessEDMLiveAutomaticFlushSameThread(nowMs);
    if (m_gapDryRun.automaticFlushTest) return ProcessEDMAutomaticFlushDryRunCaseSameThread(nowMs);
    if (m_gapDryRun.shortFlushTest) return ProcessEDMShortFlushDryRunCaseSameThread(nowMs);
    if (m_gapDryRun.recipeTest) return ProcessEDMRecipeDryRunCaseSameThread(nowMs);
    if (m_gapDryRun.voltageTest) return ProcessEDMVoltageDryRunCaseSameThread(nowMs);
    if (m_gapDryRun.adcShadowTest) return ProcessEDMAcquisitionDryRunCaseSameThread(nowMs);
    if (m_gapDryRun.feedRetreatTest) return ProcessEDMFeedRetreatDryRunCaseSameThread(nowMs);
    if (m_gapDryRun.processTest) return ProcessEDMProcessDryRunCaseSameThread(nowMs);
    const std::uint32_t phase = m_gapDryRun.phase;
    if (phase >= GapCaseCount)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "CASE_INDEX");
        return false;
    }
    if (phase == 0U) m_gapInput.Poll(nowMs);
    else if (phase == 10U) m_gapInput.Publish(m_gapDryRun.sample, nowMs);
    else if (phase == 12U)
    {
        EDMGap::Sample replay = m_gapDryRun.sample;
        ++replay.voltageMv;
        m_gapInput.Publish(replay, nowMs);
    }
    else
    {
        EDMGap::Sample sample{};
        sample.source = EDMGap::Source::SIMULATED;
        sample.sequence = ++m_gapDryRun.sequence;
        sample.sampledAtMs = nowMs;
        sample.valid = phase != 8U;
        sample.voltageMv = 50000;
        if (phase == 2U) sample.voltageMv = 0;
        else if (phase == 3U) sample.voltageMv = nowMs - m_gapDryRun.phaseStartMs >= 375ULL ?
            31000 : ((sample.sequence & 1ULL) ? 29000 : 31000);
        else if (phase == 4U) sample.voltageMv = 36000;
        else if (phase == 5U) sample.voltageMv = 90000;
        else if (phase == 6U) sample.voltageMv = nowMs - m_gapDryRun.phaseStartMs >= 375ULL ?
            79000 : ((sample.sequence & 1ULL) ? 79000 : 81000);
        else if (phase == 7U) sample.voltageMv = 70000;
        else if (phase == 14U) sample.voltageMv = 200001;
        m_gapDryRun.sample = sample;
        m_gapInput.Publish(sample, nowMs);
        ++m_gapDryRun.phaseSamples;
    }
    ++m_gapDryRun.observations;
    if (m_gapDryRun.observations >= 4096U)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "CLOCK_NOT_ADVANCING");
        return false;
    }
    const EDMGap::Snapshot& observed = m_gapInput.Current();
    if (observed.quality != EDMGap::Quality::VALID && observed.band != EDMGap::Band::UNKNOWN)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "INVALID_BAND");
        return false;
    }
    if ((phase == 3U || phase == 6U) &&
        (observed.quality != EDMGap::Quality::VALID || observed.band != GapExpectedBand(phase)))
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "HYSTERESIS_CHANGED");
        return false;
    }
    if (nowMs - m_gapDryRun.phaseStartMs < GapCaseMs) return false;
    const bool matching = observed.quality == GapExpectedQuality(phase) &&
        observed.band == GapExpectedBand(phase) && m_gapDryRun.observations >= 4U &&
        (GapExpectedQuality(phase) != EDMGap::Quality::VALID || m_gapDryRun.phaseSamples >= 4U);
    RtPrintf("[GAP-CG] event=CASE test=%llu restart=%u case=%u name=%s result=%s source=%s quality=%s band=%s mv=%d seq=%llu ageMs=%llu observations=%u samples=%u\n",
        static_cast<unsigned long long>(m_gapDryRun.test),
        static_cast<unsigned int>(m_gapDryRun.restarts), static_cast<unsigned int>(phase + 1U),
        GapCaseNames[phase], matching ? "PASS" : "FAIL", EDMGap::SourceName(observed.source),
        EDMGap::QualityName(observed.quality), EDMGap::BandName(observed.band),
        static_cast<int>(observed.voltageMv), static_cast<unsigned long long>(observed.sequence),
        static_cast<unsigned long long>(nowMs >= observed.sampledAtMs ? nowMs - observed.sampledAtMs : 0ULL),
        static_cast<unsigned int>(m_gapDryRun.observations), static_cast<unsigned int>(m_gapDryRun.phaseSamples));
    if (!matching)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, GapCaseNames[phase]);
        return false;
    }
    m_gapDryRun.passedMask |= 1U << phase;
    ++m_gapDryRun.phase;
    m_gapDryRun.phaseStartMs = nowMs;
    m_gapDryRun.observations = 0U;
    m_gapDryRun.phaseSamples = 0U;
    if (m_gapDryRun.phase != GapCaseCount) return false;
    if (m_gapDryRun.passedMask != GapAllCases)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "COVERAGE");
        return false;
    }
    m_gapDryRun.active = false;
    m_gapDryRun.result = 1U;
    RtPrintf("[GAP-CG] event=SUMMARY result=PASS test=%llu run=%llu dispatch=%llu line=%d restart=%u cases=16 passedMask=FFFF mode=SIMULATED motion=0 discharge=0\n",
        static_cast<unsigned long long>(m_gapDryRun.test),
        static_cast<unsigned long long>(m_gapDryRun.run),
        static_cast<unsigned long long>(m_gapDryRun.dispatch), m_gapDryRun.sourceLine,
        static_cast<unsigned int>(m_gapDryRun.restarts));
    if (!m_gapDryRun.shortFlushTest && !m_gapDryRun.automaticFlushTest && !m_gapDryRun.liveAutomaticFlushTest && !m_gapDryRun.processCoordinatorTest && !m_gapDryRun.liveProcessTest && !m_gapDryRun.motionBridgeTest) m_gapInput.Reset(); // A completed simulation cannot become a live voltage source.
    return true;
}

// EDM81 is a detached NC-owned simulation. Virtual 250 us steps are bounded
// calculation batches at NC service cadence, never EtherCAT timing evidence.
WaitConditionFunc NCManager::StartEDMMultiAxisSimulationSameThread(const NCBlock& block)
{
    const int line = m_gapDryRun.sourceLine;
    if (m_gapDryRun.active || m_edmZFixture.cleanupPending || m_edmZFixture.resetPending ||
        m_edmZFixture.failureAlarmPending || m_edmPhysicalServoRT.retirePending || m_edmGapServoRT.retirePending)
    { RejectEDMGapAdmissionSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM81_PREVIOUS_SESSION_BUSY"); return nullptr; }
    const unsigned profile = block.has('P') && block.val('P') == 64.0 ? 64U : 63U;
    const int pc = GetActiveDispatchPC();
    const NCProgramCacheLine* authored = pc >= 0 ? GetBaseProgramCache().TryGetLine(pc) : nullptr;
    bool literal = authored && authored->sourceLineNumber == line &&
        EDM77::Detail::Clean(authored->parsedBlock) && !authored->parsedBlock.isEmpty &&
        authored->parsedBlock.gCount == 1 && authored->parsedBlock.mCount == 0 &&
        EDM77::Detail::EqualLiteral(authored->parsedBlock.gExpressions[0], 180U) &&
        EDM77::Detail::OnlyAddresses(authored->parsedBlock, "NP") &&
        authored->parsedBlock.has('P') &&
        EDM77::Detail::EqualLiteral(authored->parsedBlock.expression('P'), profile);
    if (literal)
    {
        for (char letter = 'A'; letter <= 'Z'; ++letter)
        {
            if (block.has(letter) != authored->parsedBlock.has(letter)) { literal = false; break; }
            if (!block.has(letter)) continue;
            std::uint32_t value = 0U;
            if (!EDM77::Detail::UnsignedLiteral(authored->parsedBlock.expression(letter),
                (std::numeric_limits<std::uint32_t>::max)(), value) || block.val(letter) != static_cast<double>(value))
            { literal = false; break; }
        }
    }
    if (!IsGapDryRunBlockShapeValid(block) || block.isBlockSkip || !literal || !IsEDMZFixtureM30ContinuationSameThread())
    { RejectEDMGapAdmissionSameThread(AlarmManager::G_Code_Invalid_parameter, line, "EDM81_LITERAL_FIXTURE_M30_REQUIRED"); return nullptr; }
    std::uint64_t mask = 0ULL;
    (void)IsEDMGapServoRunReadySameThread(&mask);
    // Retain owner/epoch, quiescent queues and interlocks. The axis Arm proof
    // is deliberately irrelevant: this fixture has no Motion/PDO consumer.
    if ((mask & ~(1ULL << 9U)) || !m_edmProcessReady ||
        m_currentExecutingBlockDispatchId == NC_BLOCK_DISPATCH_ID_INVALID ||
        m_motion.GetCurrentExecutionEpoch() == MOTION_EXECUTION_EPOCH_INVALID ||
        !m_pathCoreLiveBookkeeping.currentRunToken || !GetBaseProgramCache().GetGeneration() ||
        m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)())
    { RejectEDMGapAdmissionSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM81_NOT_READY"); return nullptr; }
    EDMRecipe::Summary summary{};
    if (!ReadEDMRecipeSummarySameThread(summary))
    { RejectEDMGapAdmissionSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM81_RECIPE_SCOPE"); return nullptr; }
    const auto servo = EDM74::BindPhysicalBidirectionalRecipe(m_recipe, m_edmProcessProfile, m_edmProcessTuningGeneration);
    const auto flush = EDM75::BindPhysicalBidirectionalFlushRecipe(m_recipe, m_edmProcessProfile.machineProfileId);
    if (!servo.ready || !flush.ready || !EDM75::SameAdmission(flush.recipe, servo.recipe))
    { RejectEDMGapAdmissionSameThread(AlarmManager::G_Code_Invalid_parameter, line, "EDM81_RECIPE_BIND_REJECTED"); return nullptr; }
    LARGE_INTEGER frequency{};
    if (!RtQueryPerformanceFrequency(&frequency) || frequency.QuadPart <= 0 ||
        static_cast<std::uint64_t>(frequency.QuadPart) > (std::numeric_limits<std::uint64_t>::max)() / 1000000ULL)
    { RejectEDMGapAdmissionSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM81_CLOCK_FREQUENCY"); return nullptr; }
    // A cancelled earlier Q2 fixture cannot lend its retained mode to this
    // detached diagnostic in a fresh run. All physical busy guards passed.
    InvalidateEDMPhysicalSequencePlanSameThread();
    m_edmGapDynamic.strictSequenceMode = false;
    m_gapDryRun = GapDryRunState{};
    m_gapDryRun.multiAxisSimulationTest = true;
    m_gapDryRun.sourceLine = line;
    m_gapDryRun.test = ++m_gapDryRunSerial;
    m_gapDryRun.run = m_pathCoreLiveBookkeeping.currentRunToken;
    m_gapDryRun.cache = GetBaseProgramCache().GetGeneration();
    m_gapDryRun.dispatch = m_currentExecutingBlockDispatchId;
    m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch();
    m_gapDryRun.lease = m_programMotionLease;
    m_gapDryRun.frequency = static_cast<std::uint64_t>(frequency.QuadPart);
    auto& control = m_edmMultiAxisSimulation;
    control.profile = profile; control.sourcePC = pc;
    control.servo = servo.recipe; control.flush = flush.recipe;
    control.startedUs = control.lastClockUs = control.lastServiceUs = 0ULL;
    control.stalledPolls = 0U;
    control.minimumUs = profile == 64U ? 20000000ULL : 1000000ULL;
    control.deadlineUs = profile == 64U ? 35000000ULL : 30000000ULL;
    m_gapDryRun.active = true;
    if (!control.suite.Start(control.servo.curve, control.servo.referenceV, control.servo.overridePercent))
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, line, "EDM81_SUITE_START_REJECTED"); return nullptr; }
    RtPrintf("[EDM81] event=BEGIN P=%u test=%llu run=%llu dispatch=%llu windowMs=%llu source=SIM clock=VIRTUAL_250US executor=NC_10MS output=NONE axes=XYZCUV hold=CANCEL_RESET_REQUIRED motion=0 physicalPermit=0 discharge=0\n",
        profile, static_cast<unsigned long long>(m_gapDryRun.test),
        static_cast<unsigned long long>(m_gapDryRun.run), static_cast<unsigned long long>(m_gapDryRun.dispatch),
        static_cast<unsigned long long>(control.minimumUs / 1000ULL));
    char recipeText[480]{};
    const int recipeLength = std::snprintf(recipeText, sizeof(recipeText),
        "[EDM81] event=RECIPE_FROZEN table=%u E=%u generation=%llu processRev=%u tuningGen=%llu E8=%.6f E9=%.6f curve=INSTALLED input=SYNTHETIC apply=SIMULATION_ONLY motion=0 physicalPermit=0 discharge=0\n",
        control.servo.binding.tableId, static_cast<unsigned>(control.servo.binding.eCode),
        static_cast<unsigned long long>(control.servo.binding.generation), control.servo.binding.processRevision,
        static_cast<unsigned long long>(control.servo.binding.tuningGeneration),
        control.servo.overridePercent, control.servo.referenceV);
    if (recipeLength < 0 || static_cast<std::size_t>(recipeLength) >= sizeof(recipeText))
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, line, "EDM81_RECIPE_LOG_FORMAT"); return nullptr; }
    RtPrintf("%s", recipeText);
    std::uint64_t nowUs = 0ULL;
    if (!ReadEDMGapServoClockSameThread(nowUs))
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, line, "EDM81_CLOCK_READ"); return nullptr; }
    control.startedUs = control.lastClockUs = control.lastServiceUs = nowUs;
    return WaitForGapDryRunCallback;
}

bool NCManager::IsEDMMultiAxisSimulationScopeCurrentSameThread() const noexcept
{
    std::uint64_t mask = 0ULL;
    (void)IsEDMGapServoRunReadySameThread(&mask);
    return m_gapDryRun.multiAxisSimulationTest && !(mask & ~(1ULL << 9U)) &&
        !m_edmPhysicalServoRT.retirePending && !m_edmGapServoRT.retirePending &&
        m_gapDryRun.run != 0ULL && m_gapDryRun.run == m_pathCoreLiveBookkeeping.currentRunToken &&
        m_gapDryRun.cache != 0ULL && m_gapDryRun.cache == GetBaseProgramCache().GetGeneration() &&
        m_gapDryRun.dispatch != NC_BLOCK_DISPATCH_ID_INVALID && m_gapDryRun.dispatch == m_waitingBlockDispatchId &&
        m_gapDryRun.epoch != MOTION_EXECUTION_EPOCH_INVALID &&
        m_gapDryRun.epoch == m_motion.GetCurrentExecutionEpoch() &&
        m_gapDryRun.lease.Matches(m_programMotionLease) &&
        m_motion.IsMotionOwnerLeaseCurrent(m_gapDryRun.lease);
}

bool NCManager::IsEDMMultiAxisSimulationRecipeCurrentSameThread() const noexcept
{
    if (!m_edmProcessReady || m_recipeRecoveryDisplayOnly || m_recipeIdleSelection || m_recipeHoldSeen ||
        m_recipeRun != m_gapDryRun.run || m_recipeCache != m_gapDryRun.cache ||
        m_recipeEpoch != m_gapDryRun.epoch || !m_recipeLease.Matches(m_gapDryRun.lease)) return false;
    const auto servo = EDM74::BindPhysicalBidirectionalRecipe(m_recipe, m_edmProcessProfile, m_edmProcessTuningGeneration);
    const auto flush = EDM75::BindPhysicalBidirectionalFlushRecipe(m_recipe, m_edmProcessProfile.machineProfileId);
    return servo.ready && flush.ready && EDM75::SameAdmission(flush.recipe, servo.recipe) &&
        EDM59::SameRecipe(servo.recipe, m_edmMultiAxisSimulation.servo) &&
        EDM57::SameRecipe(flush.recipe, m_edmMultiAxisSimulation.flush);
}

void NCManager::EndEDMMultiAxisSimulationSameThread(bool failed, const char* reason) noexcept
{
    if (!m_gapDryRun.active && m_gapDryRun.result == 3U) return;
    const bool report = m_gapDryRun.active || (failed && m_gapDryRun.result != 3U);
    const auto prior = m_edmMultiAxisSimulation.suite.Report();
    m_edmMultiAxisSimulation.suite.Cancel();
    m_gapDryRun.active = m_gapDryRun.paused = false;
    m_gapDryRun.result = failed ? 3U : 2U;
    if (report)
        RtPrintf("[EDM81] event=SUMMARY result=%s reason=%.64s P=%u test=%llu cases=%u checks=%u virtualTicks=%u resetRequired=1 source=SIM output=NONE motion=0 physicalPermit=0 discharge=0\n",
            failed ? "FAIL" : "CANCELLED", reason ? reason : "UNKNOWN", m_edmMultiAxisSimulation.profile,
            static_cast<unsigned long long>(m_gapDryRun.test), prior.casesPassed, prior.checks, prior.virtualTicks);
}

void NCManager::ValidateEDMMultiAxisSimulationSameThread()
{
    if (!m_gapDryRun.active) return;
    if (m_state == NCState::HOLD || m_edmState == EDMState::HOLD)
    { EndEDMMultiAxisSimulationSameThread(false, "HOLD_RESET_REQUIRED"); return; }
    if (!IsEDMMultiAxisSimulationScopeCurrentSameThread())
    { EndEDMMultiAxisSimulationSameThread(false, "NC_SCOPE_REVOKED"); return; }
    if (!IsEDMMultiAxisSimulationRecipeCurrentSameThread())
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM81_LIVE_RECIPE_CHANGED"); return; }
    auto& control = m_edmMultiAxisSimulation;
    std::uint64_t nowUs = 0ULL;
    if (!ReadEDMGapServoClockSameThread(nowUs) || nowUs < control.lastClockUs || nowUs < control.startedUs)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM81_CLOCK_INVALID"); return; }
    // This is a permissive NC supervisor watchdog, not a 250 us RT claim.
    if (nowUs - control.lastServiceUs > 1000000ULL)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM81_NC_SERVICE_GAP"); return; }
    if (nowUs == control.lastClockUs) ++control.stalledPolls;
    else { control.lastClockUs = nowUs; control.stalledPolls = 0U; }
    if (control.stalledPolls >= 10000U || nowUs - control.startedUs >= control.deadlineUs)
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine,
            control.stalledPolls >= 10000U ? "EDM81_CLOCK_STALLED" : "EDM81_WINDOW_TIMEOUT");
}

bool NCManager::ProcessEDMMultiAxisSimulationSameThread()
{
    ValidateEDMMultiAxisSimulationSameThread();
    if (!m_gapDryRun.active)
    {
        if (m_gapDryRun.result == 1U && m_edmMultiAxisSimulation.suite.Report().complete &&
            m_edmMultiAxisSimulation.suite.Report().passed && IsEDMMultiAxisSimulationScopeCurrentSameThread() &&
            IsEDMMultiAxisSimulationRecipeCurrentSameThread()) return true;
        if ((m_gapDryRun.result == 1U || m_gapDryRun.result == 2U) && m_state == NCState::RUN &&
            !AlarmManager::GetInstance().HasAlarm() && m_resetContinuationPhase == ResetContinuationPhase::IDLE)
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM81_CANCELLED_CALLBACK_RESET_REQUIRED");
        return false;
    }
    auto& control = m_edmMultiAxisSimulation;
    // The suite advances a fixed maximum of 128 virtual steps per callback.
    // Never catch up against wall time and never touch an axis command path.
    const auto& report = control.suite.Advance();
    if (report.complete && !report.passed)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine,
        report.failedCase ? report.failedCase : "EDM81_SUITE_FAILED"); return false; }
    std::uint64_t nowUs = 0ULL;
    if (!ReadEDMGapServoClockSameThread(nowUs) || nowUs < control.lastClockUs || nowUs < control.startedUs)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM81_CLOCK_INVALID"); return false; }
    control.lastClockUs = control.lastServiceUs = nowUs;
    if (!report.complete || nowUs - control.startedUs < control.minimumUs) return false;
    if (nowUs - control.startedUs >= control.deadlineUs || !IsEDMMultiAxisSimulationScopeCurrentSameThread() ||
        !IsEDMMultiAxisSimulationRecipeCurrentSameThread())
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM81_FINAL_SCOPE_REVOKED"); return false; }
    m_gapDryRun.active = false; m_gapDryRun.result = 1U;
    RtPrintf("[EDM81] event=SUMMARY result=PASS P=%u test=%llu cases=%u checks=%u virtualTicks=%u observedMs=%llu source=SIM clock=VIRTUAL_250US executor=NC_10MS output=NONE rtxTimingVerified=0 motion=0 physicalPermit=0 discharge=0\n",
        control.profile, static_cast<unsigned long long>(m_gapDryRun.test), report.casesPassed, report.checks,
        report.virtualTicks, static_cast<unsigned long long>((nowUs - control.startedUs) / 1000ULL));
    return true;
}

// EDM82 is a detached NC-owned simulation. Virtual 250 us steps are bounded
// calculation batches at NC service cadence, never EtherCAT timing evidence.
WaitConditionFunc NCManager::StartEDMCNCRetainedSimulationSameThread(const NCBlock& block)
{
    const int line = m_gapDryRun.sourceLine;
    if (m_gapDryRun.active || m_edmZFixture.cleanupPending || m_edmZFixture.resetPending ||
        m_edmZFixture.failureAlarmPending || m_edmPhysicalServoRT.retirePending || m_edmGapServoRT.retirePending)
    { RejectEDMGapAdmissionSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM82_PREVIOUS_SESSION_BUSY"); return nullptr; }
    const unsigned profile = block.has('P') && block.val('P') == 66.0 ? 66U : 65U;
    const int pc = GetActiveDispatchPC();
    const NCProgramCacheLine* authored = pc >= 0 ? GetBaseProgramCache().TryGetLine(pc) : nullptr;
    bool literal = authored && authored->sourceLineNumber == line &&
        EDM77::Detail::Clean(authored->parsedBlock) && !authored->parsedBlock.isEmpty &&
        authored->parsedBlock.gCount == 1 && authored->parsedBlock.mCount == 0 &&
        EDM77::Detail::EqualLiteral(authored->parsedBlock.gExpressions[0], 180U) &&
        EDM77::Detail::OnlyAddresses(authored->parsedBlock, "NP") &&
        authored->parsedBlock.has('P') &&
        EDM77::Detail::EqualLiteral(authored->parsedBlock.expression('P'), profile);
    if (literal)
    {
        for (char letter = 'A'; letter <= 'Z'; ++letter)
        {
            if (block.has(letter) != authored->parsedBlock.has(letter)) { literal = false; break; }
            if (!block.has(letter)) continue;
            std::uint32_t value = 0U;
            if (!EDM77::Detail::UnsignedLiteral(authored->parsedBlock.expression(letter),
                (std::numeric_limits<std::uint32_t>::max)(), value) || block.val(letter) != static_cast<double>(value))
            { literal = false; break; }
        }
    }
    if (!IsGapDryRunBlockShapeValid(block) || block.isBlockSkip || !literal || !IsEDMZFixtureM30ContinuationSameThread())
    { RejectEDMGapAdmissionSameThread(AlarmManager::G_Code_Invalid_parameter, line, "EDM82_LITERAL_FIXTURE_M30_REQUIRED"); return nullptr; }
    std::uint64_t mask = 0ULL;
    (void)IsEDMGapServoRunReadySameThread(&mask);
    // Retain owner/epoch, quiescent queues and interlocks. The axis Arm proof
    // is deliberately irrelevant: this fixture has no Motion/PDO consumer.
    if ((mask & ~(1ULL << 9U)) || !m_edmProcessReady ||
        m_currentExecutingBlockDispatchId == NC_BLOCK_DISPATCH_ID_INVALID ||
        m_motion.GetCurrentExecutionEpoch() == MOTION_EXECUTION_EPOCH_INVALID ||
        !m_pathCoreLiveBookkeeping.currentRunToken || !GetBaseProgramCache().GetGeneration() ||
        m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)())
    { RejectEDMGapAdmissionSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM82_NOT_READY"); return nullptr; }
    EDMRecipe::Summary summary{};
    if (!ReadEDMRecipeSummarySameThread(summary))
    { RejectEDMGapAdmissionSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM82_RECIPE_SCOPE"); return nullptr; }
    const auto servo = EDM74::BindPhysicalBidirectionalRecipe(m_recipe, m_edmProcessProfile, m_edmProcessTuningGeneration);
    const auto flush = EDM75::BindPhysicalBidirectionalFlushRecipe(m_recipe, m_edmProcessProfile.machineProfileId);
    if (!servo.ready || !flush.ready || !EDM75::SameAdmission(flush.recipe, servo.recipe))
    { RejectEDMGapAdmissionSameThread(AlarmManager::G_Code_Invalid_parameter, line, "EDM82_RECIPE_BIND_REJECTED"); return nullptr; }
    LARGE_INTEGER frequency{};
    if (!RtQueryPerformanceFrequency(&frequency) || frequency.QuadPart <= 0 ||
        static_cast<std::uint64_t>(frequency.QuadPart) > (std::numeric_limits<std::uint64_t>::max)() / 1000000ULL)
    { RejectEDMGapAdmissionSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM82_CLOCK_FREQUENCY"); return nullptr; }
    // A cancelled earlier Q2 fixture cannot lend its retained mode to this
    // detached diagnostic in a fresh run. All physical busy guards passed.
    InvalidateEDMPhysicalSequencePlanSameThread();
    m_edmGapDynamic.strictSequenceMode = false;
    m_gapDryRun = GapDryRunState{};
    m_gapDryRun.cncRetainedSimulationTest = true;
    m_gapDryRun.sourceLine = line;
    m_gapDryRun.test = ++m_gapDryRunSerial;
    m_gapDryRun.run = m_pathCoreLiveBookkeeping.currentRunToken;
    m_gapDryRun.cache = GetBaseProgramCache().GetGeneration();
    m_gapDryRun.dispatch = m_currentExecutingBlockDispatchId;
    m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch();
    m_gapDryRun.lease = m_programMotionLease;
    m_gapDryRun.frequency = static_cast<std::uint64_t>(frequency.QuadPart);
    auto& control = m_edmCNCRetainedSimulation;
    control.profile = profile; control.sourcePC = pc;
    control.servo = servo.recipe; control.flush = flush.recipe;
    control.startedUs = control.lastClockUs = control.lastServiceUs = 0ULL;
    control.stalledPolls = 0U; control.clockStarted = false;
    control.minimumUs = profile == 66U ? 20000000ULL : 1000000ULL;
    control.deadlineUs = profile == 66U ? 35000000ULL : 30000000ULL;
    m_gapDryRun.active = true;
    if (!control.suite.Start(control.servo.curve, control.servo.referenceV, control.servo.overridePercent))
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, line, "EDM82_SUITE_START_REJECTED"); return nullptr; }
    RtPrintf("[EDM82] event=BEGIN build=EDM82_FIX1_20261011 P=%u test=%llu run=%llu dispatch=%llu windowMs=%llu source=SIM clock=VIRTUAL_250US executor=NC_10MS output=NONE axes=XYZCUV rotary=HELD geometry=CNC_RETAINED_LINEAR_ARC plan=DIAGNOSTIC_PRIVATE livePath=0 hold=CANCEL_RESET_REQUIRED motion=0 physicalPermit=0 discharge=0\n",
        profile, static_cast<unsigned long long>(m_gapDryRun.test),
        static_cast<unsigned long long>(m_gapDryRun.run), static_cast<unsigned long long>(m_gapDryRun.dispatch),
        static_cast<unsigned long long>(control.minimumUs / 1000ULL));
    char recipeText[480]{};
    const int recipeLength = std::snprintf(recipeText, sizeof(recipeText),
        "[EDM82] event=RECIPE_FROZEN table=%u E=%u generation=%llu processRev=%u tuningGen=%llu E8=%.6f E9=%.6f curve=INSTALLED input=SYNTHETIC apply=SIMULATION_ONLY motion=0 physicalPermit=0 discharge=0\n",
        control.servo.binding.tableId, static_cast<unsigned>(control.servo.binding.eCode),
        static_cast<unsigned long long>(control.servo.binding.generation), control.servo.binding.processRevision,
        static_cast<unsigned long long>(control.servo.binding.tuningGeneration),
        control.servo.overridePercent, control.servo.referenceV);
    if (recipeLength < 0 || static_cast<std::size_t>(recipeLength) >= sizeof(recipeText))
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, line, "EDM82_RECIPE_LOG_FORMAT"); return nullptr; }
    RtPrintf("%s", recipeText);
    std::uint64_t nowUs = 0ULL;
    if (!ReadEDMGapServoClockSameThread(nowUs))
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, line, "EDM82_CLOCK_READ"); return nullptr; }
    control.startedUs = control.lastClockUs = control.lastServiceUs = nowUs;
    control.clockStarted = true;
    return WaitForGapDryRunCallback;
}

bool NCManager::IsEDMCNCRetainedSimulationScopeCurrentSameThread() const noexcept
{
    std::uint64_t mask = 0ULL;
    (void)IsEDMGapServoRunReadySameThread(&mask);
    return m_gapDryRun.cncRetainedSimulationTest && !(mask & ~(1ULL << 9U)) &&
        !m_edmPhysicalServoRT.retirePending && !m_edmGapServoRT.retirePending &&
        m_gapDryRun.run != 0ULL && m_gapDryRun.run == m_pathCoreLiveBookkeeping.currentRunToken &&
        m_gapDryRun.cache != 0ULL && m_gapDryRun.cache == GetBaseProgramCache().GetGeneration() &&
        m_gapDryRun.dispatch != NC_BLOCK_DISPATCH_ID_INVALID && m_gapDryRun.dispatch == m_waitingBlockDispatchId &&
        m_gapDryRun.epoch != MOTION_EXECUTION_EPOCH_INVALID &&
        m_gapDryRun.epoch == m_motion.GetCurrentExecutionEpoch() &&
        m_gapDryRun.lease.Matches(m_programMotionLease) &&
        m_motion.IsMotionOwnerLeaseCurrent(m_gapDryRun.lease);
}

bool NCManager::IsEDMCNCRetainedSimulationRecipeCurrentSameThread() const noexcept
{
    if (!m_edmProcessReady || m_recipeRecoveryDisplayOnly || m_recipeIdleSelection || m_recipeHoldSeen ||
        m_recipeRun != m_gapDryRun.run || m_recipeCache != m_gapDryRun.cache ||
        m_recipeEpoch != m_gapDryRun.epoch || !m_recipeLease.Matches(m_gapDryRun.lease)) return false;
    const auto servo = EDM74::BindPhysicalBidirectionalRecipe(m_recipe, m_edmProcessProfile, m_edmProcessTuningGeneration);
    const auto flush = EDM75::BindPhysicalBidirectionalFlushRecipe(m_recipe, m_edmProcessProfile.machineProfileId);
    return servo.ready && flush.ready && EDM75::SameAdmission(flush.recipe, servo.recipe) &&
        EDM59::SameRecipe(servo.recipe, m_edmCNCRetainedSimulation.servo) &&
        EDM57::SameRecipe(flush.recipe, m_edmCNCRetainedSimulation.flush);
}

void NCManager::EndEDMCNCRetainedSimulationSameThread(bool failed, const char* reason) noexcept
{
    if (!m_gapDryRun.active && m_gapDryRun.result == 3U) return;
    const bool report = m_gapDryRun.active || (failed && m_gapDryRun.result != 3U);
    const auto& control = m_edmCNCRetainedSimulation;
    std::uint64_t cancelledUs = 0ULL;
    const bool clockValid = control.clockStarted && ReadEDMGapServoClockSameThread(cancelledUs) &&
        cancelledUs >= control.startedUs && cancelledUs >= control.lastClockUs;
    const std::uint64_t observedMs = clockValid ? (cancelledUs - control.startedUs) / 1000ULL : 0ULL;
    const auto prior = m_edmCNCRetainedSimulation.suite.Report();
    m_edmCNCRetainedSimulation.suite.Cancel();
    m_gapDryRun.active = m_gapDryRun.paused = false;
    m_gapDryRun.result = failed ? 3U : 2U;
    if (report)
        RtPrintf("[EDM82] event=SUMMARY result=%s reason=%.64s P=%u test=%llu cases=%u checks=%u virtualTicks=%u observedMs=%llu clockValid=%u resetRequired=1 source=SIM output=NONE motion=0 physicalPermit=0 discharge=0\n",
            failed ? "FAIL" : "CANCELLED", reason ? reason : "UNKNOWN", m_edmCNCRetainedSimulation.profile,
            static_cast<unsigned long long>(m_gapDryRun.test), prior.casesPassed, prior.checks, prior.virtualTicks,
            static_cast<unsigned long long>(observedMs), clockValid ? 1U : 0U);
}

void NCManager::ValidateEDMCNCRetainedSimulationSameThread()
{
    if (!m_gapDryRun.active) return;
    if (m_state == NCState::HOLD || m_edmState == EDMState::HOLD)
    { EndEDMCNCRetainedSimulationSameThread(false, "HOLD_RESET_REQUIRED"); return; }
    if (!IsEDMCNCRetainedSimulationScopeCurrentSameThread())
    { EndEDMCNCRetainedSimulationSameThread(false, "NC_SCOPE_REVOKED"); return; }
    if (!IsEDMCNCRetainedSimulationRecipeCurrentSameThread())
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM82_LIVE_RECIPE_CHANGED"); return; }
    auto& control = m_edmCNCRetainedSimulation;
    std::uint64_t nowUs = 0ULL;
    if (!ReadEDMGapServoClockSameThread(nowUs) || nowUs < control.lastClockUs || nowUs < control.startedUs)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM82_CLOCK_INVALID"); return; }
    // This is a permissive NC supervisor watchdog, not a 250 us RT claim.
    if (nowUs - control.lastServiceUs > 1000000ULL)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM82_NC_SERVICE_GAP"); return; }
    if (nowUs == control.lastClockUs) ++control.stalledPolls;
    else { control.lastClockUs = nowUs; control.stalledPolls = 0U; }
    if (control.stalledPolls >= 10000U || nowUs - control.startedUs >= control.deadlineUs)
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine,
            control.stalledPolls >= 10000U ? "EDM82_CLOCK_STALLED" : "EDM82_WINDOW_TIMEOUT");
}

bool NCManager::ProcessEDMCNCRetainedSimulationSameThread()
{
    ValidateEDMCNCRetainedSimulationSameThread();
    if (!m_gapDryRun.active)
    {
        if (m_gapDryRun.result == 1U && m_edmCNCRetainedSimulation.suite.Report().complete &&
            m_edmCNCRetainedSimulation.suite.Report().passed && IsEDMCNCRetainedSimulationScopeCurrentSameThread() &&
            IsEDMCNCRetainedSimulationRecipeCurrentSameThread()) return true;
        if ((m_gapDryRun.result == 1U || m_gapDryRun.result == 2U) && m_state == NCState::RUN &&
            !AlarmManager::GetInstance().HasAlarm() && m_resetContinuationPhase == ResetContinuationPhase::IDLE)
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM82_CANCELLED_CALLBACK_RESET_REQUIRED");
        return false;
    }
    auto& control = m_edmCNCRetainedSimulation;
    // The suite advances a fixed maximum of 128 virtual steps per callback.
    // Never catch up against wall time and never touch an axis command path.
    const auto& report = control.suite.Advance();
    if (report.complete && !report.passed)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine,
        report.failedCase ? report.failedCase : "EDM82_SUITE_FAILED"); return false; }
    std::uint64_t nowUs = 0ULL;
    if (!ReadEDMGapServoClockSameThread(nowUs) || nowUs < control.lastClockUs || nowUs < control.startedUs)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM82_CLOCK_INVALID"); return false; }
    control.lastClockUs = control.lastServiceUs = nowUs;
    if (!report.complete || nowUs - control.startedUs < control.minimumUs) return false;
    if (nowUs - control.startedUs >= control.deadlineUs || !IsEDMCNCRetainedSimulationScopeCurrentSameThread() ||
        !IsEDMCNCRetainedSimulationRecipeCurrentSameThread())
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM82_FINAL_SCOPE_REVOKED"); return false; }
    m_gapDryRun.active = false; m_gapDryRun.result = 1U;
    RtPrintf("[EDM82] event=SUMMARY result=PASS P=%u test=%llu cases=%u checks=%u virtualTicks=%u observedMs=%llu source=SIM clock=VIRTUAL_250US executor=NC_10MS output=NONE rtxTimingVerified=0 motion=0 physicalPermit=0 discharge=0\n",
        control.profile, static_cast<unsigned long long>(m_gapDryRun.test), report.casesPassed, report.checks,
        report.virtualTicks, static_cast<unsigned long long>((nowUs - control.startedUs) / 1000ULL));
    return true;
}

// EDM83 supervises an explicit private retained-path calculator in Motion's RT
// callback. None of these methods publishes an axis target or discharge permit.
WaitConditionFunc NCManager::StartEDMCNCRetainedRTSameThread(const NCBlock& block)
{
    const int line = m_gapDryRun.sourceLine;
    ServiceEDMCNCRetainedRTRetirementSameThread();
    ServiceEDMGapServoRetirementSameThread();
    ServiceEDMPhysicalServoRetirementSameThread();
    if (m_gapDryRun.active || m_edmCNCRetainedRT.retirePending ||
        (m_edmCNCRetainedRT.submitted && !m_edmCNCRetainedRT.retired) ||
        m_edmZFixture.cleanupPending || m_edmZFixture.resetPending || m_edmZFixture.failureAlarmPending ||
        m_edmPhysicalServoRT.retirePending || m_edmGapServoRT.retirePending)
    { RejectEDMGapAdmissionSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM83_PREVIOUS_SESSION_BUSY"); return nullptr; }
    const unsigned profile = block.has('P') && block.val('P') == 68.0 ? 68U : 67U;
    const int pc = GetActiveDispatchPC();
    const NCProgramCacheLine* authored = pc >= 0 ? GetBaseProgramCache().TryGetLine(pc) : nullptr;
    bool literal = authored && authored->sourceLineNumber == line &&
        EDM77::Detail::Clean(authored->parsedBlock) && !authored->parsedBlock.isEmpty &&
        authored->parsedBlock.gCount == 1 && authored->parsedBlock.mCount == 0 &&
        EDM77::Detail::EqualLiteral(authored->parsedBlock.gExpressions[0], 180U) &&
        EDM77::Detail::OnlyAddresses(authored->parsedBlock, "NP") && authored->parsedBlock.has('P') &&
        EDM77::Detail::EqualLiteral(authored->parsedBlock.expression('P'), profile);
    if (literal)
    {
        for (char letter = 'A'; letter <= 'Z'; ++letter)
        {
            if (block.has(letter) != authored->parsedBlock.has(letter)) { literal = false; break; }
            if (!block.has(letter)) continue;
            std::uint32_t value = 0U;
            if (!EDM77::Detail::UnsignedLiteral(authored->parsedBlock.expression(letter),
                (std::numeric_limits<std::uint32_t>::max)(), value) || block.val(letter) != static_cast<double>(value))
            { literal = false; break; }
        }
    }
    if (!IsGapDryRunBlockShapeValid(block) || block.isBlockSkip || !literal || !IsEDMZFixtureM30ContinuationSameThread())
    { RejectEDMGapAdmissionSameThread(AlarmManager::G_Code_Invalid_parameter, line, "EDM83_LITERAL_FIXTURE_M30_REQUIRED"); return nullptr; }
    std::uint64_t mask = 0ULL;
    (void)IsEDMGapServoRunReadySameThread(&mask);
    if ((mask & ~(1ULL << 9U)) || !m_edmProcessReady ||
        m_currentExecutingBlockDispatchId == NC_BLOCK_DISPATCH_ID_INVALID ||
        m_motion.GetCurrentExecutionEpoch() == MOTION_EXECUTION_EPOCH_INVALID ||
        !m_pathCoreLiveBookkeeping.currentRunToken || !GetBaseProgramCache().GetGeneration())
    { RejectEDMGapAdmissionSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM83_NOT_READY"); return nullptr; }
    EDMRecipe::Summary summary{};
    if (!ReadEDMRecipeSummarySameThread(summary))
    { RejectEDMGapAdmissionSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM83_RECIPE_SCOPE"); return nullptr; }
    const auto servo = EDM74::BindPhysicalBidirectionalRecipe(m_recipe, m_edmProcessProfile, m_edmProcessTuningGeneration);
    const auto flush = EDM75::BindPhysicalBidirectionalFlushRecipe(m_recipe, m_edmProcessProfile.machineProfileId);
    if (!servo.ready || !flush.ready || !EDM75::SameAdmission(flush.recipe, servo.recipe))
    { RejectEDMGapAdmissionSameThread(AlarmManager::G_Code_Invalid_parameter, line, "EDM83_RECIPE_BIND_REJECTED"); return nullptr; }
    LARGE_INTEGER frequency{};
    if (!RtQueryPerformanceFrequency(&frequency) || frequency.QuadPart <= 0 ||
        static_cast<std::uint64_t>(frequency.QuadPart) > (std::numeric_limits<std::uint64_t>::max)() / 1000000ULL)
    { RejectEDMGapAdmissionSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM83_CLOCK_FREQUENCY"); return nullptr; }
    const auto latest = m_motion.ReadEDMCNCRetainedRTLatestSession();
    const auto serial = latest > m_gapDryRunSerial ? latest : m_gapDryRunSerial;
    if (serial == (std::numeric_limits<std::uint64_t>::max)())
    { RejectEDMGapAdmissionSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM83_SESSION_OVERFLOW"); return nullptr; }
    m_edmCNCRetainedRT = EDMCNCRetainedRTControlState{};
    auto& control = m_edmCNCRetainedRT;
    control.servo = servo.recipe; control.flush = flush.recipe;
    auto& request = control.request;
    request.session = serial + 1ULL;
    request.profile = profile == 68U ? EDM83RT::Profile::OperatorWindow : EDM83RT::Profile::Normal;
    request.qpcFrequency = static_cast<std::uint64_t>(frequency.QuadPart);
    request.scope.run = m_pathCoreLiveBookkeeping.currentRunToken;
    request.scope.cache = GetBaseProgramCache().GetGeneration();
    request.scope.dispatch = m_currentExecutingBlockDispatchId;
    request.scope.executionEpoch = m_motion.GetCurrentExecutionEpoch();
    request.scope.ownerLease = (static_cast<std::uint64_t>(m_programMotionLease.owner) << 32U) |
        static_cast<std::uint64_t>(m_programMotionLease.generation);
    // Fixed private fixture geometry/frame/kinematics revision, not live TCP.
    request.scope.configRevision = request.scope.frameRevision = request.scope.kinematicsRevision = 1ULL;
    request.scope.recipeGeneration = servo.recipe.binding.generation;
    request.scope.sourceGeneration = request.session;
    if (!ReadEDMCNCRetainedRTClockSameThread(request.issuedUs))
    { RejectEDMGapAdmissionSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, line, "EDM83_CLOCK_READ"); return nullptr; }
    if (!EDM83RT::BuildFixtureRequest(request, servo.recipe.curve, servo.recipe.referenceV, servo.recipe.overridePercent))
    { RejectEDMGapAdmissionSameThread(AlarmManager::G_Code_Invalid_parameter, line, "EDM83_FIXTURE_BUILD"); return nullptr; }
    InvalidateEDMPhysicalSequencePlanSameThread();
    m_edmGapDynamic.strictSequenceMode = false;
    m_gapDryRun = GapDryRunState{};
    m_gapDryRun.cncRetainedRTTest = m_gapDryRun.active = true;
    m_gapDryRun.sourceLine = line; m_gapDryRun.test = request.session;
    m_gapDryRunSerial = request.session;
    m_gapDryRun.run = request.scope.run; m_gapDryRun.cache = request.scope.cache;
    m_gapDryRun.dispatch = request.scope.dispatch;
    m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
    control.lastClockUs = control.lastServiceUs = control.lastFeedbackUs = request.issuedUs;
    control.deadlineUs = profile == 68U ? 35000000ULL : 27000000ULL;
    // Mark before submission: a failing transport may already have reserved
    // the session. Cancellation and a later RT receipt are still mandatory.
    control.submitted = true; control.retired = false;
    if (!m_motion.StartEDMCNCRetainedRT(request))
    {
        // A request rejected before reservation has no RT-owned state to retire.
        if (m_motion.ReadEDMCNCRetainedRTLatestSession() < request.session)
        { control.submitted = false; control.retired = true; }
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, line, "EDM83_START_REJECTED"); return nullptr;
    }
    RtPrintf("[EDM83] event=BEGIN build=EDM83_20261011 P=%u session=%llu run=%llu dispatch=%llu source=SIM executor=RT_CALLBACK modelStepUs=250 output=NONE axes=XYZCUV rotary=HELD geometry=CNC_RETAINED_LINEAR_ARC plan=DIAGNOSTIC_PRIVATE livePath=0 hold=CANCEL_RESET_REQUIRED wcetVerified=0 motion=0 physicalPermit=0 discharge=0\n",
        profile, static_cast<unsigned long long>(request.session), static_cast<unsigned long long>(request.scope.run),
        static_cast<unsigned long long>(request.scope.dispatch));
    char text[480]{};
    const int length = std::snprintf(text, sizeof(text),
        "[EDM83] event=RECIPE_FROZEN table=%u E=%u generation=%llu processRev=%u tuningGen=%llu E8=%.6f E9=%.6f curve=INSTALLED input=SYNTHETIC output=NONE motion=0 physicalPermit=0 discharge=0\n",
        control.servo.binding.tableId, static_cast<unsigned>(control.servo.binding.eCode),
        static_cast<unsigned long long>(control.servo.binding.generation), control.servo.binding.processRevision,
        static_cast<unsigned long long>(control.servo.binding.tuningGeneration), control.servo.overridePercent, control.servo.referenceV);
    if (length < 0 || static_cast<std::size_t>(length) >= sizeof(text))
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, line, "EDM83_RECIPE_LOG_FORMAT"); return nullptr; }
    RtPrintf("%s", text);
    return WaitForGapDryRunCallback;
}

bool NCManager::ReadEDMCNCRetainedRTClockSameThread(std::uint64_t& nowUs) noexcept
{
    auto& control = m_edmCNCRetainedRT;
    const auto frequency = control.request.qpcFrequency;
    LARGE_INTEGER counter{};
    if (!frequency || frequency > (std::numeric_limits<std::uint64_t>::max)() / 1000000ULL ||
        !RtQueryPerformanceCounter(&counter) || counter.QuadPart < 0) return false;
    const auto ticks = static_cast<std::uint64_t>(counter.QuadPart);
    if (ticks < control.lastTicks) return false;
    const auto seconds = ticks / frequency;
    if (seconds > (std::numeric_limits<std::uint64_t>::max)() / 1000000ULL) return false;
    const auto fraction = (ticks % frequency) * 1000000ULL / frequency;
    if (seconds * 1000000ULL > (std::numeric_limits<std::uint64_t>::max)() - fraction) return false;
    nowUs = seconds * 1000000ULL + fraction;
    control.lastTicks = ticks;
    return true;
}

bool NCManager::IsEDMCNCRetainedRTScopeCurrentSameThread() const noexcept
{
    std::uint64_t mask = 0ULL;
    (void)IsEDMGapServoRunReadySameThread(&mask);
    return m_gapDryRun.cncRetainedRTTest && !(mask & ~(1ULL << 9U)) &&
        !m_edmPhysicalServoRT.retirePending && !m_edmGapServoRT.retirePending &&
        m_gapDryRun.run != 0ULL && m_gapDryRun.run == m_pathCoreLiveBookkeeping.currentRunToken &&
        m_gapDryRun.cache != 0ULL && m_gapDryRun.cache == GetBaseProgramCache().GetGeneration() &&
        m_gapDryRun.dispatch != NC_BLOCK_DISPATCH_ID_INVALID && m_gapDryRun.dispatch == m_waitingBlockDispatchId &&
        m_gapDryRun.epoch != MOTION_EXECUTION_EPOCH_INVALID && m_gapDryRun.epoch == m_motion.GetCurrentExecutionEpoch() &&
        m_gapDryRun.lease.Matches(m_programMotionLease) && m_motion.IsMotionOwnerLeaseCurrent(m_gapDryRun.lease);
}

bool NCManager::IsEDMCNCRetainedRTRecipeCurrentSameThread() const noexcept
{
    if (!m_edmProcessReady || m_recipeRecoveryDisplayOnly || m_recipeIdleSelection || m_recipeHoldSeen ||
        m_recipeRun != m_gapDryRun.run || m_recipeCache != m_gapDryRun.cache ||
        m_recipeEpoch != m_gapDryRun.epoch || !m_recipeLease.Matches(m_gapDryRun.lease)) return false;
    const auto servo = EDM74::BindPhysicalBidirectionalRecipe(m_recipe, m_edmProcessProfile, m_edmProcessTuningGeneration);
    const auto flush = EDM75::BindPhysicalBidirectionalFlushRecipe(m_recipe, m_edmProcessProfile.machineProfileId);
    return servo.ready && flush.ready && EDM75::SameAdmission(flush.recipe, servo.recipe) &&
        EDM59::SameRecipe(servo.recipe, m_edmCNCRetainedRT.servo) && EDM57::SameRecipe(flush.recipe, m_edmCNCRetainedRT.flush);
}

void NCManager::LogEDMCNCRetainedRTSameThread(const char* event, const char* reason) noexcept
{
    const auto& c = m_edmCNCRetainedRT;
    const auto& f = c.feedback;
    const char* result = std::strcmp(event ? event : "", "SUMMARY") == 0 ? "PASS" :
        std::strcmp(event ? event : "", "FAIL") == 0 ? "FAIL" :
        std::strcmp(event ? event : "", "CANCEL") == 0 ? "CANCELLED" : "PENDING";
    RtPrintf("[EDM83] event=%.24s result=%s reason=%.64s P=%u session=%llu state=%s rtReason=%s kernelReason=%u stage=%u pub=%llu rtTick=%llu terminalTick=%llu quiescentTick=%llu quiescent=%u output=NONE motion=0 physicalPermit=0 discharge=0\n",
        event ? event : "UNKNOWN", result, reason ? reason : "NONE", static_cast<unsigned>(c.request.profile),
        static_cast<unsigned long long>(c.request.session), EDM83RT::StateName(f.state), EDM83RT::ReasonName(f.reason),
        static_cast<unsigned>(f.kernelReason), static_cast<unsigned>(f.stage), static_cast<unsigned long long>(f.publication), static_cast<unsigned long long>(f.rtTick),
        static_cast<unsigned long long>(f.terminalTick), static_cast<unsigned long long>(f.quiescentTick), f.quiescent ? 1U : 0U);
}

void NCManager::EndEDMCNCRetainedRTSameThread(bool failed, const char* reason) noexcept
{
    auto& c = m_edmCNCRetainedRT;
    const bool report = m_gapDryRun.cncRetainedRTTest &&
        (m_gapDryRun.active || m_gapDryRun.result == 1U || (failed && m_gapDryRun.result == 2U));
    if (c.submitted && !c.retired)
    {
        // Publish the exact-session cancellation before any console work.
        m_motion.CancelEDMCNCRetainedRT(c.request.session);
        if (!c.retirePending)
        {
            c.cancelPublication = c.feedback.publication; c.cancelTick = c.feedback.rtTick;
            c.cancelUs = c.lastClockUs; c.retirePending = true;
        }
    }
    c.completionPending = false;
    if (m_gapDryRun.cncRetainedRTTest)
    {
        m_gapDryRun.active = m_gapDryRun.paused = false;
        if (report) m_gapDryRun.result = failed ? 3U : 2U;
    }
    if (report) LogEDMCNCRetainedRTSameThread(failed ? "FAIL" : "CANCEL", reason);
}

void NCManager::ServiceEDMCNCRetainedRTRetirementSameThread() noexcept
{
    auto& c = m_edmCNCRetainedRT;
    if (!c.retirePending) return;
    EDM83RT::Feedback receipt{};
    if (m_motion.ReadEDMCNCRetainedRT(receipt) && receipt.session == c.request.session &&
        receipt.latestSession == c.request.session && receipt.profile == c.request.profile &&
        EDM81::SameScope(receipt.scope, c.request.scope) && receipt.cancelThroughSession >= c.request.session &&
        receipt.publication > c.cancelPublication && receipt.rtTick > c.cancelTick &&
        receipt.state == EDM83RT::State::Cancelled && receipt.quiescent &&
        receipt.terminalTick > c.cancelTick && receipt.quiescentTick > receipt.terminalTick &&
        receipt.rtTick >= receipt.quiescentTick)
    {
        c.feedback = receipt; c.retired = true; c.retirePending = false;
        LogEDMCNCRetainedRTSameThread("RETIRED", "EXACT_SESSION_RT_QUIESCENT");
        return;
    }
    std::uint64_t nowUs = 0ULL;
    if (!c.retirementWarning && ReadEDMCNCRetainedRTClockSameThread(nowUs) &&
        nowUs >= c.cancelUs && nowUs - c.cancelUs >= 2000000ULL)
    {
        c.retirementWarning = true;
        LogEDMCNCRetainedRTSameThread("RETIRE_WAIT", "NO_VALID_RT_QUIESCENT_ACK_START_BLOCKED");
    }
}

void NCManager::ValidateEDMCNCRetainedRTSameThread()
{
    auto& c = m_edmCNCRetainedRT;
    if (!m_gapDryRun.active) return;
    if (m_state == NCState::HOLD || m_edmState == EDMState::HOLD)
    { EndEDMCNCRetainedRTSameThread(false, "HOLD_RESET_REQUIRED"); return; }
    if (!IsEDMCNCRetainedRTScopeCurrentSameThread())
    { EndEDMCNCRetainedRTSameThread(false, "NC_SCOPE_REVOKED"); return; }
    if (!IsEDMCNCRetainedRTRecipeCurrentSameThread())
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM83_LIVE_RECIPE_CHANGED"); return; }
    std::uint64_t nowUs = 0ULL;
    if (!ReadEDMCNCRetainedRTClockSameThread(nowUs) || nowUs < c.lastClockUs || nowUs < c.request.issuedUs)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM83_CLOCK_INVALID"); return; }
    if (nowUs - c.lastServiceUs > 100000ULL)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM83_NC_SERVICE_GAP"); return; }
    if (nowUs == c.lastClockUs) ++c.stalledPolls;
    else { c.lastClockUs = nowUs; c.stalledPolls = 0U; }
    if (c.stalledPolls >= 10000U || nowUs - c.request.issuedUs >= c.deadlineUs)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine,
        c.stalledPolls >= 10000U ? "EDM83_CLOCK_STALLED" : "EDM83_WINDOW_TIMEOUT"); return; }
    c.lastServiceUs = nowUs;
    if (!c.completionPending && !m_motion.TouchEDMCNCRetainedRT(c.request.session, nowUs))
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM83_HEARTBEAT_REJECTED");
}

bool NCManager::ProcessEDMCNCRetainedRTSameThread()
{
    ServiceEDMCNCRetainedRTRetirementSameThread();
    ValidateEDMCNCRetainedRTSameThread();
    auto& c = m_edmCNCRetainedRT;
    if (!m_gapDryRun.active)
    {
        if (m_gapDryRun.result == 1U && c.retired && IsEDMCNCRetainedRTScopeCurrentSameThread() &&
            IsEDMCNCRetainedRTRecipeCurrentSameThread()) return true;
        if ((m_gapDryRun.result == 1U || m_gapDryRun.result == 2U) && m_state == NCState::RUN &&
            !AlarmManager::GetInstance().HasAlarm() && m_resetContinuationPhase == ResetContinuationPhase::IDLE)
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM83_CANCELLED_CALLBACK_RESET_REQUIRED");
        return false;
    }
    if (c.completionPending)
    {
        if (!c.retired) return false;
        // Scope/recipe were rechecked above AFTER exact-session retirement.
        m_gapDryRun.active = false; m_gapDryRun.result = 1U;
        LogEDMCNCRetainedRTSameThread("SUMMARY", "PASS");
        RtPrintf("[EDM83] event=TIMING result=PASS session=%llu callbackSteps=%llu modelUs=%llu observedUs=%llu minCallbackUs=%llu maxCallbackUs=%llu coverage=%llu clock=QPC modelStepUs=250 wcetVerified=0 output=NONE motion=0 physicalPermit=0 discharge=0\n",
            static_cast<unsigned long long>(c.request.session), static_cast<unsigned long long>(c.completedSteps),
            static_cast<unsigned long long>(c.completedModelUs), static_cast<unsigned long long>(c.lastClockUs - c.request.issuedUs),
            static_cast<unsigned long long>(c.feedback.minimumStepUs), static_cast<unsigned long long>(c.feedback.maximumStepUs),
            static_cast<unsigned long long>(c.completedCoverage));
        return true;
    }
    EDM83RT::Feedback receipt{};
    if (!m_motion.ReadEDMCNCRetainedRT(receipt) || !receipt.publication || !receipt.session)
    {
        if (c.lastClockUs - c.lastFeedbackUs >= 1000000ULL)
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM83_RT_RECEIPT_TIMEOUT");
        return false;
    }
    // A new asynchronous request can initially read the previous session's
    // terminal bank. It grants no credit and is bounded by the start timeout.
    if (!c.lastPublication && receipt.session < c.request.session)
    {
        if (c.lastClockUs - c.lastFeedbackUs >= 1000000ULL)
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM83_RT_START_ACK_TIMEOUT");
        return false;
    }
    if (receipt.session != c.request.session || receipt.latestSession != c.request.session ||
        receipt.profile != c.request.profile || !EDM81::SameScope(receipt.scope, c.request.scope) ||
        receipt.cancelThroughSession >= c.request.session || receipt.publication < c.lastPublication || receipt.rtTick < c.lastRTTick)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM83_RT_RECEIPT_SCOPE"); return false; }
    // Read the NC clock AFTER copying the RT bank: normal RT preemption may
    // publish a sample newer than Validate's earlier QPC reading.
    std::uint64_t observedUs = 0ULL;
    if (!ReadEDMCNCRetainedRTClockSameThread(observedUs) || observedUs < c.lastClockUs ||
        observedUs - c.lastServiceUs > 100000ULL || observedUs - c.request.issuedUs >= c.deadlineUs ||
        receipt.qpcFrequency != c.request.qpcFrequency || !receipt.nowUs ||
        receipt.nowUs > observedUs || observedUs - receipt.nowUs > 100000ULL)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM83_RT_RECEIPT_CLOCK"); return false; }
    c.lastClockUs = observedUs;
    c.feedback = receipt;
    if (receipt.publication > c.lastPublication && receipt.rtTick > c.lastRTTick)
    { c.lastFeedbackUs = c.lastClockUs; c.lastPublication = receipt.publication; c.lastRTTick = receipt.rtTick; }
    else if (c.lastClockUs - c.lastFeedbackUs >= 1000000ULL)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM83_RT_RECEIPT_STALLED"); return false; }
    if (receipt.state == EDM83RT::State::Fault || receipt.state == EDM83RT::State::Cancelled)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, EDM83RT::ReasonName(receipt.reason)); return false; }
    if (receipt.state != EDM83RT::State::Completed || !receipt.startAcknowledged || !receipt.quiescent ||
        !receipt.terminalTick || receipt.quiescentTick <= receipt.terminalTick || receipt.rtTick < receipt.quiescentTick)
        return false;
    if (receipt.reason != EDM83RT::Reason::None || receipt.coverageMask != EDM83RT::RequiredCoverage ||
        !receipt.runtimeSteps || receipt.modelUs < EDM83RT::DurationUs(c.request.profile))
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM83_COMPLETION_EVIDENCE"); return false; }
    if (!c.completedPublication)
    {
        c.completedPublication = receipt.publication; c.completedTick = receipt.rtTick;
        c.completedTerminalTick = receipt.terminalTick; c.completedQuiescentTick = receipt.quiescentTick;
        c.completedSteps = receipt.runtimeSteps;
        c.completedModelUs = receipt.modelUs; c.completedCoverage = receipt.coverageMask;
        LogEDMCNCRetainedRTSameThread("COMPLETE_OBSERVED", "WAIT_SECOND_PUBLICATION");
        return false;
    }
    if (receipt.publication <= c.completedPublication || receipt.rtTick <= c.completedTick) return false;
    if (receipt.terminalTick != c.completedTerminalTick || receipt.quiescentTick != c.completedQuiescentTick ||
        receipt.runtimeSteps != c.completedSteps ||
        receipt.modelUs != c.completedModelUs || receipt.coverageMask != c.completedCoverage ||
        !IsEDMCNCRetainedRTScopeCurrentSameThread() || !IsEDMCNCRetainedRTRecipeCurrentSameThread())
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM83_TERMINAL_RECEIPT_CHANGED"); return false; }
    // Even PASS retires the exact RT session. M30 cannot consume this wait
    // until a later valid callback proves the cancelled calculator quiescent.
    m_motion.CancelEDMCNCRetainedRT(c.request.session);
    c.cancelPublication = receipt.publication; c.cancelTick = receipt.rtTick; c.cancelUs = c.lastClockUs;
    c.retirePending = c.completionPending = true;
    LogEDMCNCRetainedRTSameThread("RETIRE_REQUEST", "COMPLETED_TWO_PUBLICATIONS");
    return false;
}

// EDM87 installation is a boot-only, single attempt. Failure disables only
// explicit Q4 R selection; old Q4 and all other CNC/EDM routes remain usable.
bool NCManager::IsEDMCoordinatorAuthorityCurrentSameThread() noexcept
{
    return IsEDMShortFlushRunReadySameThread() &&
        m_gapDryRun.run == m_pathCoreLiveBookkeeping.currentRunToken &&
        m_gapDryRun.cache == GetBaseProgramCache().GetGeneration() &&
        m_gapDryRun.dispatch == m_waitingBlockDispatchId &&
        m_gapDryRun.epoch == m_motion.GetCurrentExecutionEpoch() &&
        m_gapDryRun.lease.Matches(m_programMotionLease) &&
        m_motion.IsMotionOwnerLeaseCurrent(m_gapDryRun.lease) &&
        !m_isG66Active && m_macroStack.empty() &&
        !m_motion.HasPendingSafetyOrRecoveryRequests() && m_motion.IsGroupDone() &&
        m_motion.GetCommandIngressSize() == 0U && m_motion.GetCommandReplaySize() == 0U &&
        !m_pathFeed.pending && !m_pathArc.pending && !m_pathReplay.pending &&
        !m_pathHold.armed && !m_pathHold.bound;
}

// P12 is an isolated intent/receipt contract. Nothing here submits an axis
// command or treats the live group's idle flag as physical stop evidence.
EDM27::Scope NCManager::MakeEDMMotionReplayScopeSameThread() const noexcept
{
    EDM27::Scope scope{};
    scope.session = m_gapDryRun.test;
    scope.ownerLease = (static_cast<std::uint64_t>(m_gapDryRun.lease.owner) << 32U) |
        static_cast<std::uint64_t>(m_gapDryRun.lease.generation);
    scope.executionEpoch = m_gapDryRun.epoch;
    scope.runGeneration = m_gapDryRun.run;
    scope.dispatchGeneration = m_gapDryRun.dispatch;
    // These generations identify the independent fixture, never installed
    // axis mapping/configuration. Actual NC cache remains an outer fence.
    scope.axisMapGeneration = 1ULL;
    scope.configGeneration = 1ULL;
    return scope;
}

bool NCManager::RestartEDMMotionReplaySameThread(std::uint64_t nowMs)
{
    if (!m_gapDryRun.lease.IsValid() || m_gapDryRun.cache == 0ULL ||
        !EDM27::ValidScope(MakeEDMMotionReplayScopeSameThread()))
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM27_SCOPE_INVALID"); return false; }
    m_edmMotionReplay.Reset(MakeEDMMotionReplayScopeSameThread());
    m_edmMotionReplayClockMs = m_edmMotionReplayLogMs = nowMs;
    m_edmMotionReplayLoggedAction = m_edmMotionReplayLoggedAdmission = 255U;
    m_edmMotionReplayReceiptPending = false;
    m_edmMotionReplayPreflightPending = true;
    m_edmMotionReplayReceiptStalledCalls = 0U;
    char line[512]{};
    const int length = std::snprintf(line, sizeof(line),
        "[EDM27] event=BEGIN test=%llu run=%llu dispatch=%llu line=%d restart=%u cases=%u caseMs=400 minMs=10000 maxMs=12000 maxServiceMs=25 axis=Z fixture=INDEPENDENT source=SIMULATED mode=INTENT_SHADOW motion=0 physicalPermit=0 discharge=0\n",
        static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(m_gapDryRun.run),
        static_cast<unsigned long long>(m_gapDryRun.dispatch), m_gapDryRun.sourceLine,
        static_cast<unsigned>(m_gapDryRun.restarts), static_cast<unsigned>(EDMMotionReplaySimulationTest::CaseCount));
    if (length < 0 || static_cast<std::size_t>(length) >= sizeof(line))
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM27_LOG_FORMAT"); return false; }
    RtPrintf("%s", line);
    return true;
}

void NCManager::LogEDMMotionReplayPreflightSameThread()
{
    // Only boot-installed axis configuration is read. Mutable AxisContext
    // position/velocity/isHomed are not coherent feedback or homing proof.
    bool present = false, linear = false, conversion = false;
    unsigned softMask = 0U;
    if ((m_edmProcessActiveAxisMask & (1U << 2U)) != 0U)
    {
        const auto& axis = m_motion.GetAxisContext(2);
        present = axis.isExist;
        linear = axis.axisType == AxisType::LINEAR;
        conversion = std::isfinite(axis.resolution_PPR) && axis.resolution_PPR > 0.0 &&
            std::isfinite(axis.finalLead) && axis.finalLead > 0.0;
        if (conversion) conversion = std::isfinite(axis.resolution_PPR / axis.finalLead) &&
            axis.resolution_PPR / axis.finalLead > 0.0;
        softMask = (axis.travelLimit1Enable ? 1U : 0U) | (axis.travelLimit2Enable ? 2U : 0U) |
            (axis.travelLimit3Enable ? 4U : 0U);
    }
    char line[512]{};
    const int length = std::snprintf(line, sizeof(line),
        "[EDM27] event=PREFLIGHT result=BLOCKED reason=STAGE_OUTPUT_DISABLED test=%llu profileMode=SHADOW_ONLY limitsConfirmed=%u zPresent=%u zLinear=%u nativeConversion=%u softEnabledMask=%u homeProof=UNAVAILABLE feedbackProof=UNAVAILABLE facts=INSTALLED_CONFIG motion=0 physicalPermit=0 discharge=0\n",
        static_cast<unsigned long long>(m_gapDryRun.test), m_edmProcessProfile.servo.limitsConfirmed ? 1U : 0U,
        present ? 1U : 0U, linear ? 1U : 0U, conversion ? 1U : 0U, softMask);
    if (length < 0 || static_cast<std::size_t>(length) >= sizeof(line))
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM27_LOG_FORMAT"); return; }
    RtPrintf("%s", line);
}

void NCManager::EndEDMMotionReplaySameThread(const char* result, const char* reason, std::uint64_t nowMs)
{
    const auto snapshot = m_edmMotionReplay.Current();
    const bool pass = result != nullptr && std::strcmp(result, "PASS") == 0;
    m_gapDryRun.active = m_gapDryRun.paused = false;
    m_gapDryRun.result = pass ? 1U : (result != nullptr && std::strcmp(result, "CANCELLED") == 0 ? 2U : 3U);
    m_edmMotionReplayPreflightPending = false;
    m_edmMotionReplayReceiptPending = pass;
    m_edmMotionReplayReceiptStartMs = m_edmMotionReplayReceiptClockMs = nowMs;
    m_edmMotionReplayReceiptStalledCalls = 0U;
    m_edmMotionReplay.Revoke();
    const std::uint64_t wallMs = nowMs >= m_gapDryRun.phaseStartMs ? nowMs - m_gapDryRun.phaseStartMs : 0ULL;
    char line[512]{};
    const int length = std::snprintf(line, sizeof(line),
        "[EDM27] event=SUMMARY result=%.10s reason=%.40s test=%llu run=%llu dispatch=%llu restart=%u wallMs=%llu cases=%u caseMask=%08X virtualCycles=%llu workMs=%llu machiningMm=%.9g flushMm=%.9g ack=%u stopProof=%u returnProof=%u proof=SYNTHETIC motion=0 physicalPermit=0 discharge=0\n",
        result ? result : "FAIL", reason ? reason : "UNKNOWN", static_cast<unsigned long long>(m_gapDryRun.test),
        static_cast<unsigned long long>(m_gapDryRun.run), static_cast<unsigned long long>(m_gapDryRun.dispatch),
        static_cast<unsigned>(m_gapDryRun.restarts), static_cast<unsigned long long>(wallMs),
        static_cast<unsigned>(m_gapDryRun.phase), static_cast<unsigned>(m_gapDryRun.passedMask),
        static_cast<unsigned long long>(snapshot.completedCycles), static_cast<unsigned long long>(snapshot.workElapsedMs),
        snapshot.machiningOffsetMm, snapshot.flushOffsetMm, snapshot.acknowledged ? 1U : 0U,
        snapshot.standstillProven ? 1U : 0U, snapshot.returnProven ? 1U : 0U);
    if (length < 0 || static_cast<std::size_t>(length) >= sizeof(line))
    {
        m_gapDryRun.result = 3U; m_edmMotionReplayReceiptPending = false;
        RtPrintf("%s", "[EDM27] event=SUMMARY result=FAIL reason=LOG_FORMAT motion=0 physicalPermit=0 discharge=0\n");
    }
    else RtPrintf("%s", line);
}

bool NCManager::ProcessEDMMotionReplaySameThread(std::uint64_t nowMs)
{
    if (!IsEDMCoordinatorAuthorityCurrentSameThread())
    { CancelGapDryRunSameThread("EDM27_AUTHORITY_CHANGED"); return false; }
    if (nowMs - m_edmMotionReplayClockMs > EDMMotionReplaySimulationTest::MaximumReplayServiceGapMs)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM27_REPLAY_SERVICE_GAP"); return false; }
    if (nowMs == m_edmMotionReplayClockMs)
    {
        if (++m_gapDryRun.observations >= 4096U)
        { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM27_CLOCK_NOT_ADVANCING"); return false; }
    }
    else { m_gapDryRun.observations = 0U; m_edmMotionReplayClockMs = nowMs; }
    const std::uint64_t elapsedMs = nowMs - m_gapDryRun.phaseStartMs;
    m_edmMotionReplay.Tick(elapsedMs);
    const auto& snapshot = m_edmMotionReplay.Current();
    if (!EDM27::SameScope(snapshot.scope, MakeEDMMotionReplayScopeSameThread()) ||
        snapshot.PhysicalMotionEnabled() || snapshot.PhysicalDischargeEnabled() ||
        snapshot.admissionState == static_cast<std::uint32_t>(EDM27::AdmissionState::Cancelled) ||
        snapshot.admissionState == static_cast<std::uint32_t>(EDM27::AdmissionState::Fault) ||
        snapshot.state == static_cast<std::uint32_t>(EDM25::ProcessState::Cancelled) ||
        snapshot.state == static_cast<std::uint32_t>(EDM25::ProcessState::Fault) ||
        !std::isfinite(snapshot.machiningOffsetMm) || !std::isfinite(snapshot.flushOffsetMm) ||
        !std::isfinite(snapshot.speedMmMin) || !std::isfinite(snapshot.stopReserveMm))
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM27_REPLAY_INVALID"); return false; }
    if (m_edmMotionReplayPreflightPending)
    { m_edmMotionReplayPreflightPending = false; LogEDMMotionReplayPreflightSameThread(); return false; }
    if (m_gapDryRun.phase < EDMMotionReplaySimulationTest::CaseCount &&
        elapsedMs >= (static_cast<std::uint64_t>(m_gapDryRun.phase) + 1ULL) * EDMMotionReplaySimulationTest::CasePeriodMs)
    {
        const auto result = m_edmMotionReplay.Step(m_gapDryRun.phase);
        if (!result.passed)
        { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM27_CASE_FAILED"); return false; }
        m_gapDryRun.passedMask |= 1U << m_gapDryRun.phase;
        ++m_gapDryRun.phase;
        char line[512]{};
        const int length = std::snprintf(line, sizeof(line),
            "[EDM27] event=CASE result=PASS test=%llu index=%u name=%.40s checks=%u caseMask=%08X admission=%u reason=%u proof=SYNTHETIC motion=0 physicalPermit=0 discharge=0\n",
            static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned>(m_gapDryRun.phase - 1U),
            EDMMotionReplaySimulationTest::CaseName(m_gapDryRun.phase - 1U), static_cast<unsigned>(result.checks), static_cast<unsigned>(m_gapDryRun.passedMask),
            static_cast<unsigned>(result.actual.admissionState), static_cast<unsigned>(result.actual.admissionReason));
        if (length < 0 || static_cast<std::size_t>(length) >= sizeof(line))
        { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM27_LOG_FORMAT"); return false; }
        RtPrintf("%s", line);
        return false;
    }
    if (elapsedMs >= EDMMotionReplaySimulationTest::MinimumWindowMs && snapshot.completionReady &&
        m_gapDryRun.passedMask == EDMMotionReplaySimulationTest::CaseMask && snapshot.caseMask == m_gapDryRun.passedMask &&
        snapshot.completedCycles >= 3ULL && snapshot.workElapsedMs == 0ULL && snapshot.flushOffsetMm == 0.0 &&
        snapshot.state == static_cast<std::uint32_t>(EDM25::ProcessState::Running) &&
        snapshot.shortState == static_cast<std::uint32_t>(EDMGapServo::ShortState::Clear) &&
        elapsedMs <= EDMMotionReplaySimulationTest::MaximumWindowMs)
    {
        // Recheck immediately before sealing, then recheck again in the next
        // consumer callback before permitting the NC block to finalize.
        if (!IsEDMCoordinatorAuthorityCurrentSameThread())
        { CancelGapDryRunSameThread("EDM27_AUTHORITY_CHANGED"); return false; }
        EndEDMMotionReplaySameThread("PASS", "INTENT_CONTRACT_PASS", nowMs);
        return false;
    }
    if (elapsedMs >= EDMMotionReplaySimulationTest::MaximumWindowMs)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM27_WINDOW_INCOMPLETE"); return false; }
    if (snapshot.action != m_edmMotionReplayLoggedAction || snapshot.admissionState != m_edmMotionReplayLoggedAdmission ||
        nowMs - m_edmMotionReplayLogMs >= 1000ULL)
    {
        m_edmMotionReplayLoggedAction = static_cast<std::uint8_t>(snapshot.action);
        m_edmMotionReplayLoggedAdmission = static_cast<std::uint8_t>(snapshot.admissionState);
        m_edmMotionReplayLogMs = nowMs;
        char line[512]{};
        const int length = std::snprintf(line, sizeof(line),
            "[EDM27] event=PROGRESS test=%llu wallMs=%llu process=%u action=%u short=%u virtualCycles=%llu workMs=%llu speedMmMin=%.9g machiningMm=%.9g flushMm=%.9g admission=%u reason=%u reserveMm=%.9g ack=%u stopProof=%u returnProof=%u proof=SYNTHETIC motion=0 physicalPermit=0 discharge=0\n",
            static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(elapsedMs),
            static_cast<unsigned>(snapshot.state), static_cast<unsigned>(snapshot.action), static_cast<unsigned>(snapshot.shortState),
            static_cast<unsigned long long>(snapshot.completedCycles), static_cast<unsigned long long>(snapshot.workElapsedMs),
            snapshot.speedMmMin, snapshot.machiningOffsetMm, snapshot.flushOffsetMm, static_cast<unsigned>(snapshot.admissionState),
            static_cast<unsigned>(snapshot.admissionReason), snapshot.stopReserveMm, snapshot.acknowledged ? 1U : 0U,
            snapshot.standstillProven ? 1U : 0U, snapshot.returnProven ? 1U : 0U);
        if (length < 0 || static_cast<std::size_t>(length) >= sizeof(line))
        { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM27_LOG_FORMAT"); return false; }
        RtPrintf("%s", line);
    }
    return false;
}

NCManager::EDMLiveProcessReceiptStatus NCManager::CheckEDMMotionReplayReceiptSameThread(std::uint64_t nowMs) noexcept
{
    if (!m_edmMotionReplayReceiptPending || !IsEDMCoordinatorAuthorityCurrentSameThread() ||
        nowMs < m_edmMotionReplayReceiptClockMs || nowMs - m_edmMotionReplayReceiptStartMs > 250ULL)
        return EDMLiveProcessReceiptStatus::Invalid;
    if (nowMs == m_edmMotionReplayReceiptClockMs)
    {
        if (++m_edmMotionReplayReceiptStalledCalls >= 4096U) return EDMLiveProcessReceiptStatus::Invalid;
        return EDMLiveProcessReceiptStatus::Wait;
    }
    m_edmMotionReplayReceiptClockMs = nowMs;
    m_edmMotionReplayReceiptPending = false;
    return m_edmMotionReplay.VerifyReceipt(nowMs - m_gapDryRun.phaseStartMs, MakeEDMMotionReplayScopeSameThread()) ?
        EDMLiveProcessReceiptStatus::Ready : EDMLiveProcessReceiptStatus::Invalid;
}

bool NCManager::ProcessEDMCoordinatorDryRunCaseSameThread(std::uint64_t nowMs)
{
    // Check after the final CASE as well as before every model call. A prior
    // all-case mask alone is never sufficient to release the NC block.
    if (!IsEDMCoordinatorAuthorityCurrentSameThread())
    {
        CancelGapDryRunSameThread("EDM25_AUTHORITY_CHANGED");
        return false;
    }
    const std::uint32_t phase = m_gapDryRun.phase;
    if (phase > EDMProcessCoordinatorSimulationTest::CaseCount)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM25_CASE_INDEX");
        return false;
    }
    char line[512]{};
    if (phase == EDMProcessCoordinatorSimulationTest::CaseCount)
    {
        if (m_gapDryRun.passedMask != EDMCoordinatorAllCases)
        {
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM25_COVERAGE");
            return false;
        }
        m_edmProcessCoordinatorTest.Revoke();
        m_edmMotionReplay.Revoke();
        m_edmMotionReplayReceiptPending = false;
        RevokeEDMLiveProcessSameThread();
        const int length = std::snprintf(line, sizeof(line),
            "[EDM25] event=SUMMARY result=PASS test=%llu run=%llu dispatch=%llu restart=%u cases=%u passedMask=%08X clock=VIRTUAL_CASES fixture=INDEPENDENT mode=SHADOW_ONLY motion=0 physicalPermit=0 discharge=0\n",
            static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(m_gapDryRun.run),
            static_cast<unsigned long long>(m_gapDryRun.dispatch), static_cast<unsigned int>(m_gapDryRun.restarts),
            static_cast<unsigned int>(EDMProcessCoordinatorSimulationTest::CaseCount), static_cast<unsigned int>(m_gapDryRun.passedMask));
        if (length < 0 || static_cast<std::size_t>(length) >= sizeof(line))
        {
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM25_LOG_FORMAT");
            return false;
        }
        m_gapDryRun.active = false; m_gapDryRun.result = 1U;
        RtPrintf("%s", line);
        return true;
    }
    if (++m_gapDryRun.observations >= 4096U)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM25_CLOCK_NOT_ADVANCING");
        return false;
    }
    if (nowMs - m_gapDryRun.phaseStartMs < EDMProcessCaseMs) return false;
    const auto result = m_edmProcessCoordinatorTest.Step(phase);
    const auto& actual = result.actual;
    if (!result.passed || result.checks == 0U || actual.PhysicalMotionEnabled() || actual.PhysicalDischargeEnabled() ||
        actual.modelCalls > EDMProcessCoordinatorSimulationTest::MaximumModelCallsPerCase ||
        !std::isfinite(actual.machiningOffsetMm) || !std::isfinite(actual.flushOffsetMm) || !std::isfinite(actual.speedMmMin))
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine,
            EDMProcessCoordinatorSimulationTest::CaseName(phase));
        return false;
    }
    const int length = std::snprintf(line, sizeof(line),
        "[EDM25] event=CASE test=%llu restart=%u case=%u name=%.64s result=PASS checks=%u state=%u reason=%u action=%u direction=%u shortState=%u cycles=%llu workMs=%llu machiningMm=%.9g flushMm=%.9g speedMmMin=%.9g deep=%u modelCalls=%u clock=VIRTUAL_CASES fixture=INDEPENDENT mode=SHADOW_ONLY motion=0 physicalPermit=0 discharge=0\n",
        static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned int>(m_gapDryRun.restarts),
        static_cast<unsigned int>(phase + 1U), EDMProcessCoordinatorSimulationTest::CaseName(phase),
        static_cast<unsigned int>(result.checks), static_cast<unsigned int>(actual.state), static_cast<unsigned int>(actual.reason),
        static_cast<unsigned int>(actual.action), static_cast<unsigned int>(actual.direction), static_cast<unsigned int>(actual.shortState),
        static_cast<unsigned long long>(actual.completedCycles), static_cast<unsigned long long>(actual.workElapsedMs),
        actual.machiningOffsetMm, actual.flushOffsetMm, actual.speedMmMin, actual.deepCycle ? 1U : 0U,
        static_cast<unsigned int>(actual.modelCalls));
    if (length < 0 || static_cast<std::size_t>(length) >= sizeof(line))
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM25_LOG_FORMAT");
        return false;
    }
    RtPrintf("%s", line);
    m_gapDryRun.passedMask |= 1U << phase; ++m_gapDryRun.phase;
    m_gapDryRun.phaseStartMs = nowMs; m_gapDryRun.observations = 0U;
    return false; // Final PASS waits for the next authority-checked callback.
}

// P8 owns independent deterministic automatic flush fixtures. Current COND
// appears only in BEGIN metadata; no fixture installs values in live control.
bool NCManager::ProcessEDMAutomaticFlushDryRunCaseSameThread(std::uint64_t nowMs)
{
    const std::uint32_t phase = m_gapDryRun.phase;
    if (phase > EDMAutomaticFlushSimulationTest::CaseCount)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM23_CASE_INDEX");
        return false;
    }
    char line[512]{};
    if (phase == EDMAutomaticFlushSimulationTest::CaseCount)
    {
        if (m_gapDryRun.passedMask != EDMAutomaticFlushAllCases)
        {
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM23_COVERAGE");
            return false;
        }
        m_edmAutomaticFlushTest.Revoke();
        m_edmProcessCoordinatorTest.Revoke();
        m_edmMotionReplay.Revoke();
        m_edmMotionReplayReceiptPending = false;
        RevokeEDMLiveProcessSameThread();
        m_edmLiveAutomaticFlush.Revoke();
        const int length = std::snprintf(line, sizeof(line),
            "[EDM23] event=SUMMARY result=PASS test=%llu run=%llu dispatch=%llu restart=%u cases=%u passedMask=%08X clock=VIRTUAL_CASES fixture=INDEPENDENT automaticActive=0 mode=SHADOW_ONLY motion=0 physicalPermit=0 discharge=0\n",
            static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(m_gapDryRun.run),
            static_cast<unsigned long long>(m_gapDryRun.dispatch), static_cast<unsigned int>(m_gapDryRun.restarts),
            static_cast<unsigned int>(EDMAutomaticFlushSimulationTest::CaseCount), static_cast<unsigned int>(m_gapDryRun.passedMask));
        if (length < 0 || static_cast<std::size_t>(length) >= sizeof(line))
        {
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM23_LOG_FORMAT");
            return false;
        }
        m_gapDryRun.active = false; m_gapDryRun.result = 1U;
        RtPrintf("%s", line);
        return true;
    }
    if (++m_gapDryRun.observations >= 4096U)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM23_CLOCK_NOT_ADVANCING");
        return false;
    }
    if (nowMs - m_gapDryRun.phaseStartMs < EDMProcessCaseMs) return false;
    const auto result = m_edmAutomaticFlushTest.Step(phase);
    const auto& actual = result.actual;
    if (!result.passed || result.checks == 0U || actual.PhysicalMotionEnabled() || actual.PhysicalDischargeEnabled() ||
        !std::isfinite(actual.offsetMm) || !std::isfinite(actual.speedMmMin) || !std::isfinite(actual.heightMm))
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, EDMAutomaticFlushSimulationTest::CaseName(phase));
        return false;
    }
    const int length = std::snprintf(line, sizeof(line),
        "[EDM23] event=CASE test=%llu restart=%u case=%u name=%.64s result=PASS checks=%u state=%u reason=%u cycles=%llu workMs=%llu heightMm=%.9g deep=%u offsetMm=%.9g speedMmMin=%.9g modelCalls=%u clock=VIRTUAL_CASES fixture=INDEPENDENT mode=SHADOW_ONLY motion=0 physicalPermit=0 discharge=0\n",
        static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned int>(m_gapDryRun.restarts),
        static_cast<unsigned int>(phase + 1U), EDMAutomaticFlushSimulationTest::CaseName(phase),
        static_cast<unsigned int>(result.checks), static_cast<unsigned int>(actual.state), static_cast<unsigned int>(actual.reason),
        static_cast<unsigned long long>(actual.completedCycles), static_cast<unsigned long long>(actual.workElapsedMs),
        actual.heightMm, actual.deepCycle ? 1U : 0U, actual.offsetMm, actual.speedMmMin, static_cast<unsigned int>(actual.modelCalls));
    if (length < 0 || static_cast<std::size_t>(length) >= sizeof(line))
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM23_LOG_FORMAT");
        return false;
    }
    RtPrintf("%s", line);
    m_gapDryRun.passedMask |= 1U << phase; ++m_gapDryRun.phase;
    m_gapDryRun.phaseStartMs = nowMs; m_gapDryRun.observations = 0U;
    return false; // Final PASS must wait for the next authority-checked scan.
}

// P7 owns isolated deterministic short/flush fixtures. Wall time only paces logs.
// Neither synthetic profile nor virtual offset is installed in live control.
bool NCManager::ProcessEDMShortFlushDryRunCaseSameThread(std::uint64_t nowMs)
{
    const std::uint32_t phase = m_gapDryRun.phase;
    if (phase > EDMShortFlushSimulationTest::CaseCount)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "EDM22_CASE_INDEX");
        return false;
    }
    char line[512]{};
    if (phase == EDMShortFlushSimulationTest::CaseCount)
    {
        if (m_gapDryRun.passedMask != EDMShortFlushAllCases)
        {
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
                m_gapDryRun.sourceLine, "EDM22_COVERAGE");
            return false;
        }
        m_edmShortFlushTest.Revoke();
        m_edmAutomaticFlushTest.Revoke();
        m_edmProcessCoordinatorTest.Revoke();
        m_edmMotionReplay.Revoke();
        m_edmMotionReplayReceiptPending = false;
        RevokeEDMLiveProcessSameThread();
        m_edmLiveAutomaticFlush.Revoke();
        const int length = std::snprintf(line, sizeof(line),
            "[EDM22] event=SUMMARY result=PASS test=%llu run=%llu dispatch=%llu restart=%u cases=%u passedMask=%08X clock=VIRTUAL_CASES mode=SHADOW_ONLY motion=0 physicalPermit=0 discharge=0\n",
            static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(m_gapDryRun.run),
            static_cast<unsigned long long>(m_gapDryRun.dispatch), static_cast<unsigned int>(m_gapDryRun.restarts),
            static_cast<unsigned int>(EDMShortFlushSimulationTest::CaseCount), static_cast<unsigned int>(m_gapDryRun.passedMask));
        if (length < 0 || static_cast<std::size_t>(length) >= sizeof(line))
        {
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
                m_gapDryRun.sourceLine, "EDM22_LOG_FORMAT");
            return false;
        }
        m_gapDryRun.active = false;
        m_gapDryRun.result = 1U;
        RtPrintf("%s", line);
        return true;
    }
    if (++m_gapDryRun.observations >= 4096U)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "EDM22_CLOCK_NOT_ADVANCING");
        return false;
    }
    if (nowMs - m_gapDryRun.phaseStartMs < EDMProcessCaseMs) return false;
    const EDMShortFlushSimulationTest::CaseResult result = m_edmShortFlushTest.Step(phase);
    const auto& actual = result.actual;
    if (!result.passed || result.checks == 0U || actual.PhysicalMotionEnabled() ||
        actual.PhysicalDischargeEnabled() || !std::isfinite(actual.offsetMm) || !std::isfinite(actual.speedMmMin))
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, EDMShortFlushSimulationTest::CaseName(phase));
        return false;
    }
    const int length = std::snprintf(line, sizeof(line),
        "[EDM22] event=CASE test=%llu restart=%u case=%u name=%.64s result=PASS checks=%u state=%u reason=%u segment=%u shortState=%u short=%u inhibit=%u offsetMm=%.9g speedMmMin=%.9g modelCalls=%u clock=VIRTUAL_CASES mode=SHADOW_ONLY motion=0 physicalPermit=0 discharge=0\n",
        static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned int>(m_gapDryRun.restarts),
        static_cast<unsigned int>(phase + 1U), EDMShortFlushSimulationTest::CaseName(phase),
        static_cast<unsigned int>(result.checks), static_cast<unsigned int>(actual.state),
        static_cast<unsigned int>(actual.reason), static_cast<unsigned int>(actual.segment),
        static_cast<unsigned int>(actual.shortState), actual.shortActive ? 1U : 0U, actual.feedInhibited ? 1U : 0U,
        actual.offsetMm, actual.speedMmMin, static_cast<unsigned int>(actual.modelCalls));
    if (length < 0 || static_cast<std::size_t>(length) >= sizeof(line))
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "EDM22_LOG_FORMAT");
        return false;
    }
    RtPrintf("%s", line);
    m_gapDryRun.passedMask |= 1U << phase;
    ++m_gapDryRun.phase;
    m_gapDryRun.phaseStartMs = nowMs;
    m_gapDryRun.observations = 0U;
    return false; // Final PASS waits for the next authority-checked callback.
}

// P6 mutates only an independent controller bound to the boot immutable catalog.
bool NCManager::ProcessEDMRecipeDryRunCaseSameThread(std::uint64_t nowMs)
{
    const std::uint32_t phase = m_gapDryRun.phase;
    if (phase > EDMRecipeSimulationTest::CaseCount)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "EDM16_CASE_INDEX");
        return false;
    }
    if (phase == EDMRecipeSimulationTest::CaseCount)
    {
        if (m_gapDryRun.passedMask != EDMRecipeAllCases)
        {
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
                m_gapDryRun.sourceLine, "EDM16_COVERAGE");
            return false;
        }
        m_edmRecipeTest.Revoke();
        ClearEDMRecipeSelectionSameThread();
        m_gapInput.Reset();
        m_gapDryRun.active = false;
        m_gapDryRun.result = 1U;
        RtPrintf("[EDM16] event=SUMMARY result=PASS test=%llu run=%llu dispatch=%llu restart=%u cases=%u passedMask=%08X selected=0 mode=SHADOW_ONLY motion=0 physicalPermit=0 discharge=0\n",
            static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(m_gapDryRun.run),
            static_cast<unsigned long long>(m_gapDryRun.dispatch), static_cast<unsigned int>(m_gapDryRun.restarts),
            static_cast<unsigned int>(EDMRecipeSimulationTest::CaseCount), static_cast<unsigned int>(m_gapDryRun.passedMask));
        return true;
    }
    if (++m_gapDryRun.observations >= 4096U)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "EDM16_CLOCK_NOT_ADVANCING");
        return false;
    }
    if (nowMs - m_gapDryRun.phaseStartMs < EDMProcessCaseMs) return false;
    const EDMRecipeSimulationTest::CaseResult result = m_edmRecipeTest.Step(phase);
    if (!result.passed || result.checks == 0U || result.actual.PhysicalDischargeEnabled())
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, EDMRecipeSimulationTest::CaseName(phase));
        return false;
    }
    char actual[40]{}, base[40]{};
    if (!EDMRecipe::FormatValue(result.field.actualValue, actual, sizeof(actual)) ||
        !EDMRecipe::FormatValue(result.field.baseValue, base, sizeof(base)))
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "EDM16_FORMAT");
        return false;
    }
    RtPrintf("[EDM16] event=CASE test=%llu restart=%u case=%u name=%s result=PASS table=%u E=%u generation=%llu field=%u configured=%u stage=%u base=%s actual=%s override=%u reason=%s mode=SHADOW_ONLY motion=0 physicalPermit=0 discharge=0\n",
        static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned int>(m_gapDryRun.restarts),
        static_cast<unsigned int>(phase + 1U), EDMRecipeSimulationTest::CaseName(phase),
        static_cast<unsigned int>(result.actual.tableId), static_cast<unsigned int>(result.actual.eCode),
        static_cast<unsigned long long>(result.actual.generation), static_cast<unsigned int>(result.field.fieldId),
        result.field.configured ? 1U : 0U, static_cast<unsigned int>(result.field.stageId), base, actual,
        result.field.overridePresent ? 1U : 0U, EDMRecipe::ErrorName(result.actual.lastError));
    m_gapDryRun.passedMask |= 1U << phase;
    ++m_gapDryRun.phase;
    m_gapDryRun.phaseStartMs = nowMs;
    m_gapDryRun.observations = 0U;
    return false; // Final PASS waits for another complete authority check.
}

// P5 uses only the boot-installed profile and synthetic VIRTUAL_CASES captures.
bool NCManager::ProcessEDMVoltageDryRunCaseSameThread(std::uint64_t nowMs)
{
    const std::uint32_t phase = m_gapDryRun.phase;
    if (phase > EDMVoltageSimulationTest::CaseCount)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "EDM15_CASE_INDEX");
        return false;
    }
    if (phase == EDMVoltageSimulationTest::CaseCount)
    {
        if (m_gapDryRun.passedMask != EDMVoltageAllCases)
        {
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
                m_gapDryRun.sourceLine, "EDM15_COVERAGE");
            return false;
        }
        m_edmVoltageTest.Revoke();
        m_gapInput.Reset();
        m_gapDryRun.active = false;
        m_gapDryRun.result = 1U;
        RtPrintf("[EDM15] event=SUMMARY result=PASS test=%llu run=%llu dispatch=%llu restart=%u cases=%u passedMask=%08X liveValid=0 mode=SYNTHETIC_SHADOW clock=VIRTUAL_CASES syntheticOnly=1 motion=0 physicalPermit=0 discharge=0\n",
            static_cast<unsigned long long>(m_gapDryRun.test),
            static_cast<unsigned long long>(m_gapDryRun.run),
            static_cast<unsigned long long>(m_gapDryRun.dispatch),
            static_cast<unsigned int>(m_gapDryRun.restarts),
            static_cast<unsigned int>(EDMVoltageSimulationTest::CaseCount),
            static_cast<unsigned int>(m_gapDryRun.passedMask));
        return true;
    }
    if (++m_gapDryRun.observations >= 4096U)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "EDM15_CLOCK_NOT_ADVANCING");
        return false;
    }
    if (nowMs - m_gapDryRun.phaseStartMs < EDMProcessCaseMs) return false;
    const EDMVoltageSimulationTest::CaseResult result = m_edmVoltageTest.Step(phase);
    if (!result.passed || result.checks == 0U || result.actual.PhysicalDischargeEnabled())
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, EDMVoltageSimulationTest::CaseName(phase));
        return false;
    }
    const auto& value = result.actual;
    const unsigned int mv = value.liveVoltageValid ? static_cast<unsigned int>(value.voltageMv) : 0U;
    RtPrintf("[EDM15] event=CASE test=%llu restart=%u case=%u name=%s result=PASS session=%llu raw=%d mv=%d volts=%u.%03u valid=%u quality=%s reason=%s ageKnown=%u ageMs=%llu lastGood=%u lastGoodMv=%d clock=VIRTUAL_CASES syntheticOnly=1 motion=0 physicalPermit=0 discharge=0\n",
        static_cast<unsigned long long>(m_gapDryRun.test),
        static_cast<unsigned int>(m_gapDryRun.restarts), static_cast<unsigned int>(phase + 1U),
        EDMVoltageSimulationTest::CaseName(phase), static_cast<unsigned long long>(value.consumerSession),
        static_cast<int>(value.rawCode), static_cast<int>(value.voltageMv), mv / 1000U, mv % 1000U,
        value.liveVoltageValid ? 1U : 0U, EDMGap::QualityName(value.quality),
        EDMGapAcquisition::ReasonName(value.reason), value.ageKnown ? 1U : 0U,
        static_cast<unsigned long long>(value.ageMs), value.lastGoodAvailable ? 1U : 0U,
        static_cast<int>(value.lastGoodVoltageMv));
    m_gapDryRun.passedMask |= 1U << phase;
    ++m_gapDryRun.phase;
    m_gapDryRun.phaseStartMs = nowMs;
    m_gapDryRun.observations = 0U;
    // Completion is emitted only by a later callback after all authority checks.
    return false;
}

// P4 exercises the production adapter with explicitly synthetic ADC frames.
// The real clock only paces cases/operator actions; it never stamps their input.
bool NCManager::ProcessEDMAcquisitionDryRunCaseSameThread(std::uint64_t nowMs)
{
    const std::uint32_t phase = m_gapDryRun.phase;
    if (phase > EDMGapAcquisitionSimulationTest::CaseCount)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "EDM14_CASE_INDEX");
        return false;
    }
    // Deliberately defer the terminal line to a later authority-checked callback.
    if (phase == EDMGapAcquisitionSimulationTest::CaseCount)
    {
        if (m_gapDryRun.passedMask != EDMAcquisitionAllCases)
        {
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
                m_gapDryRun.sourceLine, "EDM14_COVERAGE");
            return false;
        }
        m_edmAcquisitionTest.Revoke();
        m_gapInput.Reset();
        m_gapDryRun.active = false;
        m_gapDryRun.result = 1U;
        RtPrintf("[EDM14] event=SUMMARY result=PASS build=EDM14 test=%llu run=%llu dispatch=%llu restart=%u cases=%u passedMask=%08X mode=SYNTHETIC_ADC_SHADOW clock=VIRTUAL_CASES motion=0 physicalPermit=0 discharge=0\n",
            static_cast<unsigned long long>(m_gapDryRun.test),
            static_cast<unsigned long long>(m_gapDryRun.run),
            static_cast<unsigned long long>(m_gapDryRun.dispatch),
            static_cast<unsigned int>(m_gapDryRun.restarts),
            static_cast<unsigned int>(EDMGapAcquisitionSimulationTest::CaseCount),
            static_cast<unsigned int>(m_gapDryRun.passedMask));
        return true;
    }
    if (++m_gapDryRun.observations >= 4096U)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "EDM14_CLOCK_NOT_ADVANCING");
        return false;
    }
    if (nowMs - m_gapDryRun.phaseStartMs < EDMProcessCaseMs) return false;
    const EDMGapAcquisitionSimulationTest::CaseResult result = m_edmAcquisitionTest.Step(phase);
    if (!result.passed || result.checks == 0U || result.actual.PhysicalDischargeEnabled())
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, EDMGapAcquisitionSimulationTest::CaseName(phase));
        return false;
    }
    const auto& observed = result.actual;
    RtPrintf("[EDM14] event=CASE test=%llu restart=%u case=%u name=%s result=PASS checks=%u session=%llu reason=%s quality=%s band=%s mv=%d seq=%llu captureMs=%llu clock=VIRTUAL_CASES motion=0 physicalPermit=0 discharge=0\n",
        static_cast<unsigned long long>(m_gapDryRun.test),
        static_cast<unsigned int>(m_gapDryRun.restarts), static_cast<unsigned int>(phase + 1U),
        EDMGapAcquisitionSimulationTest::CaseName(phase), static_cast<unsigned int>(result.checks),
        static_cast<unsigned long long>(observed.consumerSession),
        EDMGapAcquisition::ReasonName(observed.reason), EDMGap::QualityName(observed.gap.quality),
        EDMGap::BandName(observed.gap.band), static_cast<int>(observed.gap.voltageMv),
        static_cast<unsigned long long>(observed.gap.sequence),
        static_cast<unsigned long long>(observed.gap.sampledAtMs));
    m_gapDryRun.passedMask |= 1U << phase;
    ++m_gapDryRun.phase;
    m_gapDryRun.phaseStartMs = nowMs;
    m_gapDryRun.observations = 0U;
    return false;
}

// EDM01 uses the real NC clock only for bounded scheduling and operator actions.
// Every case has its own deterministic virtual GAP clock. No value here is an
// actual ADC sample, a feed command, or a physical pulse-enable request.
bool NCManager::ProcessEDMProcessDryRunCaseSameThread(std::uint64_t nowMs)
{
    const std::uint32_t phase = m_gapDryRun.phase;
    if (phase >= EDMProcessSimulationTest::CaseCount)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "EDM01_CASE_INDEX");
        return false;
    }
    if (++m_gapDryRun.observations >= 4096U)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "EDM01_CLOCK_NOT_ADVANCING");
        return false;
    }
    if (nowMs - m_gapDryRun.phaseStartMs < EDMProcessCaseMs) return false;
    const EDMProcessSimulationTest::CaseResult result = m_edmProcessTest.Step(phase);
    const auto& observed = result.actual;
    const bool matching = result.passed && !observed.PhysicalDischargeEnabled();
    RtPrintf("[EDM01] event=CASE test=%llu restart=%u case=%u name=%s result=%s expected=%s state=%s reason=%s intent=%s simPermit=%u retreatIntent=%u clock=VIRTUAL_CASES motion=0 discharge=0\n",
        static_cast<unsigned long long>(m_gapDryRun.test),
        static_cast<unsigned int>(m_gapDryRun.restarts), static_cast<unsigned int>(phase + 1U),
        EDMProcessSimulationTest::CaseName(phase), matching ? "PASS" : "FAIL",
        EDMProcessSimulation::StateName(result.expectedState),
        EDMProcessSimulation::StateName(observed.state),
        EDMProcessSimulation::ReasonName(observed.reason),
        EDMProcessSimulation::FeedIntentName(observed.feedIntent),
        observed.simulatedPermit ? 1U : 0U, observed.retreatRequested ? 1U : 0U);
    if (!matching)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, EDMProcessSimulationTest::CaseName(phase));
        return false;
    }
    m_gapDryRun.passedMask |= 1U << phase;
    ++m_gapDryRun.phase;
    m_gapDryRun.phaseStartMs = nowMs;
    m_gapDryRun.observations = 0U;
    if (m_gapDryRun.phase != EDMProcessSimulationTest::CaseCount) return false;
    if (m_gapDryRun.passedMask != EDMProcessAllCases)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "EDM01_COVERAGE");
        return false;
    }
    // Completion also retires any remaining synthetic processing permission.
    m_edmProcessTest.Revoke();
    m_edmFeedRetreatTest.Revoke();
    m_gapInput.Reset();
    m_gapDryRun.active = false;
    m_gapDryRun.result = 1U;
    RtPrintf("[EDM01] event=SUMMARY result=PASS test=%llu run=%llu dispatch=%llu line=%d restart=%u cases=%u passedMask=%08X mode=SIMULATED clock=VIRTUAL_CASES simPermit=0 motion=0 discharge=0\n",
        static_cast<unsigned long long>(m_gapDryRun.test),
        static_cast<unsigned long long>(m_gapDryRun.run),
        static_cast<unsigned long long>(m_gapDryRun.dispatch), m_gapDryRun.sourceLine,
        static_cast<unsigned int>(m_gapDryRun.restarts),
        static_cast<unsigned int>(EDMProcessSimulationTest::CaseCount),
        static_cast<unsigned int>(m_gapDryRun.passedMask));
    return true;
}

// EDM02 continuously connects the real GAP/permission models to a virtual
// consumer. The real clock only paces operator-visible cases, never motion.
bool NCManager::ProcessEDMFeedRetreatDryRunCaseSameThread(std::uint64_t nowMs)
{
    const std::uint32_t phase = m_gapDryRun.phase;
    if (phase >= EDMFeedRetreatSimulationTest::CaseCount)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "EDM02_CASE_INDEX");
        return false;
    }
    if (++m_gapDryRun.observations >= 4096U)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "EDM02_CLOCK_NOT_ADVANCING");
        return false;
    }
    if (nowMs - m_gapDryRun.phaseStartMs < EDMProcessCaseMs) return false;
    const EDMFeedRetreatSimulationTest::CaseResult result = m_edmFeedRetreatTest.Step(phase);
    const auto& observed = result.actual;
    const bool matching = result.passed && !observed.PhysicalDischargeEnabled();
    RtPrintf("[EDM02] event=CASE test=%llu restart=%u case=%u name=%s result=%s expected=%s phase=%s action=%s reason=%s simPermit=%u virtualPosition=%d virtualVelocity=%d commandToken=%llu clock=VIRTUAL_CONTINUOUS consumer=VIRTUAL motion=0 discharge=0\n",
        static_cast<unsigned long long>(m_gapDryRun.test),
        static_cast<unsigned int>(m_gapDryRun.restarts), static_cast<unsigned int>(phase + 1U),
        EDMFeedRetreatSimulationTest::CaseName(phase), matching ? "PASS" : "FAIL",
        EDMFeedRetreatSimulation::PhaseName(result.expectedPhase),
        EDMFeedRetreatSimulation::PhaseName(observed.phase),
        EDMFeedRetreatSimulation::ActionName(observed.action),
        EDMFeedRetreatSimulation::ReasonName(observed.reason),
        observed.simulatedPermit ? 1U : 0U,
        static_cast<int>(observed.virtualPosition), static_cast<int>(observed.simulatedVelocity),
        static_cast<unsigned long long>(observed.commandToken));
    if (!matching)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, EDMFeedRetreatSimulationTest::CaseName(phase));
        return false;
    }
    m_gapDryRun.passedMask |= 1U << phase;
    ++m_gapDryRun.phase;
    m_gapDryRun.phaseStartMs = nowMs;
    m_gapDryRun.observations = 0U;
    if (m_gapDryRun.phase != EDMFeedRetreatSimulationTest::CaseCount) return false;
    if (m_gapDryRun.passedMask != EDMFeedRetreatAllCases)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "EDM02_COVERAGE");
        return false;
    }
    m_edmFeedRetreatTest.Revoke();
    m_edmProcessTest.Revoke();
    m_gapInput.Reset();
    m_gapDryRun.active = false;
    m_gapDryRun.result = 1U;
    RtPrintf("[EDM02] event=SUMMARY result=PASS build=EDM02 test=%llu run=%llu dispatch=%llu line=%d restart=%u cases=%u passedMask=%08X mode=SIMULATED clock=VIRTUAL_CONTINUOUS consumer=VIRTUAL simPermit=0 virtualVelocity=0 motion=0 discharge=0\n",
        static_cast<unsigned long long>(m_gapDryRun.test),
        static_cast<unsigned long long>(m_gapDryRun.run),
        static_cast<unsigned long long>(m_gapDryRun.dispatch), m_gapDryRun.sourceLine,
        static_cast<unsigned int>(m_gapDryRun.restarts),
        static_cast<unsigned int>(EDMFeedRetreatSimulationTest::CaseCount),
        static_cast<unsigned int>(m_gapDryRun.passedMask));
    return true;
}

// EDM24: P9 explicitly observes the installed live COND/GAP. This session has
// only a virtual B0 actuator; it cannot produce a Motion or discharge permit.
EDM24::LiveObservation NCManager::MakeEDMLiveAutomaticFlushObservationSameThread(std::uint64_t nowMs) const noexcept
{
    EDM24::LiveObservation input{};
    input.nowMs = nowMs;
    input.configRevision = m_edmProcessProfile.revision;
    input.recipeGeneration = m_recipe.Current().generation;
    input.gap = m_edmProcessObservedGap; // Current owner observation, including invalid identities; never reread PDO here.
    input.interlockReady = IsEDMShortFlushRunReadySameThread();
    const auto& shadow = m_edmProcessShadow;
    const auto summary = m_recipe.Current();
    input.shadowReady = m_edmProcessReady && m_edmProcessHaveGap && m_edmProcessHaveClock &&
        !m_recipeRecoveryDisplayOnly && shadow.reason == EDMProcessShadowReason::Ready &&
        shadow.profileReady && shadow.recipeReady && shadow.previewAvailable &&
        shadow.configRevision == input.configRevision && shadow.machineProfileId == m_edmProcessProfile.machineProfileId &&
        shadow.recipeGeneration == input.recipeGeneration && shadow.tableId == summary.tableId && shadow.eCode == summary.eCode &&
        summary.tableId == m_edmLiveAutomaticFlushBinding.tableId && summary.eCode == m_edmLiveAutomaticFlushBinding.eCode &&
        shadow.source == input.gap.configuredSource && shadow.sampleSequence == input.gap.sampleSequence &&
        shadow.observedAtMs == input.gap.observedAtMs && shadow.observedAtMs == m_edmProcessLastObservedMs &&
        shadow.voltageV == static_cast<double>(input.gap.voltageMv) / 1000.0 &&
        shadow.machiningShort.valid && std::isfinite(shadow.machiningScenarioSpeedMmPerMin);
    input.shortActive = shadow.machiningShort.feedInhibited;
    input.machiningAllowed = input.shadowReady && shadow.machiningScenarioSpeedMmPerMin > 0.0;
    input.returnAllowed = input.shadowReady && !input.shortActive;
    return input;
}

bool NCManager::IsEDMLiveAutomaticFlushReceiptCurrentSameThread(std::uint64_t nowMs) const noexcept
{
    const auto input = MakeEDMLiveAutomaticFlushObservationSameThread(nowMs);
    const auto& gap = input.gap;
    const auto& receipt = m_edmLiveAutomaticFlush.Snapshot();
    const auto& identity = receipt.sourceIdentity;
    // A PASS receipt must still refer to a fresh, consistent owner observation
    // on the next scan. Cancellation, source changes and edits cannot reach M30.
    return input.interlockReady && input.shadowReady && input.machiningAllowed && input.returnAllowed &&
        input.configRevision == receipt.cycle.configRevision && input.recipeGeneration == receipt.cycle.recipeGeneration &&
        gap.configuredSource == identity.source && gap.gap.source == identity.source &&
        gap.profileRevision == identity.profileRevision && gap.maxAgeMs == identity.maxAgeMs &&
        (identity.source != EDMGap::Source::PHYSICAL ||
            (gap.adIndex == identity.adIndex && gap.deviceId == identity.deviceId && gap.channelId == identity.channelId &&
             gap.pdoOffset == identity.pdoOffset && gap.calibrationRevision == identity.calibrationRevision &&
             gap.calibrationConfirmed == identity.calibrationConfirmed)) &&
        gap.liveVoltageValid && gap.ownerClockValid && gap.ageKnown && gap.configValid &&
        (identity.source != EDMGap::Source::PHYSICAL ||
            (gap.inputStatus == EDMGapInput::InputStatus::Valid && gap.rawAvailable && gap.boardVoltageValid && gap.calibrationConfirmed)) &&
        gap.gap.configured && gap.gap.quality == EDMGap::Quality::VALID &&
        gap.gap.sequence == gap.sampleSequence && gap.gap.sampledAtMs == gap.sampledAtMs &&
        gap.gap.voltageMv == gap.voltageMv && gap.gap.observedAtMs <= nowMs &&
        gap.observedAtMs <= nowMs && gap.sampledAtMs <= gap.observedAtMs &&
        gap.ageMs == gap.observedAtMs - gap.sampledAtMs &&
        nowMs - gap.sampledAtMs <= gap.maxAgeMs && nowMs - gap.observedAtMs <= gap.maxAgeMs &&
        gap.sampleSequence > receipt.lastSampleSeq && gap.sampledAtMs > receipt.lastSampleTimeMs;
}

void NCManager::EndEDMLiveAutomaticFlushSameThread(const char* result, const char* reason,
    std::uint64_t nowMs, std::uint16_t failedField)
{
    const auto snapshot = m_edmLiveAutomaticFlush.Snapshot();
    const bool passed = std::strcmp(result, "PASS") == 0;
    const bool cancelled = std::strcmp(result, "CANCELLED") == 0;
    m_gapDryRun.active = false;
    m_gapDryRun.paused = false;
    m_gapDryRun.result = passed ? 1U : (cancelled ? 2U : 3U);
    m_edmLiveAutomaticFlushResume = EDMLiveAutomaticFlushResumeState{};
    m_edmLiveProcessResume = EDMLiveAutomaticFlushResumeState{};
    m_edmLiveProcessReceiptWait = EDMLiveAutomaticFlushResumeState{};
    m_edmLiveAutomaticFlush.Revoke();
    char line[512]{};
    const int length = std::snprintf(line, sizeof(line),
        "[EDM24] event=SUMMARY result=%.16s test=%llu run=%llu dispatch=%llu restart=%u reason=%.48s field=%u wallMs=%llu cycles=%llu workMs=%llu offsetMm=%.9g fresh=%llu blocked=%llu config=%llu generation=%llu mode=LIVE_COND_SHADOW origin=VIRTUAL_B0 motion=0 physicalPermit=0 discharge=0\n",
        result, static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(m_gapDryRun.run),
        static_cast<unsigned long long>(m_gapDryRun.dispatch), static_cast<unsigned>(m_gapDryRun.restarts),
        reason ? reason : "UNKNOWN", static_cast<unsigned>(failedField),
        static_cast<unsigned long long>(nowMs >= m_gapDryRun.phaseStartMs ? nowMs - m_gapDryRun.phaseStartMs : 0ULL),
        static_cast<unsigned long long>(snapshot.cycle.completedCycles), static_cast<unsigned long long>(snapshot.cycle.workElapsedMs),
        snapshot.cycle.virtualOffsetMm, static_cast<unsigned long long>(snapshot.freshSamples),
        static_cast<unsigned long long>(snapshot.blockedSamples), static_cast<unsigned long long>(snapshot.cycle.configRevision),
        static_cast<unsigned long long>(snapshot.cycle.recipeGeneration));
    if (length >= 0 && static_cast<std::size_t>(length) < sizeof(line)) RtPrintf("%s", line);
    else
    {
        m_gapDryRun.result = 3U;
        RtPrintf("[EDM24] event=SUMMARY result=FAIL reason=LOG_FORMAT mode=LIVE_COND_SHADOW motion=0 physicalPermit=0 discharge=0\n");
    }
}

bool NCManager::ProcessEDMLiveAutomaticFlushResumeSameThread()
{
    std::uint64_t nowMs = 0ULL;
    if (!ReadGapDryRunClockSameThread(nowMs))
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "EDM24_RESUME_CLOCK_READ");
        return false;
    }
    m_gapDryRun.lastServiceMs = nowMs;
    auto& resume = m_edmLiveAutomaticFlushResume;
    const auto& gap = m_edmProcessObservedGap; // Same owner observation; no PDO reread.
    if (!resume.pending)
    {
        // The first post-START callback pins the new authority. It never uses
        // the previous HOLD sample to restart or advance a virtual actuator.
        m_edmLiveAutomaticFlushBinding = EDM23::BindRecipe(m_recipe, m_edmProcessProfile.machineProfileId);
        EDM23::AutomaticFlushCycleConfig config{};
        EDM20::FlushRequest request{};
        request.mode = EDM20::FlushMode::B0;
        request.jumpHeightConfirmed = request.frame.machiningDirectionConfirmed = true;
        const auto reason = m_edmProcessShadow.reason;
        const bool recoverable = reason == EDMProcessShadowReason::Ready ||
            reason == EDMProcessShadowReason::Hold || reason == EDMProcessShadowReason::AwaitFreshSample;
        const char* rejected = !m_edmLiveAutomaticFlushBinding.ready ?
            EDM23::RecipeBindingErrorName(m_edmLiveAutomaticFlushBinding.error) :
            (!m_edmProcessReady ? "CONFIG_UNAVAILABLE" :
             (!EDM23::MakeAutomaticFlushCycleConfig(m_edmLiveAutomaticFlushBinding,
                m_edmProcessProfile.flush, m_edmProcessProfile.revision, request, config) ?
              "PLAN_ADMISSION_BLOCKED" : (!recoverable ? "EDM24_RESUME_SHADOW_NOT_READY" :
                (!EDMLiveResumeGapCurrent(gap, nowMs) ? "EDM24_RESUME_INVALID_GAP" : nullptr))));
        if (rejected != nullptr)
        {
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, rejected);
            return false;
        }
        resume.pending = true;
        resume.gapFloor = gap;
        resume.startMs = resume.lastClockMs = nowMs;
        resume.configRevision = m_edmProcessProfile.revision;
        resume.recipeGeneration = m_recipe.Current().generation;
        m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch();
        m_gapDryRun.lease = m_programMotionLease;
        char line[512]{};
        const int length = std::snprintf(line, sizeof(line),
            "[EDM24] event=RESUME_WAIT test=%llu run=%llu dispatch=%llu reason=FRESH_RUN_OBSERVATION maxWaitMs=250 seq=%llu config=%llu generation=%llu mode=LIVE_COND_SHADOW motion=0 physicalPermit=0 discharge=0\n",
            static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(m_gapDryRun.run),
            static_cast<unsigned long long>(m_gapDryRun.dispatch), static_cast<unsigned long long>(gap.sampleSequence),
            static_cast<unsigned long long>(resume.configRevision), static_cast<unsigned long long>(resume.recipeGeneration));
        if (length < 0 || static_cast<std::size_t>(length) >= sizeof(line))
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM24_LOG_FORMAT");
        else RtPrintf("%s", line);
        return false;
    }
    const char* rejected = nullptr;
    if (nowMs < resume.lastClockMs) rejected = "EDM24_RESUME_CLOCK_REGRESSION";
    else if (nowMs - resume.startMs > EDMLiveResumeMaxWaitMs) rejected = "EDM24_RESUME_TIMEOUT";
    else if (nowMs == resume.lastClockMs && ++resume.stalledCalls >= 4096U)
        rejected = "EDM24_RESUME_CLOCK_NOT_ADVANCING";
    else if (m_edmProcessProfile.revision != resume.configRevision) rejected = "CONFIG_REVISION_CHANGED";
    else if (m_recipe.Current().generation != resume.recipeGeneration) rejected = "RECIPE_GENERATION_CHANGED";
    else if (!EDMLiveResumeSameSource(gap, resume.gapFloor)) rejected = "EDM24_RESUME_SOURCE_CHANGED";
    else if (gap.observedAtMs < resume.gapFloor.observedAtMs) rejected = "EDM24_RESUME_OBSERVER_REGRESSION";
    else if (gap.sampleSequence < resume.gapFloor.sampleSequence) rejected = "SAMPLE_SEQUENCE_REGRESSED";
    else if (gap.sampledAtMs < resume.gapFloor.sampledAtMs) rejected = "SAMPLE_TIME_REGRESSED";
    else if (gap.sampleSequence == resume.gapFloor.sampleSequence &&
        (gap.sampledAtMs != resume.gapFloor.sampledAtMs || gap.voltageMv != resume.gapFloor.voltageMv ||
            (gap.configuredSource == EDMGap::Source::PHYSICAL && gap.rawCode != resume.gapFloor.rawCode)))
        rejected = "CONTRADICTORY_SAMPLE";
    else if (!EDMLiveResumeGapCurrent(gap, nowMs)) rejected = "EDM24_RESUME_INVALID_GAP";
    else if (m_edmProcessShadow.reason != EDMProcessShadowReason::Ready &&
        m_edmProcessShadow.reason != EDMProcessShadowReason::Hold &&
        m_edmProcessShadow.reason != EDMProcessShadowReason::AwaitFreshSample)
        rejected = "EDM24_RESUME_SHADOW_NOT_READY";
    if (rejected != nullptr)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, rejected);
        return false;
    }
    if (nowMs != resume.lastClockMs) resume.stalledCalls = 0U;
    resume.lastClockMs = nowMs;
    const bool fresh = gap.sampleSequence > resume.gapFloor.sampleSequence &&
        gap.sampledAtMs > resume.gapFloor.sampledAtMs;
    // Consume a renumbered cache floor without admitting its old acquisition.
    if (gap.sampleSequence > resume.gapFloor.sampleSequence)
    {
        resume.gapFloor.sampleSequence = gap.sampleSequence;
        resume.gapFloor.sampledAtMs = gap.sampledAtMs;
        resume.gapFloor.rawCode = gap.rawCode;
        resume.gapFloor.voltageMv = gap.voltageMv;
    }
    resume.gapFloor.observedAtMs = gap.observedAtMs;
    if (!fresh || !MakeEDMLiveAutomaticFlushObservationSameThread(nowMs).shadowReady) return false;
    if (m_gapDryRun.restarts == (std::numeric_limits<std::uint32_t>::max)() ||
        m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)())
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine,
            m_gapDryRun.restarts == (std::numeric_limits<std::uint32_t>::max)() ? "RESTART_EXHAUSTED" : "TEST_SERIAL_EXHAUSTED");
        return false;
    }
    ++m_gapDryRun.restarts;
    m_gapDryRun.test = ++m_gapDryRunSerial;
    resume = EDMLiveAutomaticFlushResumeState{};
    (void)RestartGapDryRunSameThread();
    return false; // BEGIN never shares a callback with virtual cycle progress.
}

bool NCManager::RestartEDMLiveAutomaticFlushSameThread(std::uint64_t nowMs)
{
    m_edmLiveAutomaticFlush.Reset();
    m_edmLiveAutomaticFlushBinding = EDM23::BindRecipe(m_recipe, m_edmProcessProfile.machineProfileId);
    const auto& binding = m_edmLiveAutomaticFlushBinding;
    EDM23::AutomaticFlushCycleConfig config{};
    EDM20::FlushRequest request{};
    request.mode = EDM20::FlushMode::B0;
    request.jumpHeightConfirmed = true;
    request.frame.machiningDirectionConfirmed = true; // Local virtual frame only.
    const bool mapped = m_edmProcessReady && EDM23::MakeAutomaticFlushCycleConfig(binding,
        m_edmProcessProfile.flush, m_edmProcessProfile.revision, request, config);
    const auto input = MakeEDMLiveAutomaticFlushObservationSameThread(nowMs);
    const char* blocked = !binding.ready ? EDM23::RecipeBindingErrorName(binding.error) :
        (!m_edmProcessReady ? "CONFIG_UNAVAILABLE" : (!mapped ? "PLAN_ADMISSION_BLOCKED" :
        (!input.shadowReady ? "SHADOW_NOT_READY" : nullptr)));
    if (blocked != nullptr)
    {
        EndEDMLiveAutomaticFlushSameThread("BLOCKED", blocked, nowMs, binding.failedFieldId);
        AlarmManager::GetInstance().Trigger(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine);
        ChangeState(NCState::ALARM);
        return false;
    }
    if (!m_edmLiveAutomaticFlush.Start(config, input.gap, nowMs))
    {
        EndEDMLiveAutomaticFlushSameThread("BLOCKED", EDM24::LiveAutomaticFlushReasonName(m_edmLiveAutomaticFlush.Snapshot().reason), nowMs);
        AlarmManager::GetInstance().Trigger(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine);
        ChangeState(NCState::ALARM);
        return false;
    }
    m_edmLiveAutomaticFlushLogMs = m_edmLiveAutomaticFlushClockMs = nowMs;
    m_edmLiveAutomaticFlushLoggedCycles = 0ULL;
    m_edmLiveAutomaticFlushLoggedCycleState = 255U;
    m_edmLiveAutomaticFlushLoggedState = m_edmLiveAutomaticFlushLoggedReason = 255U;
    char line[512]{};
    const int length = std::snprintf(line, sizeof(line),
        "[EDM24] event=BEGIN test=%llu run=%llu dispatch=%llu restart=%u COND=%u E=%u generation=%llu config=%llu E5ms=%llu E6mm=%.9g E7pct=%.9g E21cycles=%u E22x=%.9g source=%s AD=%u seq=%llu minMs=10000 maxMs=12000 requiredCycles=3 clock=WALL mode=LIVE_COND_SHADOW origin=VIRTUAL_B0 motion=0 physicalPermit=0 discharge=0\n",
        static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(m_gapDryRun.run),
        static_cast<unsigned long long>(m_gapDryRun.dispatch), static_cast<unsigned>(m_gapDryRun.restarts),
        static_cast<unsigned>(binding.tableId), static_cast<unsigned>(binding.eCode), static_cast<unsigned long long>(binding.generation),
        static_cast<unsigned long long>(config.configRevision), static_cast<unsigned long long>(binding.workTimeMs), binding.jumpHeightMm,
        binding.speedOverridePercent, static_cast<unsigned>(binding.deepFlushCycleInterval), binding.deepFlushHeightMultiplier,
        EDMGap::SourceName(input.gap.configuredSource), static_cast<unsigned>(input.gap.adIndex),
        static_cast<unsigned long long>(input.gap.sampleSequence));
    if (length < 0 || static_cast<std::size_t>(length) >= sizeof(line))
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM24_LOG_FORMAT");
        return false;
    }
    RtPrintf("%s", line);
    return true;
}

bool NCManager::ProcessEDMLiveAutomaticFlushSameThread(std::uint64_t nowMs)
{
    if (nowMs == m_edmLiveAutomaticFlushClockMs)
    {
        if (++m_gapDryRun.observations >= 4096U)
        {
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM24_CLOCK_NOT_ADVANCING");
            return false;
        }
    }
    else { m_gapDryRun.observations = 0U; m_edmLiveAutomaticFlushClockMs = nowMs; }
    const auto input = MakeEDMLiveAutomaticFlushObservationSameThread(nowMs);
    // Current observer/source coherence retires an inconsistent owner snapshot.
    if (m_edmProcessShadow.source != m_edmLiveAutomaticFlush.Snapshot().sourceIdentity.source)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM24_SOURCE_CHANGED");
        return false;
    }
    const auto& snapshot = m_edmLiveAutomaticFlush.Observe(input);
    if (snapshot.state == EDM24::LiveAutomaticFlushState::Fault)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, EDM24::LiveAutomaticFlushReasonName(snapshot.reason));
        return false;
    }
    const std::uint64_t wallMs = nowMs - m_gapDryRun.phaseStartMs;
    const bool boundary = snapshot.state == EDM24::LiveAutomaticFlushState::Running && snapshot.acceptedFreshSample &&
        input.shadowReady && input.machiningAllowed && input.returnAllowed &&
        snapshot.cycle.completedCycles >= 3ULL && snapshot.cycle.state == EDM23::AutomaticFlushCycleState::Machining &&
        snapshot.cycle.workElapsedMs == 0ULL && snapshot.cycle.virtualOffsetMm == 0.0;
    if (wallMs >= 10000ULL && wallMs <= 12000ULL && boundary)
    {
        EndEDMLiveAutomaticFlushSameThread("PASS", "RETURNED_CYCLE_BOUNDARY", nowMs);
        return false; // Next scan revalidates the completed scope/observer receipt.
    }
    if (wallMs >= 12000ULL)
    {
        EndEDMLiveAutomaticFlushSameThread("INCOMPLETE", "WINDOW_WITHOUT_COMPLETE_BOUNDARY", nowMs);
        AlarmManager::GetInstance().Trigger(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine);
        ChangeState(NCState::ALARM);
        return false;
    }
    const auto state = static_cast<std::uint8_t>(snapshot.state);
    const auto reason = static_cast<std::uint8_t>(snapshot.reason);
    const auto cycleState = static_cast<std::uint8_t>(snapshot.cycle.state);
    if (state != m_edmLiveAutomaticFlushLoggedState || reason != m_edmLiveAutomaticFlushLoggedReason ||
        cycleState != m_edmLiveAutomaticFlushLoggedCycleState ||
        snapshot.cycle.completedCycles != m_edmLiveAutomaticFlushLoggedCycles || nowMs - m_edmLiveAutomaticFlushLogMs >= 1000ULL)
    {
        char line[512]{};
        const int length = std::snprintf(line, sizeof(line),
            "[EDM24] event=PROGRESS test=%llu wallMs=%llu state=%s reason=%.40s cycleState=%u workMs=%llu cycles=%llu heightMm=%.9g deep=%u offsetMm=%.9g speedMmMin=%.9g fresh=%u seq=%llu sampleAgeMs=%llu AD=%u config=%llu generation=%llu clock=WALL mode=LIVE_COND_SHADOW motion=0 physicalPermit=0 discharge=0\n",
            static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(wallMs),
            EDM24::LiveAutomaticFlushStateName(snapshot.state), EDM24::LiveAutomaticFlushReasonName(snapshot.reason), static_cast<unsigned>(snapshot.cycle.state),
            static_cast<unsigned long long>(snapshot.cycle.workElapsedMs), static_cast<unsigned long long>(snapshot.cycle.completedCycles),
            snapshot.cycle.effectiveHeightMm, snapshot.cycle.deepCycle ? 1U : 0U, snapshot.cycle.virtualOffsetMm,
            snapshot.requestedSpeedMmPerMin, snapshot.acceptedFreshSample ? 1U : 0U, static_cast<unsigned long long>(input.gap.sampleSequence),
            static_cast<unsigned long long>(nowMs >= input.gap.sampledAtMs ? nowMs - input.gap.sampledAtMs : 0ULL),
            static_cast<unsigned>(input.gap.adIndex), static_cast<unsigned long long>(input.configRevision),
            static_cast<unsigned long long>(input.recipeGeneration));
        if (length < 0 || static_cast<std::size_t>(length) >= sizeof(line))
        {
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM24_LOG_FORMAT");
            return false;
        }
        RtPrintf("%s", line);
        m_edmLiveAutomaticFlushLogMs = nowMs;
        m_edmLiveAutomaticFlushLoggedState = state;
        m_edmLiveAutomaticFlushLoggedReason = reason;
        m_edmLiveAutomaticFlushLoggedCycleState = cycleState;
        m_edmLiveAutomaticFlushLoggedCycles = snapshot.cycle.completedCycles;
    }
    return false;
}

bool NCManager::ProcessEDMLiveProcessResumeSameThread()
{
    std::uint64_t nowMs = 0ULL;
    if (!ReadGapDryRunClockSameThread(nowMs))
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "EDM26_RESUME_CLOCK_READ");
        return false;
    }
    m_gapDryRun.lastServiceMs = nowMs;
    auto& resume = m_edmLiveProcessResume;
    const auto& gap = m_edmProcessObservedGap; // Same owner observation; no PDO reread.
    if (!resume.pending)
    {
        // The first post-START callback pins the new authority. It never uses
        // the previous HOLD sample to restart or advance a virtual actuator.
        EDM20::FlushRequest request{};
        request.mode = EDM20::FlushMode::B0;
        request.jumpHeightConfirmed = request.frame.machiningDirectionConfirmed = true;
        const auto candidate = EDM26::BindProcessRecipe(m_recipe, m_edmProcessProfile, gap, request);
        const auto reason = m_edmProcessShadow.reason;
        const bool recoverable = reason == EDMProcessShadowReason::Ready ||
            reason == EDMProcessShadowReason::Hold || reason == EDMProcessShadowReason::AwaitFreshSample;
        const char* rejected = !m_edmLiveProcessBinding.ready ?
            EDM23::RecipeBindingErrorName(m_edmLiveProcessBinding.error) :
            (!m_edmProcessReady ? "CONFIG_UNAVAILABLE" :
             (!candidate.ready ?
              "PLAN_ADMISSION_BLOCKED" : (!recoverable ? "EDM26_RESUME_SHADOW_NOT_READY" :
                (!EDMLiveResumeGapCurrent(gap, nowMs) ? "EDM26_RESUME_INVALID_GAP" : nullptr))));
        if (rejected != nullptr)
        {
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, rejected);
            return false;
        }
        if (!m_motion.IsGroupDone() || m_motion.GetCommandIngressSize() != 0U ||
            m_motion.GetCommandReplaySize() != 0U || m_pathFeed.pending || m_pathArc.pending ||
            m_pathReplay.pending || m_pathHold.armed || m_pathHold.bound)
        {
            CancelGapDryRunSameThread("EDM26_MOTION_NOT_IDLE");
            return false;
        }
        resume.pending = true;
        resume.gapFloor = gap;
        resume.startMs = resume.lastClockMs = nowMs;
        resume.configRevision = m_edmProcessProfile.revision;
        resume.recipeGeneration = m_recipe.Current().generation;
        m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch();
        m_gapDryRun.lease = m_programMotionLease;
        char line[512]{};
        const int length = std::snprintf(line, sizeof(line),
            "[EDM26] event=RESUME_WAIT test=%llu run=%llu dispatch=%llu reason=FRESH_RUN_OBSERVATION maxWaitMs=250 seq=%llu config=%llu generation=%llu mode=LIVE_COND_SHADOW motion=0 physicalPermit=0 discharge=0\n",
            static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(m_gapDryRun.run),
            static_cast<unsigned long long>(m_gapDryRun.dispatch), static_cast<unsigned long long>(gap.sampleSequence),
            static_cast<unsigned long long>(resume.configRevision), static_cast<unsigned long long>(resume.recipeGeneration));
        if (length < 0 || static_cast<std::size_t>(length) >= sizeof(line))
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM26_LOG_FORMAT");
        else RtPrintf("%s", line);
        return false;
    }
    if (!IsEDMCoordinatorAuthorityCurrentSameThread())
    {
        CancelGapDryRunSameThread("EDM26_AUTHORITY_CHANGED");
        return false;
    }
    const char* rejected = nullptr;
    if (nowMs < resume.lastClockMs) rejected = "EDM26_RESUME_CLOCK_REGRESSION";
    else if (nowMs - resume.startMs > EDMLiveResumeMaxWaitMs) rejected = "EDM26_RESUME_TIMEOUT";
    else if (nowMs == resume.lastClockMs && ++resume.stalledCalls >= 4096U)
        rejected = "EDM26_RESUME_CLOCK_NOT_ADVANCING";
    else if (m_edmProcessProfile.revision != resume.configRevision) rejected = "CONFIG_REVISION_CHANGED";
    else if (m_recipe.Current().generation != resume.recipeGeneration) rejected = "RECIPE_GENERATION_CHANGED";
    else if (!EDMLiveResumeSameSource(gap, resume.gapFloor)) rejected = "EDM26_RESUME_SOURCE_CHANGED";
    else if (gap.observedAtMs < resume.gapFloor.observedAtMs) rejected = "EDM26_RESUME_OBSERVER_REGRESSION";
    else if (gap.sampleSequence < resume.gapFloor.sampleSequence) rejected = "SAMPLE_SEQUENCE_REGRESSED";
    else if (gap.sampledAtMs < resume.gapFloor.sampledAtMs) rejected = "SAMPLE_TIME_REGRESSED";
    else if (gap.sampleSequence == resume.gapFloor.sampleSequence &&
        (gap.sampledAtMs != resume.gapFloor.sampledAtMs || gap.voltageMv != resume.gapFloor.voltageMv ||
            (gap.configuredSource == EDMGap::Source::PHYSICAL && gap.rawCode != resume.gapFloor.rawCode)))
        rejected = "CONTRADICTORY_SAMPLE";
    else if (!EDMLiveResumeGapCurrent(gap, nowMs)) rejected = "EDM26_RESUME_INVALID_GAP";
    else if (m_edmProcessShadow.reason != EDMProcessShadowReason::Ready &&
        m_edmProcessShadow.reason != EDMProcessShadowReason::Hold &&
        m_edmProcessShadow.reason != EDMProcessShadowReason::AwaitFreshSample)
        rejected = "EDM26_RESUME_SHADOW_NOT_READY";
    if (rejected != nullptr)
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, rejected);
        return false;
    }
    if (nowMs != resume.lastClockMs) resume.stalledCalls = 0U;
    resume.lastClockMs = nowMs;
    const bool fresh = gap.sampleSequence > resume.gapFloor.sampleSequence &&
        gap.sampledAtMs > resume.gapFloor.sampledAtMs;
    // Consume a renumbered cache floor without admitting its old acquisition.
    if (gap.sampleSequence > resume.gapFloor.sampleSequence)
    {
        resume.gapFloor.sampleSequence = gap.sampleSequence;
        resume.gapFloor.sampledAtMs = gap.sampledAtMs;
        resume.gapFloor.rawCode = gap.rawCode;
        resume.gapFloor.voltageMv = gap.voltageMv;
    }
    resume.gapFloor.observedAtMs = gap.observedAtMs;
    if (!fresh || !MakeEDMLiveProcessObservationSameThread(nowMs).shadowReady) return false;
    if (m_gapDryRun.restarts == (std::numeric_limits<std::uint32_t>::max)() ||
        m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)())
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine,
            m_gapDryRun.restarts == (std::numeric_limits<std::uint32_t>::max)() ? "RESTART_EXHAUSTED" : "TEST_SERIAL_EXHAUSTED");
        return false;
    }
    ++m_gapDryRun.restarts;
    m_gapDryRun.test = ++m_gapDryRunSerial;
    resume = EDMLiveAutomaticFlushResumeState{};
    (void)RestartGapDryRunSameThread();
    return false; // BEGIN never shares a callback with virtual cycle progress.
}


// EDM26 P11: installed COND and the existing NC owner GAP observation drive one
// virtual action. No sample acquisition, Motion submission, PID or IO occurs.
EDM26::LiveObservation NCManager::MakeEDMLiveProcessObservationSameThread(std::uint64_t nowMs) const noexcept
{
    EDM26::LiveObservation input{};
    input.nowMs = nowMs; input.configRevision = m_edmProcessProfile.revision;
    input.recipeGeneration = m_recipe.Current().generation;
    input.gap = m_edmProcessObservedGap;
    input.interlockReady = IsEDMShortFlushRunReadySameThread();
    const auto& shadow = m_edmProcessShadow;
    const auto summary = m_recipe.Current();
    input.shadowReady = m_edmProcessReady && m_edmProcessHaveGap && m_edmProcessHaveClock &&
        !m_recipeRecoveryDisplayOnly && shadow.reason == EDMProcessShadowReason::Ready &&
        shadow.profileReady && shadow.recipeReady && shadow.previewAvailable &&
        shadow.configRevision == input.configRevision && shadow.machineProfileId == m_edmProcessProfile.machineProfileId &&
        shadow.recipeGeneration == input.recipeGeneration && shadow.tableId == summary.tableId && shadow.eCode == summary.eCode &&
        summary.tableId == m_edmLiveProcessBinding.tableId && summary.eCode == m_edmLiveProcessBinding.eCode &&
        shadow.source == input.gap.configuredSource && shadow.sampleSequence == input.gap.sampleSequence &&
        shadow.observedAtMs == input.gap.observedAtMs && shadow.observedAtMs == m_edmProcessLastObservedMs &&
        shadow.voltageV == static_cast<double>(input.gap.voltageMv) / 1000.0 && shadow.curve.valid;
    // This allows both signs. EDM25 alone chooses servo/short/flush action and
    // determines whether a positive, uninhibited endpoint earns working time.
    input.machiningAllowed = input.returnAllowed = input.shadowReady;
    input.shortActive = false;
    return input;
}

NCManager::EDMLiveProcessReceiptStatus NCManager::CheckEDMLiveProcessReceiptSameThread(std::uint64_t nowMs) noexcept
{
    // A revoked zero intent is only a local shadow lifecycle condition.
    // It is never a replacement for a Motion/drive stop or Disarm receipt.
    const auto& intent = m_edmProcessMotionIntent.Snapshot();
    if (intent.state != EDM46::IntentState::Revoked || !EDM46::AllSpeedsZero(intent) ||
        !EDM46::SameScope(intent.scope, MakeEDMProcessIntentScopeSameThread()))
        return EDMLiveProcessReceiptStatus::Invalid;
    const auto input = MakeEDMLiveProcessObservationSameThread(nowMs);
    const auto& gap = input.gap;
    const auto& receipt = m_edmLiveProcess.Snapshot();
    const auto& identity = receipt.sourceIdentity;
    const bool valid = IsEDMCoordinatorAuthorityCurrentSameThread() && input.interlockReady && input.shadowReady &&
        input.machiningAllowed && input.returnAllowed && EDMLiveResumeGapCurrent(gap, nowMs) &&
        m_edmProcessShadow.machiningShort.valid &&
        m_edmProcessShadow.machiningShort.state == EDMGapServo::ShortState::Clear &&
        !m_edmProcessShadow.machiningShort.feedInhibited &&
        input.configRevision == receipt.cycle.configRevision && input.recipeGeneration == receipt.cycle.recipeGeneration &&
        gap.configuredSource == identity.source && gap.gap.source == identity.source &&
        gap.profileRevision == identity.profileRevision && gap.maxAgeMs == identity.maxAgeMs &&
        (identity.source != EDMGap::Source::PHYSICAL ||
            (gap.adIndex == identity.adIndex && gap.deviceId == identity.deviceId && gap.channelId == identity.channelId &&
             gap.pdoOffset == identity.pdoOffset && gap.calibrationRevision == identity.calibrationRevision &&
             gap.calibrationConfirmed == identity.calibrationConfirmed)) &&
        gap.sampleSequence >= receipt.lastSampleSeq && gap.sampledAtMs >= receipt.lastSampleTimeMs;
    auto& wait = m_edmLiveProcessReceiptWait;
    if (!valid || !wait.pending || nowMs < wait.lastClockMs || nowMs < wait.startMs ||
        nowMs - wait.startMs > EDMLiveResumeMaxWaitMs ||
        !EDMLiveResumeSameSource(gap, wait.gapFloor) || gap.observedAtMs < wait.gapFloor.observedAtMs ||
        gap.sampleSequence < wait.gapFloor.sampleSequence || gap.sampledAtMs < wait.gapFloor.sampledAtMs ||
        (gap.sampleSequence == wait.gapFloor.sampleSequence &&
            (gap.sampledAtMs != wait.gapFloor.sampledAtMs || gap.voltageMv != wait.gapFloor.voltageMv ||
             (identity.source == EDMGap::Source::PHYSICAL && gap.rawCode != wait.gapFloor.rawCode))))
        return EDMLiveProcessReceiptStatus::Invalid;
    if (nowMs == wait.lastClockMs && ++wait.stalledCalls >= 4096U)
        return EDMLiveProcessReceiptStatus::Invalid;
    if (nowMs != wait.lastClockMs) wait.stalledCalls = 0U;
    wait.lastClockMs = nowMs;
    const bool fresh = gap.sampleSequence > wait.gapFloor.sampleSequence && gap.sampledAtMs > wait.gapFloor.sampledAtMs;
    // Normal sub-ms polling and renumbered cached acquisitions never release
    // the completed NC block. Full gates/identity remain mandatory each scan.
    wait.gapFloor = gap;
    const bool completedBefore = m_edmProcessExecution.Snapshot().state == EDM47::ExecutionState::Completed;
    const auto drain = m_edmProcessExecution.PollDrain(MakeEDMProcessIntentScopeSameThread(), nowMs);
    if (drain == EDM47::DrainStatus::Invalid) return EDMLiveProcessReceiptStatus::Invalid;
    if (!completedBefore && drain == EDM47::DrainStatus::Ready)
        LogEDMProcessExecutionSameThread("COMPLETED");
    if (!ReadEDMRTShadowSameThread(nowMs)) return EDMLiveProcessReceiptStatus::Invalid;
    const auto& control = m_edmRTShadowControl;
    const auto& runtime = control.feedback;
    const bool rtComplete = control.current && control.drainSubmitted &&
        runtime.state == EDM48::State::Completed && !runtime.pendingDrain && EDM48::AllSpeedsZero(runtime) &&
        runtime.lastCompleted.stage == EDM47::ReceiptStage::Completed &&
        EDM47::SamePacket(runtime.lastCompleted.packet, control.submitted) &&
        runtime.lastCompleted.packet.kind == EDM47::CommandKind::Drain &&
        runtime.completedTick > runtime.appliedTick && runtime.lastCompleted.atMs > runtime.lastApplied.atMs &&
        runtime.lastCompleted.publication > runtime.lastApplied.publication;
    if (rtComplete && !m_edmRTShadowControl.completedLogged)
    {
        m_edmRTShadowControl.completedLogged = true;
        LogEDMRTShadowSameThread("COMPLETED");
    }
    // Three independent gates: local closure, RT shadow closure, fresh GAP.
    // None establishes a physical drive Stop, position or discharge receipt.
    return fresh && drain == EDM47::DrainStatus::Ready && rtComplete ?
        EDMLiveProcessReceiptStatus::Ready : EDMLiveProcessReceiptStatus::Wait;
}


void NCManager::EndEDMLiveProcessSameThread(const char* result, const char* reason,
    std::uint64_t nowMs, std::uint16_t failedField)
{
    const auto snapshot = m_edmLiveProcess.Snapshot();
    m_gapDryRun.active = m_gapDryRun.paused = false;
    m_gapDryRun.result = std::strcmp(result, "PASS") == 0 ? 1U : (std::strcmp(result, "CANCELLED") == 0 ? 2U : 3U);
    m_edmLiveProcessResume = EDMLiveAutomaticFlushResumeState{};
    m_edmLiveProcessReceiptWait = EDMLiveAutomaticFlushResumeState{};
    if (m_gapDryRun.result == 1U)
    {
        auto& wait = m_edmLiveProcessReceiptWait;
        wait.pending = true; wait.gapFloor = m_edmProcessObservedGap;
        wait.startMs = wait.lastClockMs = nowMs;
        wait.configRevision = m_edmProcessProfile.revision;
        wait.recipeGeneration = m_recipe.Current().generation;
    }
    m_edmLiveProcessConfigLogPending = false;
    if (!RevokeEDMLiveProcessSameThread(m_gapDryRun.result == 1U, nowMs))
    {
        result = "FAIL"; reason = "EDM47_DRAIN_REJECTED";
        m_gapDryRun.result = 3U;
        m_edmLiveProcessReceiptWait = EDMLiveAutomaticFlushResumeState{};
        m_edmProcessExecution.Cancel();
        AlarmManager::GetInstance().Trigger(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine);
        ChangeState(NCState::ALARM);
    }
    char line[512]{};
    const int length = std::snprintf(line, sizeof(line),
        "[EDM26] event=SUMMARY result=%.16s test=%llu run=%llu dispatch=%llu restart=%u reason=%.44s field=%u wallMs=%llu cycles=%llu workMs=%llu machiningMm=%.9g flushMm=%.9g fresh=%llu seq=%llu config=%llu generation=%llu mode=LIVE_COND_SHADOW motion=0 physicalPermit=0 discharge=0\n",
        result, static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(m_gapDryRun.run),
        static_cast<unsigned long long>(m_gapDryRun.dispatch), static_cast<unsigned>(m_gapDryRun.restarts), reason ? reason : "UNKNOWN",
        static_cast<unsigned>(failedField),
        static_cast<unsigned long long>(nowMs >= m_gapDryRun.phaseStartMs ? nowMs - m_gapDryRun.phaseStartMs : 0ULL),
        static_cast<unsigned long long>(snapshot.cycle.completedCycles), static_cast<unsigned long long>(snapshot.cycle.workElapsedMs),
        snapshot.process.machiningOffsetMm, snapshot.process.flushOffsetMm, static_cast<unsigned long long>(snapshot.freshSamples),
        static_cast<unsigned long long>(snapshot.lastSampleSeq), static_cast<unsigned long long>(snapshot.cycle.configRevision),
        static_cast<unsigned long long>(snapshot.cycle.recipeGeneration));
    if (length >= 0 && static_cast<std::size_t>(length) < sizeof(line)) RtPrintf("%s", line);
    else
    {
        m_gapDryRun.result = 3U;
        RtPrintf("[EDM26] event=SUMMARY result=FAIL reason=LOG_FORMAT mode=LIVE_COND_SHADOW motion=0 physicalPermit=0 discharge=0\n");
    }
}

// EDM46 owns only a virtual linear-axis intent. Its frame generation names
// this fixed -Z preview basis, not a real RT mapping or motion authorization.
EDM46::Scope NCManager::MakeEDMProcessIntentScopeSameThread() const noexcept
{
    EDM46::Scope scope{};
    scope.session = m_gapDryRun.test; scope.run = m_gapDryRun.run;
    scope.cache = m_gapDryRun.cache; scope.dispatch = m_gapDryRun.dispatch;
    scope.executionEpoch = m_gapDryRun.epoch;
    scope.ownerLease = (static_cast<std::uint64_t>(m_gapDryRun.lease.owner) << 32U) |
        static_cast<std::uint64_t>(m_gapDryRun.lease.generation);
    scope.configRevision = m_edmProcessProfile.revision;
    scope.recipeGeneration = m_recipe.Current().generation;
    scope.frameGeneration = 1ULL;
    return scope;
}

bool NCManager::StartEDMProcessIntentSameThread(std::uint64_t nowMs) noexcept
{
    EDM46::Config config{};
    config.frame.frameGeneration = 1ULL;
    config.frame.virtualFrameConfirmed = true;
    config.frame.selectedAxisMask = 1U << 2U;
    config.frame.machiningUnit[2] = -1.0;
    config.frame.machineZAxis = 2U;
    // Installed gain slots bound the scan only. Existence and LINEAR type
    // determine dimensional eligibility; they do not grant an axis permit.
    for (unsigned axis = 0U; axis < 8U; ++axis)
    {
        if ((m_edmProcessActiveAxisMask & (1U << axis)) == 0U) continue;
        const auto& installed = m_motion.GetAxisContext(static_cast<int>(axis));
        if (installed.isExist && installed.axisType == AxisType::LINEAR)
            config.frame.installedLinearAxisMask |= 1U << axis;
    }
    config.frame.machineZConfirmed = (config.frame.installedLinearAxisMask & (1U << 2U)) != 0U;
    const auto& bound = m_edmLiveProcessBinding.config;
    // Logical recipe bounds only. No physical velocity, stopping envelope,
    // current position or PID is inferred from this preview configuration.
    const auto& flush = bound.flush.flushProfile;
    const double limits[] = { bound.servo.maxFeedMmPerMin, bound.servo.maxRetreatMmPerMin,
        flush.finalApproachMmPerMin, flush.centerRetractMmPerMin, flush.mainRetractMmPerMin,
        flush.centerReturnMmPerMin, flush.mainReturnMmPerMin, flush.pathRetractMmPerMin,
        flush.initialSlowMmPerMin, flush.manualReturnMmPerMin };
    config.maximumScalarMmMin = 0.0;
    for (double limit : limits)
    {
        if (!std::isfinite(limit) || limit <= 0.0) return false;
        if (limit > config.maximumScalarMmMin) config.maximumScalarMmMin = limit;
    }
    config.maximumAxisMmMin[2] = config.maximumScalarMmMin;
    m_edmProcessMotionIntent.Reset(); // Retired-session floor survives Reset.
    if (!m_edmProcessMotionIntent.Start(config, MakeEDMProcessIntentScopeSameThread(),
        m_edmLiveProcess.Snapshot(), nowMs)) return false;
    if (!m_edmProcessExecution.Reset()) return false;
    if (!m_edmProcessExecution.Start(config, MakeEDMProcessIntentScopeSameThread(),
        m_edmProcessMotionIntent.Snapshot(), nowMs)) return false;
    // A superseded session is retired before publishing the new Start. The
    // runtime, not this NC call, owns the eventual Start/Applied evidence.
    CancelEDMRTShadowSameThread();
    m_edmRTShadowControl = EDMRTShadowControlState{};
    m_edmRTShadowControl.scope = MakeEDMProcessIntentScopeSameThread();
    m_edmRTShadowControl.lastFeedbackMs = m_edmRTShadowControl.lastPollMs = nowMs;
    m_edmRTShadowControl.logMs = nowMs;
    return m_motion.StartEDMRTShadow(config, m_edmRTShadowControl.scope,
        m_edmProcessMotionIntent.Snapshot(), nowMs);
}

bool NCManager::RevokeEDMLiveProcessSameThread(bool drainShadow, std::uint64_t nowMs) noexcept
{
    const bool report = m_gapDryRun.liveProcessTest &&
        m_edmProcessMotionIntent.Snapshot().state != EDM46::IntentState::Idle &&
        m_edmProcessMotionIntent.Snapshot().state != EDM46::IntentState::Revoked;
    // Cancellation is immediate and clock independent. Normal P11 closure
    // instead creates a distinct zero Drain packet; a later local consumer
    // publication must complete it before the fresh-GAP receipt may release.
    bool accepted = true;
    if (drainShadow)
    {
        accepted = m_edmProcessExecution.BeginDrain(nowMs) && SubmitEDMRTShadowSameThread();
        if (!accepted) CancelEDMRTShadowSameThread();
    }
    else
    {
        CancelEDMRTShadowSameThread();
        m_edmProcessExecution.Cancel();
    }
    m_edmProcessMotionIntent.Revoke();
    m_edmLiveProcess.Revoke();
    if (report)
    {
        LogEDMProcessIntentSameThread("REVOKED");
        LogEDMProcessExecutionSameThread(drainShadow ? "DRAIN" : "CANCELLED");
        if (drainShadow) LogEDMRTShadowSameThread(accepted ? "DRAIN_QUEUED" : "DRAIN_FAILED");
    }
    return accepted;
}


// EDM48 never equates NC enqueue with runtime application. Only a coherent
// RT publication with the current producer fences is eligible as readback.
bool NCManager::ReadEDMRTShadowSameThread(std::uint64_t nowMs) noexcept
{
    auto& control = m_edmRTShadowControl;
    control.current = false;
    if (!control.scope.session || control.cancelled || nowMs < control.lastPollMs ||
        nowMs < control.lastFeedbackMs) return false;
    control.lastPollMs = nowMs;
    EDM48::Feedback runtime{};
    if (m_motion.ReadEDMRTShadowFeedback(runtime))
    {
        if (!runtime.publication || runtime.latestSession != control.scope.session) return false;
        if (runtime.state == EDM48::State::Fault || runtime.state == EDM48::State::Cancelled)
        {
            // Preserve the actual RT rejection for READ_FAILED diagnostics;
            // it remains ineligible for any completion or motion decision.
            control.feedback = runtime; control.observed = true; return false;
        }
        if (!runtime.rtTick || !runtime.nowUs) return false;
        if (runtime.startAcknowledged)
        {
            if (!EDM46::SameScope(runtime.scope, control.scope) ||
                runtime.lastCommand > control.submitted.commandSequence ||
                (control.observed && (runtime.publication < control.feedback.publication ||
                    runtime.rtTick < control.feedback.rtTick || runtime.nowUs < control.feedback.nowUs))) return false;
            if (runtime.lastApplied.stage == EDM47::ReceiptStage::Applied &&
                runtime.state != EDM48::State::Blocked &&
                !EDM47::SamePacket(runtime.lastApplied.packet, control.submitted)) return false;
            if ((runtime.state == EDM48::State::Draining || runtime.state == EDM48::State::Completed) &&
                !control.drainSubmitted) return false;
            // RT can be sampled just after the NC's captured millisecond.
            // Bound both directions rather than rejecting that normal race.
            const auto runtimeMs = runtime.nowUs / 1000ULL;
            if ((runtimeMs <= nowMs ? nowMs - runtimeMs : runtimeMs - nowMs) > EDMLiveResumeMaxWaitMs)
                return false;
            if (!control.observed || runtime.publication > control.feedback.publication)
                control.lastFeedbackMs = nowMs;
            control.feedback = runtime; control.observed = control.current = true;
        }
    }
    return nowMs - control.lastFeedbackMs <= EDMLiveResumeMaxWaitMs;
}

bool NCManager::SubmitEDMRTShadowSameThread() noexcept
{
    auto& control = m_edmRTShadowControl;
    const auto& local = m_edmProcessExecution.Snapshot();
    if (!control.scope.session || control.cancelled || !EDM46::SameScope(local.scope, control.scope)) return false;
    if (local.state == EDM47::ExecutionState::Blocked)
    {
        // No queue slot or valid clock is needed to invalidate old velocity.
        m_motion.NeutralizeEDMRTShadow(control.scope.session, local.lastAccepted.packet.commandSequence);
        control.current = false; return true;
    }
    if (local.state != EDM47::ExecutionState::Running && local.state != EDM47::ExecutionState::Draining) return false;
    if (local.lastApplied.stage != EDM47::ReceiptStage::Applied) return true;
    const auto& packet = local.lastApplied.packet;
    if (packet.commandSequence == control.submitted.commandSequence) return true;
    if (packet.commandSequence < control.submitted.commandSequence || !m_motion.SubmitEDMRTShadow(packet))
    {
        CancelEDMRTShadowSameThread(); return false;
    }
    control.submitted = packet; control.current = false;
    control.drainSubmitted = packet.kind == EDM47::CommandKind::Drain;
    return true;
}

void NCManager::CancelEDMRTShadowSameThread() noexcept
{
    auto& control = m_edmRTShadowControl;
    if (!control.scope.session || control.cancelled) return;
    m_motion.CancelEDMRTShadow(control.scope.session);
    control.cancelled = control.cancelPending = true; control.current = false;
    LogEDMRTShadowSameThread("CANCEL_REQUEST");
}

void NCManager::ServiceEDMRTShadowCancelSameThread() noexcept
{
    auto& control = m_edmRTShadowControl;
    if (!control.cancelPending) return;
    EDM48::Feedback runtime{};
    if (!m_motion.ReadEDMRTShadowFeedback(runtime) || !runtime.publication || !runtime.rtTick ||
        runtime.latestSession != control.scope.session || runtime.cancelThroughSession < control.scope.session ||
        runtime.state != EDM48::State::Cancelled || runtime.pendingDrain || !EDM48::AllSpeedsZero(runtime)) return;
    // The session fence also acknowledges Start cancelled before consumption;
    // it does not invent a frozen runtime scope that never existed.
    control.feedback = runtime; control.observed = control.current = true; control.cancelPending = false;
    LogEDMRTShadowSameThread("CANCEL_ACK");
}

void NCManager::LogEDMRTShadowSameThread(const char* event) noexcept
{
    const auto& control = m_edmRTShadowControl;
    const auto& runtime = control.feedback;
    char line[512]{};
    int length = std::snprintf(line, sizeof(line),
        "[EDM48] event=%.14s part=STATE test=%llu run=%llu queued=%llu seen=%u current=%u state=%s reason=%s pub=%llu rtTick=%llu rtUs=%llu superseded=%llu domain=RT_SHADOW motion=0 physicalPermit=0 discharge=0\n",
        event, static_cast<unsigned long long>(control.scope.session), static_cast<unsigned long long>(control.scope.run),
        static_cast<unsigned long long>(control.submitted.commandSequence), control.observed ? 1U : 0U, control.current ? 1U : 0U,
        control.observed ? EDM48::StateName(runtime.state) : "WAIT_RT", control.observed ? EDM48::ReasonName(runtime.reason) : "UNOBSERVED",
        static_cast<unsigned long long>(runtime.publication), static_cast<unsigned long long>(runtime.rtTick),
        static_cast<unsigned long long>(runtime.nowUs), static_cast<unsigned long long>(runtime.supersededCount));
    if (length >= 0 && static_cast<std::size_t>(length) < sizeof(line)) RtPrintf("%s", line);
    else RtPrintf("[EDM48] event=FORMAT_ERROR part=STATE motion=0 discharge=0\n");
    length = std::snprintf(line, sizeof(line),
        "[EDM48] event=%.14s part=RECEIPT test=%llu appliedCmd=%llu completedCmd=%llu applyTick=%llu completeTick=%llu applyMs=%llu completeMs=%llu rtZero=%u zMmMin=%.9g domain=RT_SHADOW stopProof=0 discharge=0\n",
        event, static_cast<unsigned long long>(control.scope.session),
        static_cast<unsigned long long>(runtime.lastApplied.packet.commandSequence),
        static_cast<unsigned long long>(runtime.lastCompleted.packet.commandSequence),
        static_cast<unsigned long long>(runtime.appliedTick), static_cast<unsigned long long>(runtime.completedTick),
        static_cast<unsigned long long>(runtime.lastApplied.atMs), static_cast<unsigned long long>(runtime.lastCompleted.atMs),
        control.current && EDM48::AllSpeedsZero(runtime) ? 1U : 0U, runtime.axisSpeedMmMin[2]);
    if (length >= 0 && static_cast<std::size_t>(length) < sizeof(line)) RtPrintf("%s", line);
    else RtPrintf("[EDM48] event=FORMAT_ERROR part=RECEIPT motion=0 discharge=0\n");
}

// Domain is deliberately local shadow. Accepted/Applied identify immutable
// packets, while Completed is only a later terminal Drain acknowledgement.
void NCManager::LogEDMProcessExecutionSameThread(const char* event) noexcept
{
    const auto& state = m_edmProcessExecution.Snapshot();
    char line[512]{};
    int length = std::snprintf(line, sizeof(line),
        "[EDM47] event=%.12s part=STATE test=%llu run=%llu state=%s reason=%s accepted=%llu applied=%llu completed=%llu neutralized=%llu pending=%u zero=%u domain=LOCAL_SHADOW motion=0 physicalPermit=0 discharge=0\n",
        event, static_cast<unsigned long long>(state.scope.session), static_cast<unsigned long long>(state.scope.run),
        EDM47::ExecutionStateName(state.state), EDM47::ExecutionReasonName(state.reason),
        static_cast<unsigned long long>(state.acceptedCount), static_cast<unsigned long long>(state.appliedCount),
        static_cast<unsigned long long>(state.completedCount), static_cast<unsigned long long>(state.neutralizedCount),
        state.pending ? 1U : 0U, EDM47::AllSpeedsZero(state) ? 1U : 0U);
    if (length >= 0 && static_cast<std::size_t>(length) < sizeof(line)) RtPrintf("%s", line);
    else RtPrintf("[EDM47] event=FORMAT_ERROR part=STATE motion=0 physicalPermit=0 discharge=0\n");
    length = std::snprintf(line, sizeof(line),
        "[EDM47] event=%.12s part=RECEIPT test=%llu acceptedCmd=%llu appliedCmd=%llu completedCmd=%llu applyPub=%llu completePub=%llu kind=%u a0=%.9g a1=%.9g a2=%.9g a3=%.9g a4=%.9g a5=%.9g a6=%.9g a7=%.9g domain=LOCAL_SHADOW stopProof=0 discharge=0\n",
        event, static_cast<unsigned long long>(state.scope.session),
        static_cast<unsigned long long>(state.lastAccepted.packet.commandSequence),
        static_cast<unsigned long long>(state.lastApplied.packet.commandSequence),
        static_cast<unsigned long long>(state.lastCompleted.packet.commandSequence),
        static_cast<unsigned long long>(state.lastApplied.publication),
        static_cast<unsigned long long>(state.lastCompleted.publication),
        static_cast<unsigned>(state.lastApplied.packet.kind),
        state.axisSpeedMmMin[0], state.axisSpeedMmMin[1], state.axisSpeedMmMin[2], state.axisSpeedMmMin[3],
        state.axisSpeedMmMin[4], state.axisSpeedMmMin[5], state.axisSpeedMmMin[6], state.axisSpeedMmMin[7]);
    if (length >= 0 && static_cast<std::size_t>(length) < sizeof(line)) RtPrintf("%s", line);
    else RtPrintf("[EDM47] event=FORMAT_ERROR part=RECEIPT motion=0 physicalPermit=0 discharge=0\n");
}

void NCManager::LogEDMProcessIntentSameThread(const char* event) noexcept
{
    const auto& intent = m_edmProcessMotionIntent.Snapshot();
    char line[512]{};
    int length = std::snprintf(line, sizeof(line),
        "[EDM46] event=%.12s part=STATE test=%llu run=%llu dispatch=%llu state=%s reason=%s seq=%llu sourceSeq=%llu ready=%llu nonzero=%llu actions=%02X nzActions=%02X zero=%u mode=INTENT_SHADOW motion=0 physicalPermit=0 discharge=0\n",
        event, static_cast<unsigned long long>(intent.scope.session),
        static_cast<unsigned long long>(intent.scope.run), static_cast<unsigned long long>(intent.scope.dispatch),
        EDM46::IntentStateName(intent.state), EDM46::IntentReasonName(intent.reason),
        static_cast<unsigned long long>(intent.sequence), static_cast<unsigned long long>(intent.sourceSequence),
        static_cast<unsigned long long>(intent.readyObservations), static_cast<unsigned long long>(intent.nonzeroObservations),
        static_cast<unsigned>(intent.actionMask), static_cast<unsigned>(intent.nonzeroActionMask),
        EDM46::AllSpeedsZero(intent) ? 1U : 0U);
    if (length >= 0 && static_cast<std::size_t>(length) < sizeof(line)) RtPrintf("%s", line);
    else RtPrintf("[EDM46] event=FORMAT_ERROR part=STATE motion=0 physicalPermit=0 discharge=0\n");
    length = std::snprintf(line, sizeof(line),
        "[EDM46] event=%.12s part=AXES test=%llu seq=%llu frame=%u action=%u scalar=%.9g a0=%.9g a1=%.9g a2=%.9g a3=%.9g a4=%.9g a5=%.9g a6=%.9g a7=%.9g unit=MM_MIN basis=VIRTUAL_NEG_Z target=NONE motion=0 physicalPermit=0 discharge=0\n",
        event, static_cast<unsigned long long>(intent.scope.session), static_cast<unsigned long long>(intent.sequence),
        static_cast<unsigned>(intent.frame), static_cast<unsigned>(intent.action), intent.signedSpeedMmMin,
        intent.axisSpeedMmMin[0], intent.axisSpeedMmMin[1], intent.axisSpeedMmMin[2], intent.axisSpeedMmMin[3],
        intent.axisSpeedMmMin[4], intent.axisSpeedMmMin[5], intent.axisSpeedMmMin[6], intent.axisSpeedMmMin[7]);
    if (length >= 0 && static_cast<std::size_t>(length) < sizeof(line)) RtPrintf("%s", line);
    else RtPrintf("[EDM46] event=FORMAT_ERROR part=AXES motion=0 physicalPermit=0 discharge=0\n");
}

bool NCManager::RestartEDMLiveProcessSameThread(std::uint64_t nowMs)
{
    m_edmLiveProcess.Reset();
    EDM20::FlushRequest request{}; request.mode = EDM20::FlushMode::B0;
    request.jumpHeightConfirmed = request.frame.machiningDirectionConfirmed = true;
    m_edmLiveProcessBinding = EDM26::BindProcessRecipe(m_recipe, m_edmProcessProfile, m_edmProcessObservedGap, request);
    const auto& binding = m_edmLiveProcessBinding;
    const auto input = MakeEDMLiveProcessObservationSameThread(nowMs);
    const char* blocked = !binding.ready ? EDM23::RecipeBindingErrorName(binding.error) :
        (!m_edmProcessReady ? "CONFIG_UNAVAILABLE" : (!input.shadowReady ? "SHADOW_NOT_READY" : nullptr));
    if (blocked != nullptr || !m_edmLiveProcess.Start(binding.config, input.gap, nowMs))
    {
        EndEDMLiveProcessSameThread("BLOCKED", blocked ? blocked : EDM26::LiveProcessReasonName(m_edmLiveProcess.Snapshot().reason), nowMs, binding.failedFieldId);
        AlarmManager::GetInstance().Trigger(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine);
        ChangeState(NCState::ALARM); return false;
    }
    if (!StartEDMProcessIntentSameThread(nowMs))
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "EDM46_INTENT_START"); return false;
    }
    LogEDMProcessIntentSameThread("BEGIN");
    LogEDMProcessExecutionSameThread("BEGIN");
    LogEDMRTShadowSameThread("START_QUEUED");
    m_edmLiveProcessLogMs = m_edmLiveProcessClockMs = nowMs;
    m_edmLiveProcessLoggedCycles = 0ULL;
    m_edmLiveProcessLoggedState = m_edmLiveProcessLoggedReason = m_edmLiveProcessLoggedCycleState = m_edmLiveProcessLoggedAction = 255U;
    m_edmLiveProcessConfigLogPending = true;
    char line[512]{};
    const int length = std::snprintf(line, sizeof(line),
        "[EDM26] event=BEGIN test=%llu run=%llu dispatch=%llu restart=%u COND=%u E=%u config=%llu generation=%llu source=%s AD=%u seq=%llu minMs=10000 maxMs=12000 requiredCycles=3 mode=LIVE_COND_SHADOW motion=0 physicalPermit=0 discharge=0\n",
        static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(m_gapDryRun.run),
        static_cast<unsigned long long>(m_gapDryRun.dispatch), static_cast<unsigned>(m_gapDryRun.restarts),
        static_cast<unsigned>(binding.tableId), static_cast<unsigned>(binding.eCode), static_cast<unsigned long long>(binding.config.flush.configRevision),
        static_cast<unsigned long long>(binding.generation), EDMGap::SourceName(input.gap.configuredSource),
        static_cast<unsigned>(input.gap.adIndex), static_cast<unsigned long long>(input.gap.sampleSequence));
    if (length < 0 || static_cast<std::size_t>(length) >= sizeof(line))
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM26_LOG_FORMAT"); return false; }
    RtPrintf("%s", line); return true;
}

bool NCManager::ProcessEDMLiveProcessSameThread(std::uint64_t nowMs)
{
    if (nowMs == m_edmLiveProcessClockMs)
    {
        if (++m_gapDryRun.observations >= 4096U)
        { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM26_CLOCK_NOT_ADVANCING"); return false; }
    }
    else { m_gapDryRun.observations = 0U; m_edmLiveProcessClockMs = nowMs; }
    const auto input = MakeEDMLiveProcessObservationSameThread(nowMs);
    if (m_edmProcessShadow.source != m_edmLiveProcess.Snapshot().sourceIdentity.source)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM26_SOURCE_CHANGED"); return false; }
    const auto& snapshot = m_edmLiveProcess.Observe(input);
    if (snapshot.state == EDM26::LiveProcessState::Fault)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, EDM26::LiveProcessReasonName(snapshot.reason)); return false; }
    const auto& intent = m_edmProcessMotionIntent.Observe(MakeEDMProcessIntentScopeSameThread(),
        snapshot, IsEDMCoordinatorAuthorityCurrentSameThread(), nowMs);
    if (intent.state == EDM46::IntentState::Fault)
    {
        LogEDMProcessIntentSameThread("FAULT");
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, EDM46::IntentReasonName(intent.reason)); return false;
    }
    const auto& execution = m_edmProcessExecution.Observe(intent,
        IsEDMCoordinatorAuthorityCurrentSameThread(), nowMs);
    if (execution.state == EDM47::ExecutionState::Fault)
    {
        LogEDMProcessExecutionSameThread("FAULT");
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, EDM47::ExecutionReasonName(execution.reason)); return false;
    }
    // Validate readback only after this scan's source/authority/intent gates.
    // A missing asynchronous reply may wait at most the existing 250 ms bound.
    if (!ReadEDMRTShadowSameThread(nowMs))
    {
        LogEDMRTShadowSameThread("READ_FAILED");
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "EDM48_RT_READBACK"); return false;
    }
    if (nowMs - m_edmRTShadowControl.logMs >= 1000ULL)
    {
        LogEDMRTShadowSameThread("PROGRESS");
        m_edmRTShadowControl.logMs = nowMs;
    }
    if (!SubmitEDMRTShadowSameThread())
    {
        RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED,
            m_gapDryRun.sourceLine, "EDM48_RT_SUBMIT"); return false;
    }
    const std::uint64_t wallMs = nowMs - m_gapDryRun.phaseStartMs;
    const bool normal = snapshot.process.state == EDM25::ProcessState::Running &&
        snapshot.process.shortState == EDMGapServo::ShortState::Clear &&
        (snapshot.process.action == EDM25::ProcessAction::Wait || snapshot.process.action == EDM25::ProcessAction::ServoAdvance ||
         snapshot.process.action == EDM25::ProcessAction::ServoRetreat);
    const bool boundary = snapshot.state == EDM26::LiveProcessState::Running && snapshot.acceptedFreshSample &&
        input.shadowReady && input.machiningAllowed && input.returnAllowed && normal &&
        snapshot.cycle.completedCycles >= 3ULL && snapshot.cycle.state == EDM23::AutomaticFlushCycleState::Machining &&
        snapshot.cycle.workElapsedMs == 0ULL && snapshot.cycle.virtualOffsetMm == 0.0 && snapshot.process.flushOffsetMm == 0.0;
    if (wallMs >= 10000ULL && wallMs <= 12000ULL && boundary)
    { EndEDMLiveProcessSameThread("PASS", "RETURNED_CYCLE_BOUNDARY", nowMs); return false; }
    if (wallMs >= 12000ULL)
    {
        EndEDMLiveProcessSameThread("INCOMPLETE", "WINDOW_WITHOUT_COMPLETE_BOUNDARY", nowMs);
        AlarmManager::GetInstance().Trigger(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine);
        ChangeState(NCState::ALARM); return false;
    }
    char line[512]{};
    if (m_edmLiveProcessConfigLogPending)
    {
        const auto& config = m_edmLiveProcessBinding.config;
        const int length = std::snprintf(line, sizeof(line),
            "[EDM26] event=CONFIG test=%llu E5ms=%llu E6mm=%.9g E7pct=%.9g E8pct=%.9g E9V=%.9g E21cycles=%u E22x=%.9g origin=VIRTUAL_B0 clock=WALL mode=LIVE_COND_SHADOW motion=0 physicalPermit=0 discharge=0\n",
            static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(config.flush.workTimeMs),
            config.flush.baseJumpHeightMm, config.flush.jumpSpeedOverridePercent, config.machiningOverridePercent, config.referenceV,
            static_cast<unsigned>(config.flush.deepFlushCycleInterval), config.flush.deepFlushHeightMultiplier);
        if (length < 0 || static_cast<std::size_t>(length) >= sizeof(line))
        { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM26_LOG_FORMAT"); return false; }
        m_edmLiveProcessConfigLogPending = false; RtPrintf("%s", line); return false;
    }
    const auto state = static_cast<std::uint8_t>(snapshot.state), reason = static_cast<std::uint8_t>(snapshot.reason);
    const auto cycle = static_cast<std::uint8_t>(snapshot.cycle.state), action = static_cast<std::uint8_t>(snapshot.process.action);
    if (state != m_edmLiveProcessLoggedState || reason != m_edmLiveProcessLoggedReason || cycle != m_edmLiveProcessLoggedCycleState ||
        action != m_edmLiveProcessLoggedAction || snapshot.cycle.completedCycles != m_edmLiveProcessLoggedCycles || nowMs - m_edmLiveProcessLogMs >= 1000ULL)
    {
        const int length = std::snprintf(line, sizeof(line),
            "[EDM26] event=PROGRESS test=%llu wallMs=%llu state=%u reason=%.32s action=%s process=%u short=%u cycle=%u cycles=%llu workMs=%llu speedMmMin=%.9g machiningMm=%.9g flushMm=%.9g fresh=%u seq=%llu sampleMs=%llu ageMs=%llu source=%s AD=%u mode=LIVE_COND_SHADOW motion=0 physicalPermit=0 discharge=0\n",
            static_cast<unsigned long long>(m_gapDryRun.test), static_cast<unsigned long long>(wallMs), static_cast<unsigned>(state),
            EDM26::LiveProcessReasonName(snapshot.reason), EDM25::ProcessActionName(snapshot.process.action), static_cast<unsigned>(snapshot.process.state),
            static_cast<unsigned>(snapshot.process.shortState), static_cast<unsigned>(cycle), static_cast<unsigned long long>(snapshot.cycle.completedCycles),
            static_cast<unsigned long long>(snapshot.cycle.workElapsedMs), snapshot.requestedSpeedMmPerMin, snapshot.process.machiningOffsetMm,
            snapshot.process.flushOffsetMm, snapshot.acceptedFreshSample ? 1U : 0U, static_cast<unsigned long long>(input.gap.sampleSequence),
            static_cast<unsigned long long>(input.gap.sampledAtMs), static_cast<unsigned long long>(nowMs >= input.gap.sampledAtMs ? nowMs - input.gap.sampledAtMs : 0ULL),
            EDMGap::SourceName(input.gap.configuredSource), static_cast<unsigned>(input.gap.adIndex));
        if (length < 0 || static_cast<std::size_t>(length) >= sizeof(line))
        { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, "EDM26_LOG_FORMAT"); return false; }
        RtPrintf("%s", line); LogEDMProcessIntentSameThread("PROGRESS");
        LogEDMProcessExecutionSameThread("PROGRESS");
        m_edmLiveProcessLogMs = nowMs;
        m_edmLiveProcessLoggedState = state; m_edmLiveProcessLoggedReason = reason;
        m_edmLiveProcessLoggedCycleState = cycle; m_edmLiveProcessLoggedAction = action;
        m_edmLiveProcessLoggedCycles = snapshot.cycle.completedCycles;
    }
    return false;
}

WaitConditionFunc NCManager::StartEDMPhysicalServoRTSameThread(const NCBlock& block)
{
    ServiceEDMPhysicalServoRetirementSameThread();
    const int line = m_gapDryRun.sourceLine;
    if (m_gapDryRun.active || m_edmPhysicalServoRT.retirePending ||
        m_edmZFixture.cleanupPending || m_edmZFixture.resetPending ||
        m_edmZFixture.failureAlarmPending || m_edmGapServoRT.retirePending)
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM69_PREVIOUS_SESSION_BUSY"); return nullptr; }
    m_gapDryRun = GapDryRunState{};
    m_gapDryRun.physicalServoRTTest = true; m_gapDryRun.sourceLine = line;
    m_edmPhysicalServoRT = EDMPhysicalServoRTControlState{};
    if (!IsGapDryRunBlockShapeValid(block) || !block.has('P') || block.val('P') != 55.0)
    { RejectGapDryRunSameThread(AlarmManager::G_Code_Invalid_parameter, line, "EDM69_BLOCK_SHAPE"); return nullptr; }
    std::uint64_t mask = 0ULL;
    (void)IsEDMGapServoRunReadySameThread(&mask);
    if ((mask & ~(1ULL << 9U)) || m_edmGapMaster == nullptr || !m_edmProcessReady ||
        m_currentExecutingBlockDispatchId == NC_BLOCK_DISPATCH_ID_INVALID ||
        !m_pathCoreLiveBookkeeping.currentRunToken || !GetBaseProgramCache().GetGeneration() ||
        m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)() ||
        m_edmZFixtureSessionSerial == (std::numeric_limits<std::uint64_t>::max)())
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM69_NOT_READY"); return nullptr; }
    EDMRecipe::Summary summary{};
    if (!ReadEDMRecipeSummarySameThread(summary))
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM69_RECIPE_SCOPE"); return nullptr; }
    const auto binding = EDM59::BindPhysicalServoRecipe(m_recipe, m_edmProcessProfile, m_edmProcessTuningGeneration);
    if (!binding.ready)
    { RejectGapDryRunSameThread(AlarmManager::G_Code_Invalid_parameter, line, "EDM69_RECIPE_BIND_REJECTED"); return nullptr; }
    LARGE_INTEGER frequency{};
    if (!RtQueryPerformanceFrequency(&frequency) || frequency.QuadPart <= 0 ||
        static_cast<std::uint64_t>(frequency.QuadPart) > (std::numeric_limits<std::uint64_t>::max)() / 1000000ULL)
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM69_CLOCK_FREQUENCY"); return nullptr; }
    m_gapDryRun.frequency = static_cast<std::uint64_t>(frequency.QuadPart);
    m_gapDryRun.test = ++m_gapDryRunSerial;
    m_gapDryRun.run = m_pathCoreLiveBookkeeping.currentRunToken;
    m_gapDryRun.cache = GetBaseProgramCache().GetGeneration();
    m_gapDryRun.dispatch = m_currentExecutingBlockDispatchId;
    m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
    RtPrintf("[EDM69] event=BUILD P=55 mode=PHYSICAL_RT_CONSUMER windowMs=10000 ackTimeoutMs=500 totalTimeoutMs=12000 heartbeatMs=100 terminal=TWO_FRESH_ZERO_READS hold=CANCEL_RESET_REQUIRED consumerOnly=1 axisReceipt=0 physicalLowShortVerified=0 motion=0 physicalPermit=0 discharge=0\n");
    // Read physical data before the NC clock. No console work follows START
    // until this session has stopped and cancellation has been requested.
    EDM64::Snapshot physical{};
    const auto stamp = m_edmGapMaster->ReadGapPhysicalRTShadow(physical);
    std::uint64_t nowUs = 0ULL;
    const bool clockValid = ReadEDMGapServoClockSameThread(nowUs);
    EDM64::PhysicalRTShadow::ValidateRead(physical, stamp, nowUs, clockValid, m_gapDryRun.frequency);
    if (!clockValid || !stamp.Ready() || physical.readerState != EDM64::ReaderState::Fresh ||
        physical.state != EDM64::State::Valid || !physical.output.valid ||
        physical.identity.source != EDM55::SourceKind::Physical || !physical.input.Usable() ||
        physical.physicalPermit || physical.discharge)
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM69_PHYSICAL_SOURCE_NOT_READY"); return nullptr; }
    auto& control = m_edmPhysicalServoRT;
    auto& request = control.request;
    request.capture.recipe = binding.recipe;
    // Share the existing serial without publishing an EDM65 historical recipe.
    request.capture.scope.session = ++m_edmZFixtureSessionSerial;
    request.capture.scope.run = m_gapDryRun.run; request.capture.scope.cache = m_gapDryRun.cache;
    request.capture.scope.dispatch = m_gapDryRun.dispatch; request.capture.scope.epoch = m_gapDryRun.epoch;
    request.capture.scope.owner = static_cast<std::uint32_t>(m_gapDryRun.lease.owner);
    request.capture.scope.ownerGeneration = m_gapDryRun.lease.generation;
    request.capture.capturedUs = nowUs; request.capture.qpcFrequency = m_gapDryRun.frequency;
    request.physicalIdentity = physical.identity;
    request.route = EDM63::ProducerRoute(physical.input.sample);
    request.baselineSequence = physical.input.sample.sequence;
    request.baselineCapturedUs = physical.input.sample.capturedAtUs;
    request.maxAgeUs = physical.maxAgeUs; request.maxTickGapUs = physical.maxTickGapUs;
    control.submittedUs = control.lastClockUs = control.lastFreshUs = nowUs;
    m_gapDryRun.active = true; m_gapDryRun.lastServiceMs = nowUs / 1000ULL;
    const bool started = m_motion.StartEDMPhysicalServoRT(request);
    // A transport failure may still have reserved the identity and queued a
    // cancellation. Track that reservation even when START returned false.
    control.issued = m_motion.ReadEDMPhysicalServoRTLatestSession() == request.capture.scope.session;
    if (!started || !control.issued)
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM69_START_REJECTED"); return nullptr; }
    if (!m_motion.TouchEDMPhysicalServoRT(request.capture.scope.session, nowUs))
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, line, "EDM69_INITIAL_HEARTBEAT_REJECTED"); return nullptr; }
    return WaitForGapDryRunCallback;
}

WaitConditionFunc NCManager::StartEDMPhysicalRecipeCaptureSameThread(const NCBlock& block)
{
    ServiceEDMPhysicalServoRetirementSameThread();
    if (m_edmPhysicalServoRT.retirePending)
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, m_gapDryRun.sourceLine, "EDM69_RETIREMENT_PENDING"); return nullptr; }
    const int line = m_gapDryRun.sourceLine;
    const bool window = block.has('P') && block.val('P') == 54.0;
    const char* tag = window ? "EDM68" : "EDM67";
    if (m_gapDryRun.active || m_edmZFixture.cleanupPending || m_edmZFixture.resetPending ||
        m_edmZFixture.failureAlarmPending || m_edmGapServoRT.retirePending)
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, (window ? "EDM68_PREVIOUS_SESSION_BUSY" : "EDM67_PREVIOUS_SESSION_BUSY")); return nullptr; }
    m_gapDryRun = GapDryRunState{};
    m_gapDryRun.physicalRecipeCaptureTest = true; m_gapDryRun.physicalRecipeWindowTest = window;
    m_gapDryRun.sourceLine = line;
    m_edmPhysicalRecipeCapture = EDMPhysicalRecipeCaptureState{};
    m_edmPhysicalRecipeWindow = EDM68::WindowReceipt{};
    if (!IsGapDryRunBlockShapeValid(block))
    { RejectGapDryRunSameThread(AlarmManager::G_Code_Invalid_parameter, line, (window ? "EDM68_BLOCK_SHAPE" : "EDM67_BLOCK_SHAPE")); return nullptr; }
    std::uint64_t mask = 0ULL;
    (void)IsEDMGapServoRunReadySameThread(&mask);
    // No axis application is requested. Retain program/queue/interlock
    // quiescence, but the axis-specific Arm proof (bit 9) is not required.
    if ((mask & ~(1ULL << 9U)) || m_edmGapMaster == nullptr || !m_edmProcessReady ||
        m_currentExecutingBlockDispatchId == NC_BLOCK_DISPATCH_ID_INVALID ||
        !m_pathCoreLiveBookkeeping.currentRunToken || !GetBaseProgramCache().GetGeneration() ||
        m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)() ||
        m_edmZFixtureSessionSerial == (std::numeric_limits<std::uint64_t>::max)())
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, (window ? "EDM68_NOT_READY" : "EDM67_NOT_READY")); return nullptr; }
    EDMRecipe::Summary summary{};
    if (!ReadEDMRecipeSummarySameThread(summary))
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, (window ? "EDM68_RECIPE_SCOPE" : "EDM67_RECIPE_SCOPE")); return nullptr; }
    const auto binding = EDM59::BindPhysicalServoRecipe(m_recipe, m_edmProcessProfile, m_edmProcessTuningGeneration);
    if (!binding.ready)
    {
        RtPrintf("[%s] event=RECIPE_BIND result=REJECT reason=%s field=%u motion=0 physicalPermit=0 discharge=0\n",
            tag, EDM59::BindingErrorName(binding.error), static_cast<unsigned>(binding.failedFieldId));
        RejectGapDryRunSameThread(AlarmManager::G_Code_Invalid_parameter, line, (window ? "EDM68_RECIPE_BIND_REJECTED" : "EDM67_RECIPE_BIND_REJECTED")); return nullptr;
    }
    LARGE_INTEGER frequency{};
    if (!RtQueryPerformanceFrequency(&frequency) || frequency.QuadPart <= 0 ||
        static_cast<std::uint64_t>(frequency.QuadPart) > (std::numeric_limits<std::uint64_t>::max)() / 1000000ULL)
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, (window ? "EDM68_CLOCK_FREQUENCY" : "EDM67_CLOCK_FREQUENCY")); return nullptr; }
    m_gapDryRun.frequency = static_cast<std::uint64_t>(frequency.QuadPart);
    m_gapDryRun.test = ++m_gapDryRunSerial;
    m_gapDryRun.run = m_pathCoreLiveBookkeeping.currentRunToken;
    m_gapDryRun.cache = GetBaseProgramCache().GetGeneration();
    m_gapDryRun.dispatch = m_currentExecutingBlockDispatchId;
    m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
    if (window)
        RtPrintf("[EDM68] event=BUILD P=54 mode=PHYSICAL_RECIPE_WINDOW windowMs=10000 ackTimeoutMs=500 totalTimeoutMs=12000 consumerGapMs=100 targetCheck=FROZEN_CURVE hold=CANCEL_RESET_REQUIRED reset=EXPLICIT_RECAPTURE motion=0 physicalPermit=0 discharge=0\n");
    else
        RtPrintf("[EDM67] event=BUILD P=53 mode=PHYSICAL_RECIPE_CAPTURE_ONLY receipt=TWO_FRESH_PDO_OBSERVATIONS ackTimeoutMs=500 freeze=EXPLICIT_CAPTURE hold=CANCEL_WAIT reset=RECAPTURE_EXPLICIT E8range=50..100 E9range=55..65 motion=0 physicalPermit=0 discharge=0\n");
    // Console work is complete before the two source copies and consumer
    // clock. The request's 100ms PDO adoption window starts after these reads.
    EDM64::Snapshot physical{}; EDM65::Snapshot previous{};
    const auto physicalStamp = m_edmGapMaster->ReadGapPhysicalRTShadow(physical);
    const auto previousStamp = m_edmGapMaster->ReadGapPhysicalRecipeRTShadow(previous);
    std::uint64_t nowUs = 0ULL;
    const bool clockValid = ReadEDMGapServoClockSameThread(nowUs);
    EDM64::PhysicalRTShadow::ValidateRead(physical, physicalStamp, nowUs, clockValid, m_gapDryRun.frequency);
    EDM65::PhysicalRecipeRTShadow::ValidateRead(previous, previousStamp, nowUs, clockValid, m_gapDryRun.frequency);
    if (!clockValid || !physicalStamp.Ready() || physical.readerState != EDM64::ReaderState::Fresh ||
        physical.state != EDM64::State::Valid || !physical.output.valid ||
        physical.identity.source != EDM55::SourceKind::Physical || !physical.input.Usable() ||
        !previousStamp.Ready() || previous.readerState != EDM64::ReaderState::Fresh ||
        previous.state == EDM65::State::Revoked || previous.physicalPermit || previous.discharge ||
        previous.request.scope.session > m_edmZFixtureSessionSerial)
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, (window ? "EDM68_PHYSICAL_SOURCE_NOT_READY" : "EDM67_PHYSICAL_SOURCE_NOT_READY")); return nullptr; }
    EDM65::Request request{};
    request.recipe = binding.recipe;
    // P49..P54 share one monotonically increasing capture identity.
    // A separate counter would regress EDM65 and permanently revoke it.
    request.scope.session = ++m_edmZFixtureSessionSerial;
    request.scope.run = m_gapDryRun.run; request.scope.cache = m_gapDryRun.cache;
    request.scope.dispatch = m_gapDryRun.dispatch; request.scope.epoch = m_gapDryRun.epoch;
    request.scope.owner = static_cast<std::uint32_t>(m_gapDryRun.lease.owner);
    request.scope.ownerGeneration = m_gapDryRun.lease.generation;
    request.capturedUs = nowUs; request.qpcFrequency = m_gapDryRun.frequency;
    auto& control = m_edmPhysicalRecipeCapture;
    const bool begun = window ?
        m_edmPhysicalRecipeWindow.Begin(request, previousStamp.publication, physical.input.sample.sequence) :
        control.receipt.Begin(request, previousStamp.publication, physical.input.sample.sequence);
    if (!begun)
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, (window ? "EDM68_REQUEST_INVALID" : "EDM67_REQUEST_INVALID")); return nullptr; }
    control.submittedUs = control.lastClockUs = nowUs;
    m_gapDryRun.active = true; m_gapDryRun.lastServiceMs = nowUs / 1000ULL;
    if (!m_edmGapMaster->PublishGapPhysicalRecipeRTShadow(request))
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, (window ? "EDM68_PUBLISH_REJECTED" : "EDM67_PUBLISH_REJECTED")); return nullptr; }
    // Publication alone never grants PASS. No console IO before readback.
    return WaitForGapDryRunCallback;
}

bool NCManager::ProcessEDMPhysicalRecipeCaptureSameThread()
{
    ValidateEDMPhysicalRecipeCaptureSameThread();
    if (!m_gapDryRun.active && m_gapDryRun.result != 1U)
    {
        if (m_gapDryRun.result == 2U && m_state == NCState::RUN &&
            !AlarmManager::GetInstance().HasAlarm() && m_resetContinuationPhase == ResetContinuationPhase::IDLE)
            RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, (m_gapDryRun.physicalRecipeWindowTest ? "EDM68_CANCELLED_CALLBACK_RESET_REQUIRED" : "EDM67_CANCELLED_CALLBACK_RESET_REQUIRED"));
        return false; // Cancellation never restarts or republishes the capture.
    }
    if (m_edmGapMaster == nullptr)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, (m_gapDryRun.physicalRecipeWindowTest ? "EDM68_MASTER_UNAVAILABLE" : "EDM67_MASTER_UNAVAILABLE")); return false; }
    if (m_gapDryRun.physicalRecipeWindowTest) return ProcessEDMPhysicalRecipeWindowSameThread();
    EDM65::Snapshot raw{};
    const auto stamp = m_edmGapMaster->ReadGapPhysicalRecipeRTShadow(raw);
    std::uint64_t nowUs = 0ULL;
    const bool clockValid = ReadEDMGapServoClockSameThread(nowUs);
    auto& receipt = m_edmPhysicalRecipeCapture.receipt;
    const auto result = receipt.Observe(raw, stamp, nowUs, clockValid, m_gapDryRun.frequency);
    if (result == EDM67::Status::Invalid)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, receipt.ReasonText()); return false; }
    if (result != EDM67::Status::Ready) return false;
    // An outer completion guard may poll again before retiring this block.
    // Keep the same request and revalidate fresh evidence without reprinting
    // PASS or mistaking the previous copied bank for a new observation.
    if (m_gapDryRun.result == 1U) return true;
    const auto& value = receipt.Current(); const auto& request = receipt.Request();
    m_gapDryRun.active = false; m_gapDryRun.result = 1U;
    char text[480]{};
    int size = std::snprintf(text, sizeof(text),
        "[EDM67] event=PASS result=PASS reason=FROZEN_RECIPE_TWO_FRESH_PDO P=53 session=%llu recipePub=%llu publication=%llu seq=%llu freshSamples=%llu E8=%.6f E9=%.6f receipt=PHYSICAL_SHADOW_ONLY axisReceipt=0 physicalLowShortVerified=0 motion=0 physicalPermit=0 discharge=0\n",
        static_cast<unsigned long long>(request.scope.session), static_cast<unsigned long long>(value.recipePublication),
        static_cast<unsigned long long>(value.publication), static_cast<unsigned long long>(value.sourceSequence),
        static_cast<unsigned long long>(value.coverage.freshSamples), request.recipe.overridePercent, request.recipe.referenceV);
    if (size >= 0 && static_cast<std::size_t>(size) < sizeof(text)) RtPrintf("%s", text);
    else RtPrintf("[EDM67] event=LOG reason=LOG_FORMAT motion=0 physicalPermit=0 discharge=0\n");
    size = std::snprintf(text, sizeof(text),
        "[EDM67] event=VALUES session=%llu recipePub=%llu source=PHYSICAL sourceId=%llu sourceGen=%llu calibration=%llu rawV=%.6f filteredV=%.6f targetMmMin=%.6f advisoryMmMin=%.6f short=%u feedInhibit=%u appliedToMotion=0 physicalPermit=0 discharge=0\n",
        static_cast<unsigned long long>(request.scope.session), static_cast<unsigned long long>(value.recipePublication),
        static_cast<unsigned long long>(value.identity.sourceId), static_cast<unsigned long long>(value.identity.sourceGeneration),
        static_cast<unsigned long long>(value.identity.calibrationRevision), value.output.rawVoltageV, value.output.filteredVoltageV,
        value.output.targetMmPerMin, value.output.speedMmPerMin, value.output.shortActive ? 1U : 0U,
        value.output.feedInhibited ? 1U : 0U);
    if (size >= 0 && static_cast<std::size_t>(size) < sizeof(text)) RtPrintf("%s", text);
    else RtPrintf("[EDM67] event=LOG reason=LOG_FORMAT motion=0 physicalPermit=0 discharge=0\n");
    return true;
}

bool NCManager::ProcessEDMPhysicalRecipeWindowSameThread()
{
    EDM65::Snapshot raw{};
    const auto stamp = m_edmGapMaster->ReadGapPhysicalRecipeRTShadow(raw);
    std::uint64_t nowUs = 0ULL;
    const bool clockValid = ReadEDMGapServoClockSameThread(nowUs);
    auto& receipt = m_edmPhysicalRecipeWindow;
    const auto result = receipt.Observe(raw, stamp, nowUs, clockValid, m_gapDryRun.frequency);
    if (result == EDM68::Status::Invalid)
    { RejectGapDryRunSameThread(AlarmManager::GAP_INPUT_TEST_FAILED, m_gapDryRun.sourceLine, receipt.ReasonText()); return false; }
    if (result != EDM68::Status::Ready) return false;
    if (m_gapDryRun.result == 1U) return true; // Freshly revalidated outer completion poll.
    const auto& value = receipt.Current(); const auto& request = receipt.Request();
    const auto& report = receipt.Report();
    m_gapDryRun.active = false; m_gapDryRun.result = 1U;
    char text[480]{};
    int size = std::snprintf(text, sizeof(text),
        "[EDM68] event=PASS result=PASS P=54 session=%llu recipePub=%llu observedMs=%llu reads=%llu pdoDelta=%llu E8=%.6f E9=%.6f targetCheck=PASS receipt=PHYSICAL_SHADOW_ONLY axisReceipt=0 physicalLowShortVerified=0 motion=0 physicalPermit=0 discharge=0\n",
        static_cast<unsigned long long>(request.scope.session), static_cast<unsigned long long>(value.recipePublication),
        static_cast<unsigned long long>(report.durationUs / 1000ULL), static_cast<unsigned long long>(report.freshObservations),
        static_cast<unsigned long long>(report.sourcePdoDelta), request.recipe.overridePercent, request.recipe.referenceV);
    if (size >= 0 && static_cast<std::size_t>(size) < sizeof(text)) RtPrintf("%s", text);
    else RtPrintf("[EDM68] event=LOG reason=LOG_FORMAT motion=0 physicalPermit=0 discharge=0\n");
    size = std::snprintf(text, sizeof(text),
        "[EDM68] event=SOURCE session=%llu source=PHYSICAL sourceId=%llu sourceGen=%llu calibration=%llu firstSeq=%llu lastSeq=%llu maxSourceAgeUs=%llu maxConsumerGapUs=%llu scope=NC_FRESH_READS motion=0 physicalPermit=0 discharge=0\n",
        static_cast<unsigned long long>(request.scope.session), static_cast<unsigned long long>(value.identity.sourceId),
        static_cast<unsigned long long>(value.identity.sourceGeneration), static_cast<unsigned long long>(value.identity.calibrationRevision),
        static_cast<unsigned long long>(report.firstSourceSequence), static_cast<unsigned long long>(report.lastSourceSequence),
        static_cast<unsigned long long>(report.maxSourceAgeUs), static_cast<unsigned long long>(report.maxConsumerGapUs));
    if (size >= 0 && static_cast<std::size_t>(size) < sizeof(text)) RtPrintf("%s", text);
    else RtPrintf("[EDM68] event=LOG reason=LOG_FORMAT motion=0 physicalPermit=0 discharge=0\n");
    size = std::snprintf(text, sizeof(text),
        "[EDM68] event=RANGE session=%llu rawMinV=%.6f rawMaxV=%.6f targetMinMmMin=%.6f targetMaxMmMin=%.6f advisoryMinMmMin=%.6f advisoryMaxMmMin=%.6f maxTargetError=%.9g shortReads=%llu inhibitReads=%llu scope=NC_FRESH_READS appliedToMotion=0 physicalPermit=0 discharge=0\n",
        static_cast<unsigned long long>(request.scope.session), report.minRawV, report.maxRawV,
        report.minTargetMmPerMin, report.maxTargetMmPerMin, report.minSpeedMmPerMin, report.maxSpeedMmPerMin,
        report.maxTargetErrorMmPerMin, static_cast<unsigned long long>(report.shortObservations),
        static_cast<unsigned long long>(report.feedInhibitObservations));
    if (size >= 0 && static_cast<std::size_t>(size) < sizeof(text)) RtPrintf("%s", text);
    else RtPrintf("[EDM68] event=LOG reason=LOG_FORMAT motion=0 physicalPermit=0 discharge=0\n");
    return true;
}

WaitConditionFunc NCManager::StartEDMGapDynamicSameThread(const NCBlock& block)
{
    ServiceEDMPhysicalServoRetirementSameThread();
    if (m_edmPhysicalServoRT.retirePending)
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, m_gapDryRun.sourceLine, "EDM69_RETIREMENT_PENDING"); return nullptr; }
    const int line = m_gapDryRun.sourceLine;
    const auto profile = static_cast<EDM28::Profile>(static_cast<unsigned>(block.val('P')));
    const bool servoFlush = EDM28::IsGapServoFlushProfile(profile);
    const bool physicalServo = EDM28::IsPhysicalServoProfile(profile);
    const bool physicalFlush = EDM28::IsPhysicalServoFlushProfile(profile);
    const bool physicalRetreat = EDM28::IsPhysicalRetreatProfile(profile);
    const bool physicalRetreatCycle = EDM28::IsPhysicalRetreatCycleProfile(profile);
    const bool physicalEndurance = EDM28::IsPhysicalBidirectionalFlushEnduranceProfile(profile);
    const bool physicalBidirectionalFlush = EDM28::IsPhysicalBidirectionalFlushFamilyProfile(profile);
    const bool physicalBidirectional = EDM28::IsPhysicalBidirectionalProfile(profile) || physicalBidirectionalFlush;
    const bool physicalNegative = EDM28::IsPhysicalNegativeProfile(profile);
    const char* flushBuild = physicalEndurance ? "EDM76" : physicalBidirectionalFlush ? "EDM75" : physicalFlush ? "EDM71" : servoFlush ? "EDM65" : "EDM58";
    const char* servoBuild = physicalEndurance ? "EDM76" : physicalBidirectionalFlush ? "EDM75" : physicalBidirectional ? "EDM74" : physicalRetreatCycle ? "EDM73" : physicalRetreat ? "EDM72" : physicalFlush ? "EDM71" : servoFlush ? "EDM65" : "EDM59";
    if (m_gapDryRun.active || m_edmZFixture.cleanupPending || m_edmZFixture.resetPending ||
        m_edmZFixture.failureAlarmPending || (physicalServo && m_edmGapServoRT.retirePending))
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM56_PREVIOUS_SESSION_BUSY"); return nullptr; }
    const bool sequenceMode = physicalBidirectionalFlush && block.has('Q') &&
        (block.val('Q') == 1.0 || block.val('Q') == 2.0);
    const bool strictSequenceMode = sequenceMode && block.val('Q') == 2.0;
    if (!IsGapDryRunBlockShapeValid(block))
    { RejectEDMGapAdmissionSameThread(AlarmManager::G_Code_Invalid_parameter, line, "EDM56_REQUIRES_STANDALONE_M30"); return nullptr; }
    if (m_edmPhysicalSequencePlan.run != m_pathCoreLiveBookkeeping.currentRunToken)
        InvalidateEDMPhysicalSequencePlanSameThread();
    const bool capturePlan = strictSequenceMode && !m_edmPhysicalSequencePlan.plan.valid;
    unsigned terminalOffset = 0U;
    if (sequenceMode)
    {
        // A pure authoring rejection belongs to this dispatch, never the
        // retained session from a completed/cancelled previous program.
        const auto rejectAdmission = [this, line](int alarmCode, const char* reason)
        { RejectEDMGapAdmissionSameThread(alarmCode, line, reason); };
        // Authoring is validated from the immutable parse cache. Evaluated Q1
        // must never promote a macro/expression into an explicit opt-in.
        const int pc = GetActiveDispatchPC();
        const NCProgramCacheLine* current = pc >= 0 ? GetBaseProgramCache().TryGetLine(pc) : nullptr;
        if (!current || !EDM77::NumericFixtureMatches(current->parsedBlock, block) ||
            pc > (std::numeric_limits<int>::max)() - 128)
        { rejectAdmission(AlarmManager::G_Code_Invalid_parameter, "EDM77_LITERAL_SEQUENCE_REQUIRED"); return nullptr; }
        const auto suffix = EDM77::ValidateSuffix(current->parsedBlock, [this, pc](unsigned offset) noexcept -> const NCParsedBlock*
        {
            const NCProgramCacheLine* next = GetBaseProgramCache().TryGetLine(pc + static_cast<int>(offset));
            return next ? &next->parsedBlock : nullptr;
        });
        if (!suffix.valid)
        {
            RtPrintf("[EDM77] event=SEQUENCE_ADMISSION result=REJECT error=%u offset=%u armIssued=0 discharge=0\n",
                static_cast<unsigned>(suffix.error), static_cast<unsigned>(suffix.errorOffset));
            rejectAdmission(AlarmManager::G_Code_Invalid_parameter, "EDM77_SEQUENCE_SUFFIX_REJECTED"); return nullptr;
        }
        terminalOffset = suffix.terminalOffset;
        // EDM78 checks the remaining recipe program in isolated fixed scratch.
        // It grants no motion/selection authority: every real leg still performs
        // its current stopped-boundary, catalog and frozen-recipe checks below.
        if (!m_edmProcessReady)
        {
            RtPrintf("[EDM78] event=SEQUENCE_PREFLIGHT result=REJECT reason=PROCESS_UNAVAILABLE offset=0 fixture=1 armIssued=0 discharge=0\n");
            rejectAdmission(AlarmManager::PATH_EXECUTION_NOT_READY, "EDM78_PROCESS_UNAVAILABLE"); return nullptr;
        }
        const auto preflight = EDM78::Preflight(m_recipe, m_edmRecipeSequencePreview,
            current->parsedBlock, [this, pc](unsigned offset) noexcept -> const NCParsedBlock*
        {
            const NCProgramCacheLine* next = GetBaseProgramCache().TryGetLine(pc + static_cast<int>(offset));
            return next ? &next->parsedBlock : nullptr;
        }, m_edmProcessProfile, m_edmProcessTuningGeneration,
            capturePlan ? &m_edmPhysicalSequencePlan.plan : nullptr);
        if (!preflight.valid)
        {
            RtPrintf("[EDM78] event=SEQUENCE_PREFLIGHT result=REJECT reason=%s offset=%u fixture=%u field=%u recipe=%s control=%s flush=%s servo=%s armIssued=0 discharge=0\n",
                EDM78::ErrorName(preflight.error), static_cast<unsigned>(preflight.errorOffset),
                static_cast<unsigned>(preflight.fixtureOrdinal), static_cast<unsigned>(preflight.failedFieldId),
                EDMRecipe::ErrorName(preflight.recipeError), EDMRecipeControl::ReasonName(preflight.controlError),
                EDM58::BindingErrorName(preflight.flushError), EDM59::BindingErrorName(preflight.servoError));
            rejectAdmission(AlarmManager::G_Code_Invalid_parameter, "EDM78_SEQUENCE_PREFLIGHT_REJECTED"); return nullptr;
        }
        RtPrintf("[EDM78] event=SEQUENCE_PREFLIGHT result=ACCEPT fixtures=%u operations=%u initialGeneration=%llu previewGeneration=%llu liveUnchanged=1 armIssued=0 discharge=0\n",
            static_cast<unsigned>(preflight.fixtureCount), static_cast<unsigned>(preflight.operationCount),
            static_cast<unsigned long long>(preflight.initialGeneration), static_cast<unsigned long long>(preflight.finalGeneration));
        RtPrintf("[EDM77] event=SEQUENCE_ADMISSION result=VALIDATED P=%u Q=%u futureFixtures=%u physicalLines=%u terminalOffset=%u run=%llu cache=%llu dispatch=%llu armIssued=0 discharge=0\n",
            static_cast<unsigned>(profile), strictSequenceMode ? 2U : 1U, static_cast<unsigned>(suffix.futureFixtures), static_cast<unsigned>(suffix.physicalLineCount),
            static_cast<unsigned>(suffix.terminalOffset), static_cast<unsigned long long>(m_pathCoreLiveBookkeeping.currentRunToken),
            static_cast<unsigned long long>(GetBaseProgramCache().GetGeneration()), static_cast<unsigned long long>(m_currentExecutingBlockDispatchId));
    }
    else if (!IsEDMZFixtureM30ContinuationSameThread())
    { RejectEDMGapAdmissionSameThread(AlarmManager::G_Code_Invalid_parameter, line, "EDM56_REQUIRES_STANDALONE_M30"); return nullptr; }
    // The old release must be re-read and proved before any previous state is
    // discarded. A new dispatch intentionally differs from the old scope.
    if (m_gapDryRun.physicalZFixtureTest && m_gapDryRun.run != 0ULL &&
        m_gapDryRun.run == m_pathCoreLiveBookkeeping.currentRunToken &&
        (sequenceMode || m_edmGapDynamic.sequenceMode) &&
        (!sequenceMode || !IsEDMPhysicalSequenceBoundarySameThread()))
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM77_PREVIOUS_RELEASE_REQUIRED"); return nullptr; }
    if (m_gapDryRun.physicalZFixtureTest && m_gapDryRun.run != 0ULL &&
        m_gapDryRun.run == m_pathCoreLiveBookkeeping.currentRunToken && m_edmGapDynamic.sequenceMode &&
        (m_edmGapDynamic.strictSequenceMode != strictSequenceMode ||
            (m_edmGapDynamic.strictSequenceMode && capturePlan)))
    { RejectEDMGapAdmissionSameThread(AlarmManager::G_Code_Invalid_parameter, line, "EDM79_PLAN_LOST_OR_MODE_CHANGED"); return nullptr; }
    if (m_edmPhysicalSequencePlan.plan.valid && !strictSequenceMode)
    { RejectEDMGapAdmissionSameThread(AlarmManager::G_Code_Invalid_parameter, line, "EDM79_SEQUENCE_MODE_CHANGED"); return nullptr; }
    if (strictSequenceMode)
    {
        auto& plan = m_edmPhysicalSequencePlan;
        const int pc = GetActiveDispatchPC();
        if (capturePlan)
        {
            plan.run = m_pathCoreLiveBookkeeping.currentRunToken;
            plan.cache = GetBaseProgramCache().GetGeneration();
            plan.epoch = m_motion.GetCurrentExecutionEpoch(); plan.lease = m_programMotionLease;
            plan.firstPC = pc; plan.next = 0U;
            plan.dispatch = NC_BLOCK_DISPATCH_ID_INVALID; plan.legPending = false;
        }
        const bool scope = plan.plan.valid && plan.plan.count > 0U &&
            plan.plan.count <= EDM77::MaximumFixtures && plan.next < plan.plan.count &&
            !plan.legPending && plan.run != 0ULL && plan.run == m_pathCoreLiveBookkeeping.currentRunToken &&
            plan.cache != 0ULL && plan.cache == GetBaseProgramCache().GetGeneration() &&
            plan.epoch == m_motion.GetCurrentExecutionEpoch() && plan.lease.Matches(m_programMotionLease) &&
            m_motion.IsMotionOwnerLeaseCurrent(plan.lease) && plan.firstPC >= 0 && pc >= plan.firstPC &&
            plan.firstPC <= (std::numeric_limits<int>::max)() - static_cast<int>(EDM77::MaximumPhysicalLines) &&
            pc <= (std::numeric_limits<int>::max)() - static_cast<int>(EDM77::MaximumPhysicalLines) &&
            plan.plan.terminalOffset > 0U && plan.plan.terminalOffset <= EDM77::MaximumPhysicalLines &&
            pc + static_cast<int>(terminalOffset) == plan.firstPC + static_cast<int>(plan.plan.terminalOffset) &&
            m_currentExecutingBlockDispatchId != NC_BLOCK_DISPATCH_ID_INVALID &&
            (capturePlan || m_currentExecutingBlockDispatchId > plan.dispatch);
        const auto match = scope ? EDM79::MatchCheckpoint(plan.plan.checkpoints[plan.next],
            static_cast<unsigned>(pc - plan.firstPC), static_cast<unsigned>(profile), m_recipe,
            m_edmProcessProfile, m_edmProcessTuningGeneration) : EDM79::MatchError::Source;
        if (!scope || match != EDM79::MatchError::None)
        {
            RtPrintf("[EDM79] event=PLAN_REJECT result=REJECT reason=%s fixture=%u count=%u run=%llu cache=%llu dispatch=%llu newSession=0 armIssued=0 discharge=0\n",
                scope ? EDM79::MatchErrorName(match) : "PLAN_SCOPE", plan.next + 1U, plan.plan.count,
                static_cast<unsigned long long>(m_pathCoreLiveBookkeeping.currentRunToken),
                static_cast<unsigned long long>(GetBaseProgramCache().GetGeneration()),
                static_cast<unsigned long long>(m_currentExecutingBlockDispatchId));
            RejectEDMGapAdmissionSameThread(AlarmManager::G_Code_Invalid_parameter, line, "EDM79_PLAN_CHANGED"); return nullptr;
        }
        if (capturePlan)
            RtPrintf("[EDM79] event=PLAN_CAPTURE result=PASS Q=2 count=%u run=%llu cache=%llu firstPC=%d terminalPC=%d generation=%llu armIssued=0 discharge=0\n",
                plan.plan.count, static_cast<unsigned long long>(plan.run), static_cast<unsigned long long>(plan.cache),
                plan.firstPC, plan.firstPC + static_cast<int>(plan.plan.terminalOffset),
                static_cast<unsigned long long>(plan.plan.checkpoints[0].key.generation));
        RtPrintf("[EDM79] event=PLAN_MATCH result=PASS fixture=%u count=%u P=%u run=%llu dispatch=%llu generation=%llu armIssued=0 discharge=0\n",
            plan.next + 1U, plan.plan.count, static_cast<unsigned>(profile), static_cast<unsigned long long>(plan.run),
            static_cast<unsigned long long>(m_currentExecutingBlockDispatchId),
            static_cast<unsigned long long>(plan.plan.checkpoints[plan.next].key.generation));
        plan.dispatch = m_currentExecutingBlockDispatchId; plan.legPending = true;
    }
    m_gapDryRun = GapDryRunState{}; m_gapDryRun.physicalZFixtureTest = true; m_gapDryRun.sourceLine = line;
    m_edmZFixture = EDMZFixtureState{}; m_edmZFixture.config.profile = profile;
    m_edmGapDynamic = EDMGapDynamicState{}; m_edmGapDynamic.active = true; m_edmGapDynamic.sequenceMode = sequenceMode;
    m_edmGapDynamic.strictSequenceMode = strictSequenceMode;
    m_edmGapRapid = EDMGapRapidState{}; m_edmGapFeedUpdate = EDMGapFeedUpdateState{};
    m_edmGapShortReplan = EDMGapShortReplanState{};
    // Capture the immutable seed before formal readiness; a changed lease or
    // epoch may wait for HOLD classification, but never silently rebinds Arm.
    m_gapDryRun.epoch = m_motion.GetCurrentExecutionEpoch(); m_gapDryRun.lease = m_programMotionLease;
    std::uint64_t mask = 0ULL; MotionStopSettleSnapshot raw{};
    (void)IsEDMGapServoRunReadySameThread(&mask, &raw);
    if (m_currentExecutingBlockDispatchId == NC_BLOCK_DISPATCH_ID_INVALID ||
        !m_pathCoreLiveBookkeeping.currentRunToken || !GetBaseProgramCache().GetGeneration()) mask |= 1ULL << 10U;
    if (m_gapDryRunSerial == (std::numeric_limits<std::uint64_t>::max)() ||
        m_edmZFixtureSessionSerial == (std::numeric_limits<std::uint64_t>::max)()) mask |= 1ULL << 11U;
    if (mask & ~(1ULL << 9U))
    { LogEDMGapDynamicAdmissionSameThread(mask, raw, "REJECT"); RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM56_ADMISSION_REJECTED"); return nullptr; }
    LARGE_INTEGER frequency{};
    if (!RtQueryPerformanceFrequency(&frequency) || frequency.QuadPart <= 0 ||
        static_cast<std::uint64_t>(frequency.QuadPart) > (std::numeric_limits<std::uint64_t>::max)() / 1000ULL)
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM56_CLOCK_FREQUENCY"); return nullptr; }
    m_gapDryRun.frequency = static_cast<std::uint64_t>(frequency.QuadPart);
    std::uint64_t now = 0ULL;
    if (!ReadGapDryRunClockSameThread(now) || !ReadEDMZFixtureFeedbackSameThread(false))
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM56_NO_COHERENT_SOURCE"); return nullptr; }
    auto& s = m_edmZFixture; auto& d = m_edmGapDynamic; const auto& f = s.feedback;
    if (f.capHeld || !std::isfinite(f.pulsePerMm) || f.pulsePerMm < 1000.0 || f.pulsePerMm > 1000000000.0 ||
        !f.axisMapGeneration || !f.configGeneration || !std::isfinite(f.referenceOffsetPulse) ||
        (f.hardwareSign != 1 && f.hardwareSign != -1))
    { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, "EDM56_PREFLIGHT_BLOCKED"); return nullptr; }
    // Capture the effective owner-thread condition once, before any physical
    // Arm submission. Admission waits and HOLD preserve this same snapshot;
    // a fresh G180 after RESET captures a new condition generation.
    if (physicalBidirectionalFlush || physicalFlush || EDM57::IsBoundProfile(static_cast<unsigned>(profile)))
    {
        EDMRecipe::Summary summary{};
        if (!m_edmProcessReady || !ReadEDMRecipeSummarySameThread(summary))
        { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, physicalEndurance ? "EDM76_FLUSH_CONDITION_SCOPE" : physicalBidirectionalFlush ? "EDM75_FLUSH_CONDITION_SCOPE" : physicalFlush ? "EDM71_FLUSH_CONDITION_SCOPE" : servoFlush ? "EDM60_FLUSH_CONDITION_SCOPE" : "EDM58_CONDITION_SCOPE"); return nullptr; }
        const auto binding = physicalBidirectionalFlush ? EDM75::BindPhysicalBidirectionalFlushRecipe(m_recipe, m_edmProcessProfile.machineProfileId) :
            physicalFlush ? EDM71::BindPhysicalFlushRecipe(m_recipe, m_edmProcessProfile.machineProfileId) :
            EDM58::BindPhysicalFlushRecipe(m_recipe, m_edmProcessProfile.machineProfileId);
        if (!binding.ready)
        {
            RtPrintf("[%s] event=RECIPE_BIND result=REJECT reason=%s field=%u armIssued=0 discharge=0\n",
                flushBuild, EDM58::BindingErrorName(binding.error), static_cast<unsigned>(binding.failedFieldId));
            RejectGapDryRunSameThread(AlarmManager::G_Code_Invalid_parameter, line, physicalEndurance ? "EDM76_FLUSH_BIND_REJECTED" : physicalBidirectionalFlush ? "EDM75_FLUSH_BIND_REJECTED" : physicalFlush ? "EDM71_FLUSH_BIND_REJECTED" : servoFlush ? "EDM60_FLUSH_CONDITION_BIND_REJECTED" : "EDM58_CONDITION_BIND_REJECTED"); return nullptr;
        }
        s.config.flush = binding.recipe;
        if (servoFlush) { s.config.flush.revision = 3U; s.config.flush.workDepthMm = .125; }
    }
    if (EDM28::IsGapConditionServoProfile(profile))
    {
        EDMRecipe::Summary summary{};
        if (!m_edmProcessReady || !ReadEDMRecipeSummarySameThread(summary))
        { RejectGapDryRunSameThread(AlarmManager::PATH_EXECUTION_NOT_READY, line, physicalEndurance ? "EDM76_SERVO_CONDITION_SCOPE" : physicalBidirectionalFlush ? "EDM75_SERVO_CONDITION_SCOPE" : physicalBidirectional ? "EDM74_SERVO_CONDITION_SCOPE" : physicalRetreatCycle ? "EDM73_SERVO_CONDITION_SCOPE" : physicalRetreat ? "EDM72_SERVO_CONDITION_SCOPE" : physicalFlush ? "EDM71_SERVO_CONDITION_SCOPE" : servoFlush ? "EDM60_SERVO_CONDITION_SCOPE" : "EDM59_CONDITION_SCOPE"); return nullptr; }
        const auto binding = physicalBidirectional ? EDM74::BindPhysicalBidirectionalRecipe(m_recipe, m_edmProcessProfile, m_edmProcessTuningGeneration) :
            physicalNegative ? EDM72::BindPhysicalRetreatRecipe(m_recipe, m_edmProcessProfile, m_edmProcessTuningGeneration) :
            EDM59::BindPhysicalServoRecipe(m_recipe, m_edmProcessProfile, m_edmProcessTuningGeneration);
        if (!binding.ready)
        {
            RtPrintf("[%s] event=RECIPE_BIND result=REJECT reason=%s field=%u armIssued=0 discharge=0\n",
                servoBuild, EDM59::BindingErrorName(binding.error), static_cast<unsigned>(binding.failedFieldId));
            RejectGapDryRunSameThread(AlarmManager::G_Code_Invalid_parameter, line, physicalEndurance ? "EDM76_SERVO_BIND_REJECTED" : physicalBidirectionalFlush ? "EDM75_SERVO_BIND_REJECTED" : physicalBidirectional ? "EDM74_SERVO_BIND_REJECTED" : physicalRetreatCycle ? "EDM73_SERVO_BIND_REJECTED" : physicalRetreat ? "EDM72_SERVO_BIND_REJECTED" : physicalFlush ? "EDM71_SERVO_BIND_REJECTED" : servoFlush ? "EDM60_SERVO_CONDITION_BIND_REJECTED" : "EDM59_CONDITION_BIND_REJECTED"); return nullptr;
        }
        s.config.servoRecipe = binding.recipe;
    }
    m_gapDryRun.active = true; m_gapDryRun.test = ++m_gapDryRunSerial;
    m_gapDryRun.run = m_pathCoreLiveBookkeeping.currentRunToken;
    m_gapDryRun.cache = GetBaseProgramCache().GetGeneration(); m_gapDryRun.dispatch = m_currentExecutingBlockDispatchId;
    s.scope.session = ++m_edmZFixtureSessionSerial;
    s.scope.runGeneration = m_gapDryRun.run; s.scope.cacheGeneration = m_gapDryRun.cache;
    s.scope.dispatchGeneration = m_gapDryRun.dispatch; s.scope.executionEpoch = m_gapDryRun.epoch;
    s.scope.owner = static_cast<std::uint8_t>(m_gapDryRun.lease.owner); s.scope.ownerGeneration = m_gapDryRun.lease.generation;
    s.scope.axisMapGeneration = f.axisMapGeneration; s.scope.configGeneration = f.configGeneration;
    s.config.pulsePerMm = f.pulsePerMm; s.config.referenceOffsetPulse = f.referenceOffsetPulse; s.config.hardwareSign = f.hardwareSign;
    s.config.feedMmMin = 30.0; s.config.pdoCapMmS = .5; s.config.targetHalfMm = .25; s.config.outerHalfMm = .4;
    if (physicalServo)
    { s.config.feedMmMin = EDM70::FeedMmMin; s.config.pdoCapMmS = EDM70::PdoCapMmS;
      s.config.targetHalfMm = EDM70::DepthMm; s.config.outerHalfMm = EDM70::OuterMm; }
    s.config.accelerationTimeSec = s.config.decelerationTimeSec = .3;
    s.config.positionToleranceMm = s.config.excursionToleranceMm = .001;
    if (!physicalServo && EDM28::IsGapFlushProfile(profile) && !EDM57::IsBoundProfile(static_cast<unsigned>(profile)))
        s.config.flush = EDM57::FixtureRecipe();
    if (servoFlush && !EDM28::ValidateGapServoFlushConfig(s.config))
    {
        RtPrintf("[EDM65] event=RECIPE_BIND result=REJECT reason=COMBINED_IDENTITY_OR_TRAVEL armIssued=0 discharge=0\n");
        RejectGapDryRunSameThread(AlarmManager::G_Code_Invalid_parameter, line, "EDM60_COMBINED_BIND_REJECTED"); return nullptr;
    }
    if (physicalBidirectionalFlush && (!EDM75::ValidateRecipe(s.config.flush) ||
        !EDM71::SameAdmission(s.config.flush, s.config.servoRecipe)))
    { RejectGapDryRunSameThread(AlarmManager::G_Code_Invalid_parameter, line, physicalEndurance ? "EDM76_COMBINED_BIND_REJECTED" : "EDM75_COMBINED_BIND_REJECTED"); return nullptr; }
    if (physicalFlush && (!EDM71::ValidateRecipe(s.config.flush) ||
        !EDM71::SameAdmission(s.config.flush, s.config.servoRecipe)))
    { RejectGapDryRunSameThread(AlarmManager::G_Code_Invalid_parameter, line, "EDM71_COMBINED_BIND_REJECTED"); return nullptr; }
    d.frozenConfig = s.config; d.admissionPending = true; d.admissionStartMs = d.validationClockMs = now;
    // Capture the already validated immutable SIM recipe once per G180. HOLD
    // cannot recapture it. Publication failure is diagnostic, never admission.
    if (servoFlush)
    {
        EDM65::Request request{};
        request.recipe = d.frozenConfig.servoRecipe;
        request.scope.session = s.scope.session;
        request.scope.run = s.scope.runGeneration;
        request.scope.cache = s.scope.cacheGeneration;
        request.scope.dispatch = s.scope.dispatchGeneration;
        request.scope.epoch = s.scope.executionEpoch;
        request.scope.owner = s.scope.owner;
        request.scope.ownerGeneration = s.scope.ownerGeneration;
        request.qpcFrequency = m_gapDryRun.frequency;
        const bool captureClockValid = m_gapDryRun.lastTicks <=
            static_cast<std::uint64_t>((std::numeric_limits<std::int64_t>::max)()) &&
            EDMAnalogInput::Publisher::QpcMicroseconds(static_cast<std::int64_t>(m_gapDryRun.lastTicks),
                m_gapDryRun.frequency, request.capturedUs) && request.capturedUs != 0ULL;
        const bool published = captureClockValid && m_edmGapMaster != nullptr &&
            m_edmGapMaster->PublishGapPhysicalRecipeRTShadow(request);
        RtPrintf("[EDM65] event=RECIPE_SHADOW_CAPTURE session=%llu result=%s reason=%s capturedUs=%llu qpcHz=%llu capture=FROZEN_SIM_ADMISSION capturedNotActive=1 appliedToMotion=0 physicalPermit=0 discharge=0\n",
            static_cast<unsigned long long>(request.scope.session), published ? "PUBLISHED" : "REJECTED",
            !captureClockValid ? "CLOCK_RANGE" : m_edmGapMaster == nullptr ? "NO_MASTER" : published ? "NONE" : "PUBLISH_REJECTED",
            static_cast<unsigned long long>(request.capturedUs),
            static_cast<unsigned long long>(request.qpcFrequency));
    }
    s.phase = EDMZFixturePhase::PreArm;
    s.lastFreshMs = s.lastClockMs = s.phaseStartMs = m_gapDryRun.lastServiceMs = now;
    if (physicalServo)
    {
        if (physicalBidirectionalFlush)
        {
            RtPrintf("[%s] event=BUILD build=%s P=%u source=PHYSICAL axis=Z workMs=%llu commandWorkHalfMm=.03 maxLiftMm=.03 commandZMinMm=-.03 commandZMaxMm=.06 motion=LIVE_AD_BIDIRECTIONAL_FLUSH minCycles=%u originHandoff=RESET_ACTIVE_I_ONCE_PER_SETTLE_ENTRY return=BOUNDED_FIXTURE anchor=FIRST_ARM settleUs=50000 feedCapMmMin=5 pdoCapMmS=.1 outerHalfMm=.1 lowShort=FORMAL_STOP directionCoverage=OBSERVED_ONLY physicalShortRecoveryVerified=0 hold=CANCEL_RESET_REQUIRED stop=PHYSICAL_RELEASE discharge=0\n", flushBuild, flushBuild, static_cast<unsigned>(profile),
                static_cast<unsigned long long>(EDM69::DurationUsFor(physicalEndurance ? EDM69::RecipeFamily::PhysicalBidirectionalEndurance : EDM69::RecipeFamily::PhysicalBidirectional) / 1000ULL),
                EDM70::MinimumSignedFlushCyclesFor(static_cast<unsigned>(profile)));
            const auto& flush = s.config.flush; const auto& servo = s.config.servoRecipe;
            const auto& key = flush.binding;
            char text[768]{};
            const int size = std::snprintf(text, sizeof(text), "[%s] event=RECIPE_FROZEN P=%u session=%llu table=%u E=%u catalog=%u tableRev=%u profile=%u definition=%u generation=%llu fields=%u/%u/%u/%u/%u/%u/%u processRev=%u tuningGen=%llu E5us=%llu E6=%.6f E7=%.3f E8=%.3f E9=%.3f E21=%u E22=%.6f maxLiftMm=%.6f freeze=ADMISSION apply=NEXT_G180 source=PHYSICAL discharge=0\n",
                flushBuild, static_cast<unsigned>(profile), static_cast<unsigned long long>(s.scope.session), key.tableId, static_cast<unsigned>(key.eCode),
                key.catalogRevision, key.tableRevision, key.profileId, key.definitionRevision,
                static_cast<unsigned long long>(key.generation), static_cast<unsigned>(key.fieldIds[0]),
                static_cast<unsigned>(key.fieldIds[1]), static_cast<unsigned>(key.fieldIds[2]),
                static_cast<unsigned>(servo.binding.fieldIds[0]), static_cast<unsigned>(servo.binding.fieldIds[1]),
                static_cast<unsigned>(key.fieldIds[3]), static_cast<unsigned>(key.fieldIds[4]),
                servo.binding.processRevision, static_cast<unsigned long long>(servo.binding.tuningGeneration),
                static_cast<unsigned long long>(flush.workTimeUs), flush.baseHeightMm, flush.jumpSpeedOverridePercent,
                servo.overridePercent, servo.referenceV, flush.deepInterval, flush.deepMultiplier, EDM71::MaximumLiftHeight(flush));
            if (size >= 0 && static_cast<std::size_t>(size) < sizeof(text)) RtPrintf("%s", text);
            else RtPrintf("[%s] event=RECIPE_FROZEN result=FAIL reason=LOG_FORMAT discharge=0\n", flushBuild);
        }
        else if (physicalBidirectional)
        {
            RtPrintf("[EDM74] event=BUILD build=EDM74_FIX2 originHandoff=RESET_ACTIVE_I_ONCE originDiag=RETAINED_ONLY P=60 source=PHYSICAL axis=Z workMs=10000 commandHalfMm=.03 motion=LIVE_AD_BIDIRECTIONAL return=BOUNDED_FIXTURE anchor=FIRST_ARM originSettleUs=50000 feedCapMmMin=5 pdoCapMmS=.1 outerHalfMm=.1 lowShort=FORMAL_STOP directionCoverage=OBSERVED_ONLY physicalShortRecoveryVerified=0 hold=CANCEL_RESET_REQUIRED stop=PHYSICAL_RELEASE discharge=0\n");
            const auto& recipe = s.config.servoRecipe; const auto& key = recipe.binding;
            char text[640]{};
            const int size = std::snprintf(text, sizeof(text), "[EDM74] event=RECIPE_FROZEN P=60 session=%llu table=%u E=%u catalog=%u tableRev=%u profile=%u definition=%u generation=%llu fields=%u/%u processRev=%u tuningGen=%llu E8=%.6f E9=%.6f freeze=ADMISSION apply=NEXT_G180 source=PHYSICAL motion=LIVE_AD_BIDIRECTIONAL return=BOUNDED_FIXTURE anchor=FIRST_ARM discharge=0\n",
                static_cast<unsigned long long>(s.scope.session), key.tableId, static_cast<unsigned>(key.eCode),
                key.catalogRevision, key.tableRevision, key.profileId, key.definitionRevision,
                static_cast<unsigned long long>(key.generation), static_cast<unsigned>(key.fieldIds[0]),
                static_cast<unsigned>(key.fieldIds[1]), key.processRevision,
                static_cast<unsigned long long>(key.tuningGeneration), recipe.overridePercent, recipe.referenceV);
            if (size >= 0 && static_cast<std::size_t>(size) < sizeof(text)) RtPrintf("%s", text);
            else RtPrintf("[EDM74] event=RECIPE_FROZEN result=FAIL reason=LOG_FORMAT discharge=0\n");
        }
        else if (physicalRetreatCycle)
        {
            RtPrintf("[EDM73] event=BUILD build=EDM73 P=59 source=PHYSICAL axis=Z workMs=10000 retreatMm=.03 motion=REAL_AD_NEGATIVE_RETREAT_CYCLE return=BOUNDED_FIXTURE anchor=FIRST_ARM minCycles=2 endpointSettleUs=50000 originSettleUs=50000 feedCapMmMin=5 pdoCapMmS=.1 outerHalfMm=.1 lowShort=FORMAL_STOP physicalShortRecoveryVerified=0 hold=CANCEL_RESET_REQUIRED stop=PHYSICAL_RELEASE discharge=0\n");
            const auto& recipe = s.config.servoRecipe; const auto& key = recipe.binding;
            char text[640]{};
            const int size = std::snprintf(text, sizeof(text), "[EDM73] event=RECIPE_FROZEN P=59 session=%llu table=%u E=%u catalog=%u tableRev=%u profile=%u definition=%u generation=%llu fields=%u/%u processRev=%u tuningGen=%llu E8=%.6f E9=%.6f freeze=ADMISSION apply=NEXT_G180 source=PHYSICAL motion=REAL_AD_NEGATIVE_RETREAT_CYCLE return=BOUNDED_FIXTURE anchor=FIRST_ARM discharge=0\n",
                static_cast<unsigned long long>(s.scope.session), key.tableId, static_cast<unsigned>(key.eCode),
                key.catalogRevision, key.tableRevision, key.profileId, key.definitionRevision,
                static_cast<unsigned long long>(key.generation), static_cast<unsigned>(key.fieldIds[0]),
                static_cast<unsigned>(key.fieldIds[1]), key.processRevision,
                static_cast<unsigned long long>(key.tuningGeneration), recipe.overridePercent, recipe.referenceV);
            if (size >= 0 && static_cast<std::size_t>(size) < sizeof(text)) RtPrintf("%s", text);
            else RtPrintf("[EDM73] event=RECIPE_FROZEN result=FAIL reason=LOG_FORMAT discharge=0\n");
        }
        else if (physicalRetreat)
        {
            RtPrintf("[EDM72] event=BUILD build=EDM72 P=58 source=PHYSICAL axis=Z workMs=10000 retreatMm=.03 motion=REAL_AD_NEGATIVE_RETREAT endpoint=HOLD_NO_RETURN feedCapMmMin=5 pdoCapMmS=.1 outerHalfMm=.1 short=FORMAL_STOP physicalShortRecoveryVerified=0 hold=CANCEL_RESET_REQUIRED stop=PHYSICAL_RELEASE discharge=0\n");
            const auto& recipe = s.config.servoRecipe; const auto& key = recipe.binding;
            char text[640]{};
            const int size = std::snprintf(text, sizeof(text), "[EDM72] event=RECIPE_FROZEN P=58 session=%llu table=%u E=%u catalog=%u tableRev=%u profile=%u definition=%u generation=%llu fields=%u/%u processRev=%u tuningGen=%llu E8=%.6f E9=%.6f freeze=ADMISSION apply=NEXT_G180 source=PHYSICAL motion=REAL_AD_NEGATIVE_RETREAT endpoint=HOLD_NO_RETURN discharge=0\n",
                static_cast<unsigned long long>(s.scope.session), key.tableId, static_cast<unsigned>(key.eCode),
                key.catalogRevision, key.tableRevision, key.profileId, key.definitionRevision,
                static_cast<unsigned long long>(key.generation), static_cast<unsigned>(key.fieldIds[0]),
                static_cast<unsigned>(key.fieldIds[1]), key.processRevision,
                static_cast<unsigned long long>(key.tuningGeneration), recipe.overridePercent, recipe.referenceV);
            if (size >= 0 && static_cast<std::size_t>(size) < sizeof(text)) RtPrintf("%s", text);
            else RtPrintf("[EDM72] event=RECIPE_FROZEN result=FAIL reason=LOG_FORMAT discharge=0\n");
        }
        else if (physicalFlush)
        {
            RtPrintf("[EDM71] event=BUILD build=EDM71 P=57 source=PHYSICAL axis=Z workMs=10000 depthMm=.03 forward=AD_SERVO flush=E5_E6_E7_E21_E22 feedCapMmMin=5 pdoCapMmS=.1 outerHalfMm=.1 lowInput=FORMAL_STOP hold=CANCEL_RESET_REQUIRED stop=PHYSICAL_RELEASE discharge=0\n");
            const auto& flush = s.config.flush; const auto& key = flush.binding;
            char line[768]{};
            const int size = std::snprintf(line, sizeof(line), "[EDM71] event=RECIPE_FROZEN P=57 session=%llu table=%u E=%u catalog=%u tableRev=%u profile=%u definition=%u generation=%llu fields=%u/%u/%u/%u/%u E5us=%llu E6=%.6f E7=%.3f E8=%.3f E9=%.3f E21=%u E22=%.6f maxLiftMm=%.6f freeze=ADMISSION apply=NEXT_G180 discharge=0\n",
                static_cast<unsigned long long>(s.scope.session), key.tableId, static_cast<unsigned>(key.eCode),
                key.catalogRevision, key.tableRevision, key.profileId, key.definitionRevision,
                static_cast<unsigned long long>(key.generation), static_cast<unsigned>(key.fieldIds[0]),
                static_cast<unsigned>(key.fieldIds[1]), static_cast<unsigned>(key.fieldIds[2]),
                static_cast<unsigned>(key.fieldIds[3]), static_cast<unsigned>(key.fieldIds[4]),
                static_cast<unsigned long long>(flush.workTimeUs), flush.baseHeightMm, flush.jumpSpeedOverridePercent,
                s.config.servoRecipe.overridePercent, s.config.servoRecipe.referenceV,
                flush.deepInterval, flush.deepMultiplier, EDM71::MaximumLiftHeight(flush));
            if (size >= 0 && static_cast<std::size_t>(size) < sizeof(line)) RtPrintf("%s", line);
            else RtPrintf("[EDM71] event=RECIPE_FROZEN result=FAIL reason=LOG_FORMAT discharge=0\n");
        }
        else
            RtPrintf("[EDM70] event=BUILD build=EDM70_FIX1 P=56 source=PHYSICAL axis=Z workMs=10000 depthMm=.03 forward=AD_SERVO return=FIXTURE feedCapMmMin=5 pdoCapMmS=.1 outerHalfMm=.1 lowInput=FORMAL_STOP hold=CANCEL_RESET_REQUIRED stop=PHYSICAL_RELEASE discharge=0\n");
        EDM64::Snapshot physical{}; EDM53::Stamp stamp{};
        if (m_edmGapMaster) stamp = m_edmGapMaster->ReadGapPhysicalRTShadow(physical);
        std::uint64_t nowUs = 0ULL;
        const bool clockValid = ReadEDMGapServoClockSameThread(nowUs);
        EDM64::PhysicalRTShadow::ValidateRead(physical, stamp, nowUs, clockValid, m_gapDryRun.frequency);
        if (!m_edmGapMaster || !clockValid || !stamp.Ready() ||
            physical.readerState != EDM64::ReaderState::Fresh || physical.state != EDM64::State::Valid ||
            !physical.output.valid || !physical.input.Usable() || physical.physicalPermit || physical.discharge)
        {
            // Admission boundary only, before Arm: retain the actual source
            // failure instead of labelling an unusable receipt as low voltage.
            RtPrintf("[%s] event=SOURCE_REJECT session=%llu state=%s reason=%s reader=%s seq=%llu observedUs=%llu nowUs=%llu clock=%u maxAgeUs=%llu maxTickGapUs=%llu armIssued=0 discharge=0\n",
                physicalEndurance ? "EDM76" : physicalBidirectionalFlush ? "EDM75" : physicalBidirectional ? "EDM74" : physicalRetreatCycle ? "EDM73" : physicalRetreat ? "EDM72" : physicalFlush ? "EDM71" : "EDM70-FIX1", static_cast<unsigned long long>(s.scope.session), EDM64::StateName(physical.state),
                EDM64::ReasonName(physical.reason), EDM64::ReaderStateName(physical.readerState),
                static_cast<unsigned long long>(physical.input.sample.sequence),
                static_cast<unsigned long long>(physical.observedUs), static_cast<unsigned long long>(nowUs),
                clockValid ? 1U : 0U, static_cast<unsigned long long>(physical.maxAgeUs),
                static_cast<unsigned long long>(physical.maxTickGapUs));
            EndEDMZFixtureSameThread("FAIL", physicalEndurance ? "EDM76_PHYSICAL_SOURCE_NOT_READY" : physicalBidirectionalFlush ? "EDM75_PHYSICAL_SOURCE_NOT_READY" : physicalBidirectional ? "EDM74_PHYSICAL_SOURCE_NOT_READY" : physicalRetreatCycle ? "EDM73_PHYSICAL_SOURCE_NOT_READY" : physicalRetreat ? "EDM72_PHYSICAL_SOURCE_NOT_READY" : physicalFlush ? "EDM71_PHYSICAL_SOURCE_NOT_READY" : "EDM70_PHYSICAL_SOURCE_NOT_READY"); return nullptr;
        }
        const double inputV = static_cast<double>(physical.input.conversion.voltageMv)/1000.0;
        if (physicalBidirectional ? !EDM74::InputAllowed(s.config.servoRecipe, inputV) :
            physicalNegative ? !EDM72::NegativeInputAllowed(s.config.servoRecipe, inputV) :
            inputV <= s.config.servoRecipe.referenceV + s.config.servoRecipe.curve.deadbandV)
        { EndEDMZFixtureSameThread("FAIL", physicalEndurance ? "EDM76_PHYSICAL_INPUT_REJECTED" : physicalBidirectionalFlush ? "EDM75_PHYSICAL_INPUT_REJECTED" : physicalBidirectional ? "EDM74_PHYSICAL_INPUT_REJECTED" : physicalRetreatCycle ? "EDM73_PHYSICAL_NEGATIVE_INPUT_REQUIRED" : physicalRetreat ? "EDM72_PHYSICAL_NEGATIVE_INPUT_REQUIRED" :
            physicalFlush ? "EDM71_PHYSICAL_HIGH_INPUT_REQUIRED" : "EDM70_PHYSICAL_HIGH_INPUT_REQUIRED"); return nullptr; }
        auto& request = s.config.physicalInput;
        request.recipeFamily = physicalEndurance ? EDM69::RecipeFamily::PhysicalBidirectionalEndurance : physicalBidirectional ? EDM69::RecipeFamily::PhysicalBidirectional : physicalNegative ? EDM69::RecipeFamily::PhysicalRetreat : EDM69::RecipeFamily::Legacy;
        request.capture.recipe = s.config.servoRecipe;
        request.capture.scope.session = s.scope.session; request.capture.scope.run = s.scope.runGeneration;
        request.capture.scope.cache = s.scope.cacheGeneration; request.capture.scope.dispatch = s.scope.dispatchGeneration;
        request.capture.scope.epoch = s.scope.executionEpoch; request.capture.scope.owner = s.scope.owner;
        request.capture.scope.ownerGeneration = s.scope.ownerGeneration;
        request.capture.capturedUs = nowUs; request.capture.qpcFrequency = m_gapDryRun.frequency;
        request.physicalIdentity = physical.identity; request.route = EDM63::ProducerRoute(physical.input.sample);
        request.baselineSequence = physical.input.sample.sequence; request.baselineCapturedUs = physical.input.sample.capturedAtUs;
        request.maxAgeUs = physical.maxAgeUs; request.maxTickGapUs = physical.maxTickGapUs;
        if (!EDM70::ValidConfig(s.config) || !EDM70::MatchesScope(request, s.scope))
        { EndEDMZFixtureSameThread("FAIL", physicalEndurance ? "EDM76_PHYSICAL_BINDING_REJECTED" : physicalBidirectionalFlush ? "EDM75_PHYSICAL_BINDING_REJECTED" : physicalBidirectional ? "EDM74_PHYSICAL_BINDING_REJECTED" : physicalRetreatCycle ? "EDM73_PHYSICAL_BINDING_REJECTED" : physicalRetreat ? "EDM72_PHYSICAL_BINDING_REJECTED" : physicalFlush ? "EDM71_PHYSICAL_BINDING_REJECTED" : "EDM70_PHYSICAL_BINDING_REJECTED"); return nullptr; }
        d.frozenConfig = s.config;
        d.admissionStartMs = d.validationClockMs = nowUs / 1000ULL;
        s.lastFreshMs = s.lastClockMs = s.phaseStartMs = m_gapDryRun.lastServiceMs = nowUs / 1000ULL;
        return WaitForGapDryRunCallback;
    }
    if (EDM57::IsBoundProfile(static_cast<unsigned>(profile)))
    {
        const auto& r = s.config.flush; const auto& key = r.binding;
        char text[480]{};
        const int size = std::snprintf(text, sizeof(text), "[%s] event=RECIPE_FROZEN session=%llu table=%u E=%u catalog=%u tableRev=%u profile=%u definition=%u generation=%llu fields=%u/%u/%u/%u/%u E5us=%llu E6=%.6f E7=%.6f E21=%u E22=%.6f freeze=ADMISSION apply=NEXT_G180 discharge=0\n",
            flushBuild, static_cast<unsigned long long>(s.scope.session), key.tableId, static_cast<unsigned>(key.eCode),
            key.catalogRevision, key.tableRevision, key.profileId, key.definitionRevision,
            static_cast<unsigned long long>(key.generation), static_cast<unsigned>(key.fieldIds[0]),
            static_cast<unsigned>(key.fieldIds[1]), static_cast<unsigned>(key.fieldIds[2]),
            static_cast<unsigned>(key.fieldIds[3]), static_cast<unsigned>(key.fieldIds[4]),
            static_cast<unsigned long long>(r.workTimeUs), r.baseHeightMm, r.jumpSpeedOverridePercent,
            r.deepInterval, r.deepMultiplier);
        if (size >= 0 && static_cast<std::size_t>(size) < sizeof(text)) RtPrintf("%s", text);
        else RtPrintf("[%s] event=RECIPE_FROZEN result=FAIL reason=LOG_FORMAT discharge=0\n", flushBuild);
    }
    if (EDM28::IsGapConditionServoProfile(profile))
    {
        const auto& r = s.config.servoRecipe; const auto& key = r.binding; const auto& c = r.curve;
        char text[480]{};
        int size = std::snprintf(text, sizeof(text), "[%s] event=RECIPE_FROZEN session=%llu table=%u E=%u catalog=%u tableRev=%u profile=%u definition=%u generation=%llu fields=%u/%u processRev=%u tuningGen=%llu E8=%.6f E9=%.6f freeze=ADMISSION apply=NEXT_G180 source=SIM discharge=0\n",
            servoBuild, static_cast<unsigned long long>(s.scope.session), key.tableId, static_cast<unsigned>(key.eCode),
            key.catalogRevision, key.tableRevision, key.profileId, key.definitionRevision,
            static_cast<unsigned long long>(key.generation), static_cast<unsigned>(key.fieldIds[0]),
            static_cast<unsigned>(key.fieldIds[1]), key.processRevision,
            static_cast<unsigned long long>(key.tuningGeneration), r.overridePercent, r.referenceV);
        if (size >= 0 && static_cast<std::size_t>(size) < sizeof(text)) RtPrintf("%s", text);
        else RtPrintf("[%s] event=RECIPE_FROZEN result=FAIL reason=LOG_FORMAT discharge=0\n", servoBuild);
        const auto advance = EDMGapServo::Evaluate(c, 80.0, r.referenceV, r.overridePercent);
        const auto returning = EDMGapServo::Evaluate(c, 50.0, r.referenceV, r.overridePercent);
        size = std::snprintf(text, sizeof(text), "[%s] event=SERVO_CURVE session=%llu P=%.6g/%.6g/%.6g N=%.6g/%.6g/%.6g Pbreak=%.6g/%.6g Nbreak=%.6g/%.6g scale=%.9g deadband=%.6g feedCap=%.6g retreatCap=%.6g short=%.6g v80=%.6g v50=%.6g v20=%.6g shortGate=25/5 %s cycles=6 limitsConfirmed=%u fixturePermit=1 discharge=0\n",
            servoBuild, static_cast<unsigned long long>(s.scope.session), c.positiveGain[0], c.positiveGain[1], c.positiveGain[2],
            c.negativeGain[0], c.negativeGain[1], c.negativeGain[2], c.positiveBreakV[0], c.positiveBreakV[1],
            c.negativeBreakV[0], c.negativeBreakV[1], c.cuttingScaleMmPerVoltMin, c.deadbandV,
            c.maxFeedMmPerMin, c.maxRetreatMmPerMin, c.shortRetreatMmPerMin,
            advance.limitedSpeedMmPerMin, returning.limitedSpeedMmPerMin,
            -c.shortRetreatMmPerMin * r.overridePercent / 100.0,
            servoFlush ? "work=GAP_SERVO flush=FROZEN_E5_E6_E7_E21_E22" : "fixtureDepth=.03 shortClear=.005", c.limitsConfirmed ? 1U : 0U);
        if (size >= 0 && static_cast<std::size_t>(size) < sizeof(text)) RtPrintf("%s", text);
        else RtPrintf("[%s] event=SERVO_CURVE result=FAIL reason=LOG_FORMAT discharge=0\n", servoBuild);
    }
    if (servoFlush)
    {
        RtPrintf("[EDM65] event=BUILD build=EDM65 P=%u work=GAP_SERVO flush=RT_RECIPE sourcePort=61 sourceBinding=FROZEN_SIM sourceId=60 rawPort=62 rawSource=SIM rawCalibration=SYNTHETIC physicalInput=REJECTED physicalPort=63 physicalMode=SHADOW_ONLY rtPhysicalPort=64 rtPhysicalMode=RT_SHADOW_ONLY recipeShadowPort=65 recipeIdentity=COMMON_ADMISSION originSettle=1 dwellUs=50000 positionMm=.00075 excursionMm=.00025 pdoGate=0 endpointIdleGain=1 retiredFaultGuard=1 discharge=0\n",
            static_cast<unsigned>(profile));
        RtPrintf("[EDM66-FIX2] event=DIAGNOSTICS observationOnly=1 liveFixtureConsole=OFF armToFreshRelease=QUIET boundary=PASS_HOLD_FAULT readyCadenceMs=1000 logCost=1 deadmanMs=50 masks=REF:1,POSITION:2,RANGE:8,TIME:16,TICKS:32 pdoHighInformational=1 discharge=0\n");
        RtPrintf("[EDM66] event=BUILD feature=PHYSICAL_RECIPE_COVERAGE accumulate=PDO log=READY_1HZ reset=NEW_RECIPE historyOnly=1 direction=TARGET_NOT_AXIS appliedToMotion=0 physicalPermit=0 discharge=0\n");
    }
    else
    {
        RtPrintf("[EDM59-FIX2] event=BUILD P=%u originSettle=%u dwellUs=50000 positionMm=.00075 excursionMm=.00025 pdoGate=0 pdoHighMmS=.002 endpointIdleGain=%u retiredFaultGuard=1 discharge=0\n",
            static_cast<unsigned>(profile), EDM28::IsGapConditionServoProfile(profile) ? 1U : 0U,
            EDM28::IsGapConditionServoProfile(profile) ? 1U : 0U);
        RtPrintf("[EDM59-FIX2] event=DIAGNOSTICS observationOnly=1 cadenceMs=1000 masks=REF:1,POSITION:2,RANGE:8,TIME:16,TICKS:32 pdoHighInformational=1 discharge=0\n");
    }
    LogEDMGapDynamicAdmissionSameThread(mask, raw, mask ? "WAIT_FORMAL_RT" : "READY_WAIT_ARM");
    return WaitForGapDryRunCallback;
}
