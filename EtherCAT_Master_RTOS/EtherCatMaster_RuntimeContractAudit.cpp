#include "EtherCatMaster.h"

#include <rtapi.h>

#include <cstring>

#include "GlobalConfig.h"


// ============================================================================
// Stage 9B - Generic Runtime Configuration Contract Safety Gate
//
// This is the generic replacement candidate for the old ProductCode-specific
// Stage5C equivalence gate.
//
// It validates Runtime XML against ITSELF and against the already-built
// Process Image contract.
//
// It deliberately does NOT know:
// - Delta
// - A3E
// - 8124
// - 6002
// - 7062
// - any VendorId/ProductCode
//
// No hardware read.
// No hardware write.
// Gate ownership is handled by the Stage 9B startup router.
// ============================================================================

namespace
{
    bool StringEquals(
        const char* a,
        const char* b)
    {
        return
            a != nullptr &&
            b != nullptr &&
            strcmp(
                a,
                b) ==
            0;
    }


    bool IsKnownTransition(
        const char* transition)
    {
        return
            StringEquals(
                transition,
                "INIT") ||
            StringEquals(
                transition,
                "PRE_OP") ||
            StringEquals(
                transition,
                "SAFE_OP") ||
            StringEquals(
                transition,
                "OP");
    }


    bool HasSmIndex(
        const EtherCatSlave& slave,
        int smIndex)
    {
        for (const auto& sm :
            slave.runtimeSyncManagers)
        {
            if (sm.index ==
                smIndex)
            {
                return
                    true;
            }
        }


        return
            false;
    }


    const EtherCatRuntimeSyncManagerConfig* FindSm(
        const EtherCatSlave& slave,
        int smIndex)
    {
        for (const auto& sm :
            slave.runtimeSyncManagers)
        {
            if (sm.index ==
                smIndex)
            {
                return
                    &sm;
            }
        }


        return
            nullptr;
    }


    bool PhysicalRangeCoveredByEnabledSms(
        const EtherCatSlave& slave,
        uint16_t startAddress,
        uint32_t byteCount)
    {
        if (byteCount ==
            0U)
        {
            return
                true;
        }


        for (uint32_t byteOffset = 0;
            byteOffset <
            byteCount;
            byteOffset++)
        {
            const uint32_t address =
                (uint32_t)
                startAddress +
                byteOffset;


            bool covered =
                false;


            for (const auto& sm :
                slave.runtimeSyncManagers)
            {
                if (!sm.enabled ||
                    sm.length ==
                    0U)
                {
                    continue;
                }


                const uint32_t smStart =
                    sm.startAddress;


                const uint32_t smEnd =
                    smStart +
                    sm.length;


                if (address >=
                    smStart &&
                    address <
                    smEnd)
                {
                    covered =
                        true;

                    break;
                }
            }


            if (!covered)
            {
                return
                    false;
            }
        }


        return
            true;
    }


    // ========================================================================
    // Stage 11G.2 - Multi-SyncManager / Multi-FMMU Contract Helpers
    //
    // One logical Process Image direction may be backed by 1..N physical
    // SyncManagers.  The application-facing logical image remains compact;
    // only the ESC physical side is segmented.
    // ========================================================================

    struct RuntimeFmmuExpectedSegment
    {
        int smIndex = -1;
        uint16_t physicalStartAddress = 0;
        uint32_t bitSize = 0;
    };


    std::vector<RuntimeFmmuExpectedSegment>
        BuildExpectedFmmuSegments(
            const EtherCatSlave& slave,
            const std::vector<EtherCatRuntimePdoConfig>& pdos)
    {
        std::vector<RuntimeFmmuExpectedSegment> result;


        for (const auto& pdo :
            pdos)
        {
            if (pdo.syncManagerIndex <
                0 ||
                pdo.bitSize ==
                0U)
            {
                continue;
            }


            RuntimeFmmuExpectedSegment* existing =
                nullptr;


            for (auto& segment :
                result)
            {
                if (segment.smIndex ==
                    pdo.syncManagerIndex)
                {
                    existing =
                        &segment;

                    break;
                }
            }


            if (existing !=
                nullptr)
            {
                existing->bitSize +=
                    pdo.bitSize;

                continue;
            }


            const EtherCatRuntimeSyncManagerConfig* sm =
                FindSm(
                    slave,
                    pdo.syncManagerIndex);


            if (sm ==
                nullptr)
            {
                continue;
            }


            RuntimeFmmuExpectedSegment segment;

            segment.smIndex =
                pdo.syncManagerIndex;

            segment.physicalStartAddress =
                sm->startAddress;

            segment.bitSize =
                pdo.bitSize;


            result.push_back(
                segment);
        }


        return
            result;
    }


    bool ValidateProcessDataPhysicalSegments(
        const EtherCatSlave& slave,
        const std::vector<EtherCatRuntimePdoConfig>& pdos,
        uint32_t totalBitLength,
        uint16_t legacyStartAddress)
    {
        if (totalBitLength ==
            0U)
        {
            return
                true;
        }


        const std::vector<RuntimeFmmuExpectedSegment> segments =
            BuildExpectedFmmuSegments(
                slave,
                pdos);


        // Compatibility fallback for an older/custom Runtime profile that
        // does not expose PDO -> SyncManager ownership.
        if (segments.empty())
        {
            const uint32_t totalBytes =
                (
                    totalBitLength +
                    7U
                    ) /
                8U;


            return
                PhysicalRangeCoveredByEnabledSms(
                    slave,
                    legacyStartAddress,
                    totalBytes);
        }


        uint64_t coveredBits =
            0U;


        for (const auto& segment :
            segments)
        {
            const EtherCatRuntimeSyncManagerConfig* sm =
                FindSm(
                    slave,
                    segment.smIndex);


            if (sm ==
                nullptr ||
                !sm->enabled ||
                segment.bitSize ==
                0U)
            {
                return
                    false;
            }


            const uint32_t requiredBytes =
                (
                    segment.bitSize +
                    7U
                    ) /
                8U;


            if (sm->length <
                requiredBytes ||
                sm->startAddress !=
                segment.physicalStartAddress)
            {
                return
                    false;
            }


            coveredBits +=
                segment.bitSize;
        }


        return
            coveredBits ==
            (uint64_t)
            totalBitLength;
    }


    bool RuntimeFmmuMatchesExpectedSegment(
        const EtherCatRuntimeFmmuConfig& fmmu,
        uint64_t logicalBitOffset,
        uint32_t bitSize,
        uint16_t physicalStartAddress,
        uint8_t expectedType)
    {
        if (bitSize ==
            0U)
        {
            return
                false;
        }


        const uint32_t expectedLogicalStartAddress =
            (uint32_t)(
                logicalBitOffset /
                8U);


        const uint8_t expectedLogicalStartBit =
            (uint8_t)(
                logicalBitOffset %
                8U);


        const uint16_t expectedLogicalLength =
            (uint16_t)(
                (
                    (uint32_t)
                    expectedLogicalStartBit +
                    bitSize +
                    7U
                    ) /
                8U);


        const uint8_t expectedLogicalEndBit =
            (uint8_t)(
                (
                    (uint32_t)
                    expectedLogicalStartBit +
                    bitSize -
                    1U
                    ) %
                8U);


        return
            fmmu.type ==
            expectedType &&
            fmmu.logicalStartAddress ==
            expectedLogicalStartAddress &&
            fmmu.logicalLength ==
            expectedLogicalLength &&
            fmmu.logicalStartBit ==
            expectedLogicalStartBit &&
            fmmu.logicalEndBit ==
            expectedLogicalEndBit &&
            fmmu.physicalStartAddress ==
            physicalStartAddress &&
            fmmu.physicalStartBit ==
            0U;
    }


    uint32_t SumPdoBits(
        const std::vector<EtherCatRuntimePdoConfig>& pdos)
    {
        uint32_t total =
            0;


        for (const auto& pdo :
            pdos)
        {
            total +=
                pdo.bitSize;
        }


        return
            total;
    }


    bool ValidatePdoCollection(
        const EtherCatSlave& slave,
        const std::vector<EtherCatRuntimePdoConfig>& pdos,
        bool requireSmIndex,
        int& pdoObjects,
        int& pdoEntries,
        int& errors)
    {
        bool allPass =
            true;


        for (const auto& pdo :
            pdos)
        {
            pdoObjects++;


            bool pass =
                pdo.index !=
                0U;


            if (requireSmIndex)
            {
                if (pdo.syncManagerIndex <
                    0 ||
                    !HasSmIndex(
                        slave,
                        pdo.syncManagerIndex))
                {
                    pass =
                        false;
                }
            }
            else if (pdo.syncManagerIndex >=
                0)
            {
                if (!HasSmIndex(
                    slave,
                    pdo.syncManagerIndex))
                {
                    pass =
                        false;
                }
            }


            uint32_t entryBits =
                0;


            for (const auto& entry :
                pdo.entries)
            {
                pdoEntries++;


                const uint32_t expectedMapping =
                    (
                        (uint32_t)
                        entry.index <<
                        16
                        ) |
                    (
                        (uint32_t)
                        entry.subIndex <<
                        8
                        ) |
                    (
                        (uint32_t)
                        entry.bitLength
                        );


                if (entry.index ==
                    0U ||
                    entry.bitLength ==
                    0U ||
                    entry.mappingValue !=
                    expectedMapping)
                {
                    pass =
                        false;
                }


                entryBits +=
                    entry.bitLength;
            }


            const uint32_t expectedBytes =
                (
                    entryBits +
                    7U
                    ) /
                8U;


            if (entryBits !=
                pdo.bitSize ||
                expectedBytes !=
                pdo.byteSize)
            {
                pass =
                    false;
            }


            if (!pass)
            {
                errors++;
                allPass =
                    false;
            }
        }


        return
            allPass;
    }
}


// ============================================================================
// AuditRuntimeConfigurationContractGeneric
// ============================================================================

bool EtherCatMaster::AuditRuntimeConfigurationContractGeneric()
{
    if (m_pEni ==
        nullptr)
    {
        RtPrintf(
            "[RUNTIME-CONTRACT] "
            "Runtime Config is not attached | Result:FAIL\n");


        return
            false;
    }


    const auto& slaves =
        m_pEni->GetSlaves();


    int profileCount =
        0;


    int bindingCount =
        0;


    int watchdogCount =
        0;


    int dcCount =
        0;


    int smCount =
        0;


    int pdoObjectCount =
        0;


    int pdoEntryCount =
        0;


    int fmmuCount =
        0;


    int mailboxSlaveCount =
        0;


    int mailboxEndpointCount =
        0;


    int initCommandCount =
        0;


    int appliedInitCommandCount =
        0;


    int dcReferenceCount =
        0;


    int errors =
        0;


    uint32_t maxBindingEnd =
        0;


    RtPrintf(
        "\n"
        "============================================================\n"
        "[RUNTIME-CONTRACT] BEGIN | "
        "Stage:9B | Mode:SAFETY_GATE_SCHEMA_AUDIT | "
        "ProductCodeBaseline:NO | HardwareRead:NO | HardwareWrite:NO\n"
        "============================================================\n");


    for (int slaveIdx = 0;
        slaveIdx <
        (int)slaves.size();
        slaveIdx++)
    {
        const EtherCatSlave& slave =
            slaves[(size_t)slaveIdx];


        bool pass =
            true;


        // ====================================================================
        // Identity completeness
        // ====================================================================

        if (!slave.hasRevision ||
            !slave.hasConfiguredAddress)
        {
            pass =
                false;
        }


        // ====================================================================
        // Runtime Profile
        // ====================================================================

        if (slave.runtimeProfile.present &&
            slave.runtimeProfile.hasPdoMappingMode)
        {
            profileCount++;
        }
        else
        {
            pass =
                false;
        }


        const bool fixedMapping =
            StringEquals(
                slave.runtimeProfile.pdoMappingMode,
                "Fixed");


        const bool configurableMapping =
            StringEquals(
                slave.runtimeProfile.pdoMappingMode,
                "Configurable");


        if (!fixedMapping &&
            !configurableMapping)
        {
            pass =
                false;
        }


        // ====================================================================
        // Process Image Binding
        // ====================================================================

        const auto& binding =
            slave.runtimeProcessImageBinding;


        if (binding.present)
        {
            bindingCount++;
        }
        else
        {
            pass =
                false;
        }


        const uint32_t outputBytes =
            (
                slave.outputBitLength +
                7U
                ) /
            8U;


        const uint32_t inputBytes =
            (
                slave.inputBitLength +
                7U
                ) /
            8U;


        if (binding.outputBytes !=
            outputBytes ||
            binding.inputBytes !=
            inputBytes)
        {
            pass =
                false;
        }


        if (outputBytes ==
            0U)
        {
            if (binding.outputOffset !=
                -1)
            {
                pass =
                    false;
            }
        }
        else
        {
            if (binding.outputOffset <
                0)
            {
                pass =
                    false;
            }
            else
            {
                const uint32_t end =
                    (uint32_t)
                    binding.outputOffset +
                    outputBytes;


                if (end >
                    maxBindingEnd)
                {
                    maxBindingEnd =
                        end;
                }
            }
        }


        if (inputBytes ==
            0U)
        {
            if (binding.inputOffset !=
                -1)
            {
                pass =
                    false;
            }
        }
        else
        {
            if (binding.inputOffset <
                0)
            {
                pass =
                    false;
            }
            else
            {
                const uint32_t end =
                    (uint32_t)
                    binding.inputOffset +
                    inputBytes;


                if (end >
                    maxBindingEnd)
                {
                    maxBindingEnd =
                        end;
                }
            }
        }


        // ====================================================================
        // Watchdog
        // ====================================================================

        if (slave.runtimeWatchdog.present &&
            slave.runtimeWatchdog.hasProcessDataTimeoutMs &&
            slave.runtimeWatchdog.processDataTimeoutMs >
            0U)
        {
            watchdogCount++;
        }
        else
        {
            pass =
                false;
        }


        // ====================================================================
        // DC Contract
        // ====================================================================

        if (slave.runtimeDc.present)
        {
            dcCount++;
        }
        else
        {
            pass =
                false;
        }


        const bool dcMode =
            StringEquals(
                slave.runtimeDc.mode,
                "DC");


        const bool freeRunMode =
            StringEquals(
                slave.runtimeDc.mode,
                "Free Run");


        if (!dcMode &&
            !freeRunMode)
        {
            pass =
                false;
        }


        if (dcMode)
        {
            if (!slave.runtimeDc.hasCycleTimeNs ||
                slave.runtimeDc.cycleTimeNs ==
                0U ||
                !slave.runtimeDc.hasShiftTimeNs ||
                slave.runtimeDc.shiftTimeNs <
                0 ||
                (
                    static_cast<uint64_t>(
                        slave.runtimeDc.shiftTimeNs) >=
                    static_cast<uint64_t>(
                        slave.runtimeDc.cycleTimeNs)
                    ) ||
                !slave.runtimeDc.hasReferenceClock)
            {
                pass =
                    false;
            }


            if (slave.runtimeDc.referenceClock)
            {
                dcReferenceCount++;
            }
        }


        // ====================================================================
        // SyncManager Contract
        // ====================================================================

        bool usedSmIndex[16] =
        {
            false
        };


        for (const auto& sm :
            slave.runtimeSyncManagers)
        {
            smCount++;


            if (sm.index <
                0 ||
                sm.index >
                15 ||
                usedSmIndex[sm.index])
            {
                pass =
                    false;


                continue;
            }


            usedSmIndex[sm.index] =
                true;


            if (sm.enabled)
            {
                // ------------------------------------------------------------
                // Stage 9A Fix2
                //
                // Min/Max are executable-size constraints.
                // They only apply when the SM is enabled.
                //
                // A disabled SM may intentionally be emitted as Length=0
                // while preserving the original ESI MinSize/MaxSize metadata.
                //
                // Current proven example:
                // R1-EC8124D0 SM2
                //     Enable = 0
                //     Length = 0
                //     ESI MinSize = 1
                //
                // Stage 5D writes it this way and Stage 8B hardware readback
                // already proves that state is correct.
                // ------------------------------------------------------------

                if (sm.length ==
                    0U)
                {
                    pass =
                        false;
                }


                if (sm.minimumSize >
                    0U &&
                    sm.length <
                    sm.minimumSize)
                {
                    pass =
                        false;
                }


                if (sm.maximumSize >
                    0U &&
                    sm.length >
                    sm.maximumSize)
                {
                    pass =
                        false;
                }
            }
            else
            {
                RtPrintf(
                    "[RUNTIME-CONTRACT-SM] "
                    "S%d SM%d | Enable:0 Len:%u Min:%u Max:%u | "
                    "Policy:DISABLED_SKIP_SIZE_RANGE | Result:PASS\n",

                    slaveIdx,
                    sm.index,

                    (unsigned int)
                    sm.length,

                    (unsigned int)
                    sm.minimumSize,

                    (unsigned int)
                    sm.maximumSize);
            }
        }


        const bool hasTransport =
            outputBytes >
            0U ||
            inputBytes >
            0U ||
            slave.runtimeMailbox.present;


        if (hasTransport &&
            slave.runtimeSyncManagers.empty())
        {
            pass =
                false;
        }


        // Stage 11G.2:
        // ProcessData may span multiple non-contiguous physical SM regions.
        // Validate PDO -> SM ownership and aggregate coverage instead of
        // assuming one contiguous physical block per direction.
        if (outputBytes >
            0U)
        {
            if (!ValidateProcessDataPhysicalSegments(
                slave,
                slave.runtimeRxPdos,
                slave.outputBitLength,
                slave.configAddrOut))
            {
                pass =
                    false;
            }
        }


        if (inputBytes >
            0U)
        {
            if (!ValidateProcessDataPhysicalSegments(
                slave,
                slave.runtimeTxPdos,
                slave.inputBitLength,
                slave.configAddrIn))
            {
                pass =
                    false;
            }
        }


        // ====================================================================
        // Mailbox <-> SM Cross Contract
        // ====================================================================

        if (slave.runtimeMailbox.present)
        {
            mailboxSlaveCount++;


            const auto& out =
                slave.runtimeMailbox.out;


            const auto& in =
                slave.runtimeMailbox.in;


            if (!out.present ||
                !in.present)
            {
                pass =
                    false;
            }


            if (out.present)
            {
                mailboxEndpointCount++;


                const auto* sm =
                    FindSm(
                        slave,
                        out.smIndex);


                if (sm ==
                    nullptr ||
                    sm->startAddress !=
                    out.startAddress ||
                    sm->length !=
                    out.length ||
                    sm->controlByte !=
                    out.controlByte ||
                    sm->enabled !=
                    out.enabled)
                {
                    pass =
                        false;
                }
            }


            if (in.present)
            {
                mailboxEndpointCount++;


                const auto* sm =
                    FindSm(
                        slave,
                        in.smIndex);


                if (sm ==
                    nullptr ||
                    sm->startAddress !=
                    in.startAddress ||
                    sm->length !=
                    in.length ||
                    sm->controlByte !=
                    in.controlByte ||
                    sm->enabled !=
                    in.enabled)
                {
                    pass =
                        false;
                }
            }
        }


        // ====================================================================
        // PDO Contract
        // ====================================================================

        ValidatePdoCollection(
            slave,
            slave.runtimeRxPdos,
            configurableMapping,
            pdoObjectCount,
            pdoEntryCount,
            errors);


        ValidatePdoCollection(
            slave,
            slave.runtimeTxPdos,
            configurableMapping,
            pdoObjectCount,
            pdoEntryCount,
            errors);


        const uint32_t rxBits =
            SumPdoBits(
                slave.runtimeRxPdos);


        const uint32_t txBits =
            SumPdoBits(
                slave.runtimeTxPdos);


        if (outputBytes >
            0U)
        {
            if (rxBits !=
                slave.outputBitLength)
            {
                pass =
                    false;
            }
        }
        else if (rxBits !=
            0U)
        {
            pass =
                false;
        }


        if (inputBytes >
            0U)
        {
            if (txBits !=
                slave.inputBitLength)
            {
                pass =
                    false;
            }
        }
        else if (txBits !=
            0U)
        {
            pass =
                false;
        }


        if (configurableMapping)
        {
            if (
                (
                    outputBytes >
                    0U &&
                    (
                        !slave.runtimeProfile.hasRxPdoAssignIndex ||
                        slave.runtimeProfile.rxPdoAssignIndex ==
                        0U ||
                        slave.runtimeRxPdos.empty()
                        )
                    ) ||
                (
                    inputBytes >
                    0U &&
                    (
                        !slave.runtimeProfile.hasTxPdoAssignIndex ||
                        slave.runtimeProfile.txPdoAssignIndex ==
                        0U ||
                        slave.runtimeTxPdos.empty()
                        )
                    ) ||
                !slave.runtimeMailbox.present
                )
            {
                pass =
                    false;
            }
        }


        // ====================================================================
        // FMMU <-> Binding <-> ProcessData Cross Contract
        //
        // Stage 11G.2:
        // One logical direction may be backed by 1..N physical SyncManagers
        // and therefore 1..N FMMUs. Logical Process Image coverage remains
        // compact and contiguous; physical ESC addresses may be segmented.
        // ====================================================================

        if (outputBytes >
            0U ||
            inputBytes >
            0U)
        {
            if (!slave.runtimeFmmuSchemaPresent)
            {
                pass =
                    false;
            }
        }


        std::vector<RuntimeFmmuExpectedSegment> expectedOutputSegments =
            BuildExpectedFmmuSegments(
                slave,
                slave.runtimeRxPdos);


        std::vector<RuntimeFmmuExpectedSegment> expectedInputSegments =
            BuildExpectedFmmuSegments(
                slave,
                slave.runtimeTxPdos);


        // Compatibility fallback for custom/legacy-style Runtime profiles
        // that have ProcessData but no PDO -> SyncManager ownership.
        if (outputBytes >
            0U &&
            expectedOutputSegments.empty())
        {
            RuntimeFmmuExpectedSegment segment;

            segment.smIndex =
                -1;

            segment.physicalStartAddress =
                slave.configAddrOut;

            segment.bitSize =
                slave.outputBitLength;

            expectedOutputSegments.push_back(
                segment);
        }


        if (inputBytes >
            0U &&
            expectedInputSegments.empty())
        {
            RuntimeFmmuExpectedSegment segment;

            segment.smIndex =
                -1;

            segment.physicalStartAddress =
                slave.configAddrIn;

            segment.bitSize =
                slave.inputBitLength;

            expectedInputSegments.push_back(
                segment);
        }


        int outputFmmuCount =
            0;


        int inputFmmuCount =
            0;


        uint64_t outputLogicalBitCursor =
            outputBytes >
            0U
            ? (uint64_t)
            binding.outputOffset *
            8U
            : 0U;


        uint64_t inputLogicalBitCursor =
            inputBytes >
            0U
            ? (uint64_t)
            binding.inputOffset *
            8U
            : 0U;


        bool usedFmmuIndex[16] =
        {
            false
        };


        for (const auto& fmmu :
            slave.runtimeFmmus)
        {
            fmmuCount++;


            bool fmmuPass =
                fmmu.index >=
                0 &&
                fmmu.index <=
                15 &&
                !usedFmmuIndex[fmmu.index] &&
                fmmu.enabled &&
                fmmu.logicalLength >
                0U &&
                fmmu.logicalStartBit <=
                7U &&
                fmmu.logicalEndBit <=
                7U &&
                fmmu.physicalStartBit <=
                7U;


            if (fmmu.index >=
                0 &&
                fmmu.index <=
                15)
            {
                usedFmmuIndex[fmmu.index] =
                    true;
            }


            if (StringEquals(
                fmmu.direction,
                "Output"))
            {
                if (outputFmmuCount >=
                    (int)
                    expectedOutputSegments.size())
                {
                    fmmuPass =
                        false;
                }
                else
                {
                    const RuntimeFmmuExpectedSegment& expected =
                        expectedOutputSegments[
                            (size_t)
                                outputFmmuCount];


                    if (!RuntimeFmmuMatchesExpectedSegment(
                        fmmu,
                        outputLogicalBitCursor,
                        expected.bitSize,
                        expected.physicalStartAddress,
                        2U))
                    {
                        fmmuPass =
                            false;
                    }


                    outputLogicalBitCursor +=
                        expected.bitSize;
                }


                outputFmmuCount++;
            }
            else if (StringEquals(
                fmmu.direction,
                "Input"))
            {
                if (inputFmmuCount >=
                    (int)
                    expectedInputSegments.size())
                {
                    fmmuPass =
                        false;
                }
                else
                {
                    const RuntimeFmmuExpectedSegment& expected =
                        expectedInputSegments[
                            (size_t)
                                inputFmmuCount];


                    if (!RuntimeFmmuMatchesExpectedSegment(
                        fmmu,
                        inputLogicalBitCursor,
                        expected.bitSize,
                        expected.physicalStartAddress,
                        1U))
                    {
                        fmmuPass =
                            false;
                    }


                    inputLogicalBitCursor +=
                        expected.bitSize;
                }


                inputFmmuCount++;
            }
            else
            {
                fmmuPass =
                    false;
            }


            const uint64_t logicalEnd =
                (uint64_t)
                fmmu.logicalStartAddress +
                (uint64_t)
                fmmu.logicalLength;


            if (logicalEnd >
                (uint64_t)m_IoMapSize)
            {
                fmmuPass =
                    false;
            }


            if (!fmmuPass)
            {
                pass =
                    false;
            }
        }


        if (outputFmmuCount !=
            (int)
            expectedOutputSegments.size() ||
            inputFmmuCount !=
            (int)
            expectedInputSegments.size())
        {
            pass =
                false;
        }


        const uint64_t expectedOutputLogicalEndBits =
            outputBytes >
            0U
            ? (
                (uint64_t)
                binding.outputOffset *
                8U
                ) +
            slave.outputBitLength
            : 0U;


        const uint64_t expectedInputLogicalEndBits =
            inputBytes >
            0U
            ? (
                (uint64_t)
                binding.inputOffset *
                8U
                ) +
            slave.inputBitLength
            : 0U;


        if (outputBytes >
            0U &&
            outputLogicalBitCursor !=
            expectedOutputLogicalEndBits)
        {
            pass =
                false;
        }


        if (inputBytes >
            0U &&
            inputLogicalBitCursor !=
            expectedInputLogicalEndBits)
        {
            pass =
                false;
        }


        // ====================================================================
        // InitCommand Contract
        // ====================================================================

        bool hasAppliedInitCommand =
            false;


        int initCommandOrdinal =
            0;


        for (const auto& command :
            slave.runtimeInitCommands)
        {
            initCommandCount++;
            initCommandOrdinal++;


            // ------------------------------------------------------------
            // Stage 9A Fix2 - InitCommand contract policy
            //
            // hasApply must always be explicit.
            //
            // Apply=0:
            //     Engineering metadata only.
            //     It must NOT be forced to satisfy executable Runtime
            //     transition/index/data requirements.
            //
            // Apply=1:
            //     Executable Runtime command.
            //     Transition / Index / Data are mandatory and validated.
            // ------------------------------------------------------------

            bool commandPass =
                command.hasApply;


            if (command.apply)
            {
                appliedInitCommandCount++;

                hasAppliedInitCommand =
                    true;


                commandPass =
                    commandPass &&
                    IsKnownTransition(
                        command.transition) &&
                    command.index !=
                    0U &&
                    !command.data.empty();


                RtPrintf(
                    "[RUNTIME-CONTRACT-INITCMD] "
                    "S%d #%d | Source:%s Apply:1 | "
                    "Transition:%s Index:0x%04X Bytes:%u | "
                    "Policy:EXECUTABLE | Result:%s\n",

                    slaveIdx,
                    initCommandOrdinal,

                    command.source[0] != '\0'
                    ? command.source
                    : "N/A",

                    command.transition[0] != '\0'
                    ? command.transition
                    : "N/A",

                    (unsigned int)
                    command.index,

                    (unsigned int)
                    command.data.size(),

                    commandPass
                    ? "PASS"
                    : "FAIL");
            }
            else
            {
                RtPrintf(
                    "[RUNTIME-CONTRACT-INITCMD] "
                    "S%d #%d | Source:%s Apply:0 | "
                    "Policy:METADATA_ONLY | Result:%s\n",

                    slaveIdx,
                    initCommandOrdinal,

                    command.source[0] != '\0'
                    ? command.source
                    : "N/A",

                    commandPass
                    ? "PASS"
                    : "FAIL");
            }


            if (!commandPass)
            {
                pass =
                    false;
            }
        }


        if (hasAppliedInitCommand &&
            !slave.runtimeMailbox.present)
        {
            pass =
                false;
        }


        if (!pass)
        {
            errors++;
        }


        RtPrintf(
            "[RUNTIME-CONTRACT-SLAVE] "
            "S%d | "
            "Profile:%s Binding:%s WD:%s DC:%s | "
            "SM:%u RxPDO:%u TxPDO:%u FMMU:%u | "
            "Mailbox:%s InitCmd:%u | "
            "Out:%u In:%u | Result:%s\n",

            slaveIdx,

            slave.runtimeProfile.present
            ? "YES"
            : "NO",

            binding.present
            ? "YES"
            : "NO",

            slave.runtimeWatchdog.present
            ? "YES"
            : "NO",

            slave.runtimeDc.present
            ? "YES"
            : "NO",

            (unsigned int)
            slave.runtimeSyncManagers.size(),

            (unsigned int)
            slave.runtimeRxPdos.size(),

            (unsigned int)
            slave.runtimeTxPdos.size(),

            (unsigned int)
            slave.runtimeFmmus.size(),

            slave.runtimeMailbox.present
            ? "YES"
            : "NO",

            (unsigned int)
            slave.runtimeInitCommands.size(),

            (unsigned int)
            outputBytes,

            (unsigned int)
            inputBytes,

            pass
            ? "PASS"
            : "FAIL");
    }


    if (maxBindingEnd !=
        (uint32_t)m_IoMapSize)
    {
        errors++;


        RtPrintf(
            "[RUNTIME-CONTRACT-GLOBAL] "
            "ProcessImage RuntimeEnd:%u ActualMap:%d | Result:FAIL\n",

            (unsigned int)
            maxBindingEnd,

            m_IoMapSize);
    }


    bool anyDc =
        false;


    for (const auto& slave :
        slaves)
    {
        if (StringEquals(
            slave.runtimeDc.mode,
            "DC"))
        {
            anyDc =
                true;

            break;
        }
    }


    if (anyDc &&
        dcReferenceCount !=
        1)
    {
        errors++;


        RtPrintf(
            "[RUNTIME-CONTRACT-GLOBAL] "
            "DC Reference count:%d | Expected:1 | Result:FAIL\n",

            dcReferenceCount);
    }


    const bool pass =
        errors ==
        0;


    RtPrintf(
        "[RUNTIME-CONTRACT-AUDIT-RESULT] "
        "Slaves:%u | "
        "Profiles:%d | Bindings:%d | Watchdogs:%d | DC:%d | "
        "SM:%d | PDO:%d Entries:%d | "
        "FMMU:%d | MailboxSlaves:%d Endpoints:%d | "
        "InitCmd:%d Applied:%d | "
        "DCReferences:%d | "
        "ProcessImage:%u/%d B | "
        "Errors:%d | "
        "Result:%s | "
        "ProductCodeBaseline:NO | "
        "HardwareRead:NO | HardwareWrite:NO | "
        "GateAction:RETURN_TO_STAGE9B_ROUTER\n",

        (unsigned int)
        slaves.size(),

        profileCount,
        bindingCount,
        watchdogCount,
        dcCount,

        smCount,
        pdoObjectCount,
        pdoEntryCount,

        fmmuCount,

        mailboxSlaveCount,
        mailboxEndpointCount,

        initCommandCount,
        appliedInitCommandCount,

        dcReferenceCount,

        (unsigned int)
        maxBindingEnd,

        m_IoMapSize,

        errors,

        pass
        ? "PASS"
        : "FAIL");


    RtPrintf(
        "============================================================\n"
        "[RUNTIME-CONTRACT] END | "
        "Stage:9B | Result:%s | ShadowOnly:NO\n"
        "============================================================\n\n",

        pass
        ? "PASS"
        : "FAIL");


    return
        pass;
}
