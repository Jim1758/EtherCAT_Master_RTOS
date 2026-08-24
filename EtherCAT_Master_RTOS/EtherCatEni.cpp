#include "EtherCatEni.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <windows.h>
#include <rtapi.h>

#include "GlobalConfig.h"


// ============================================================================
// Local Helpers
//
// 注意：
// 這些函式只在 EtherCAT 初始化 / 讀取 Runtime XML 時使用。
// 不在 250 us PDO 即時迴圈中執行。
// ============================================================================

namespace
{
    const long MAX_RUNTIME_XML_FILE_SIZE =
        16L * 1024L * 1024L;


    // ------------------------------------------------------------------------
    // 判斷 "<Tag" 後面的字元是否真的是 XML Tag 邊界。
    //
    // 允許：
    //
    // <Slave>
    // <Slave Position="0">
    //
    // 排除：
    //
    // <Slaves>
    // <SlaveSomething>
    // ------------------------------------------------------------------------
    bool IsXmlTagBoundary(
        char c)
    {
        return
            c == '>' ||
            c == '/' ||
            c == '\0' ||
            isspace(
                static_cast<unsigned char>(
                    c)) != 0;
    }


    // ------------------------------------------------------------------------
    // 尋找 XML Element Opening Tag。
    //
    // 支援：
    //
    // <Slave>
    // <Slave Position="0">
    //
    // 不要求 Opening Tag 必須完全等於 "<Slave>"。
    // ------------------------------------------------------------------------
    char* FindXmlElementStart(
        char* searchStart,
        const char* tag)
    {
        if (searchStart == nullptr ||
            tag == nullptr ||
            tag[0] == '\0')
        {
            return nullptr;
        }


        char prefix[96] = { 0 };

        sprintf(
            prefix,
            "<%s",
            tag);


        char* p =
            searchStart;


        while ((p =
            strstr(
                p,
                prefix)) != nullptr)
        {
            const size_t prefixLength =
                strlen(
                    prefix);

            const char next =
                p[prefixLength];


            if (IsXmlTagBoundary(
                next))
            {
                return
                    p;
            }


            p +=
                prefixLength;
        }


        return nullptr;
    }


    // ------------------------------------------------------------------------
    // Read UInt32
    //
    // 支援：
    //
    // 477
    // 0x000001DD
    // #x000001DD
    // #X000001DD
    //
    // Runtime Config 現在主要輸出十進位，
    // 但保留 Hex 相容性方便未來工具與手動診斷。
    // ------------------------------------------------------------------------
    uint32_t ParseUInt32Flexible(
        const char* text)
    {
        if (text == nullptr)
        {
            return 0;
        }


        while (*text != '\0' &&
            isspace(
                static_cast<unsigned char>(
                    *text)) != 0)
        {
            ++text;
        }


        if (text[0] == '#' &&
            (text[1] == 'x' ||
                text[1] == 'X'))
        {
            return
                static_cast<uint32_t>(
                    strtoul(
                        text + 2,
                        nullptr,
                        16));
        }


        if (text[0] == '0' &&
            (text[1] == 'x' ||
                text[1] == 'X'))
        {
            return
                static_cast<uint32_t>(
                    strtoul(
                        text + 2,
                        nullptr,
                        16));
        }


        return
            static_cast<uint32_t>(
                strtoul(
                    text,
                    nullptr,
                    10));
    }


    // ------------------------------------------------------------------------
    // 將 XML Element 的內容複製到固定大小輸出 Buffer。
    //
    // 支援 Opening Tag Attribute：
    //
    // <Name>...</Name>
    // <Name LcId="1033">...</Name>
    // ------------------------------------------------------------------------
    bool CopyXmlElementValue(
        const char* content,
        const char* tag,
        char* outBuffer,
        size_t outCapacity)
    {
        if (content == nullptr ||
            tag == nullptr ||
            outBuffer == nullptr ||
            outCapacity == 0)
        {
            return false;
        }


        outBuffer[0] =
            '\0';


        char* mutableContent =
            const_cast<char*>(
                content);

        char* pStart =
            FindXmlElementStart(
                mutableContent,
                tag);


        if (pStart == nullptr)
        {
            return false;
        }


        char* pOpenEnd =
            strchr(
                pStart,
                '>');


        if (pOpenEnd == nullptr)
        {
            return false;
        }


        ++pOpenEnd;


        char endTag[96] = { 0 };

        sprintf(
            endTag,
            "</%s>",
            tag);


        char* pEnd =
            strstr(
                pOpenEnd,
                endTag);


        if (pEnd == nullptr)
        {
            return false;
        }


        size_t length =
            static_cast<size_t>(
                pEnd -
                pOpenEnd);


        if (length >=
            outCapacity)
        {
            length =
                outCapacity -
                1;
        }


        memcpy(
            outBuffer,
            pOpenEnd,
            length);

        outBuffer[length] =
            '\0';


        return true;
    }


    // ------------------------------------------------------------------------
    // Copy one complete XML element block.
    //
    // Caller must free() the returned buffer.
    // Startup parser only; never used in the 250 us PDO loop.
    // ------------------------------------------------------------------------
    char* CopyXmlElementBlockAllocated(
        const char* content,
        const char* tag)
    {
        if (content == nullptr ||
            tag == nullptr ||
            tag[0] == '\0')
        {
            return nullptr;
        }


        char* pStart =
            FindXmlElementStart(
                const_cast<char*>(
                    content),
                tag);


        if (pStart == nullptr)
        {
            return nullptr;
        }


        char endTag[96] =
        {
            0
        };


        sprintf(
            endTag,
            "</%s>",
            tag);


        char* pEnd =
            strstr(
                pStart,
                endTag);


        if (pEnd == nullptr)
        {
            return nullptr;
        }


        pEnd +=
            strlen(
                endTag);


        const size_t blockLength =
            static_cast<size_t>(
                pEnd -
                pStart);


        char* block =
            static_cast<char*>(
                malloc(
                    blockLength +
                    1));


        if (block == nullptr)
        {
            return nullptr;
        }


        memcpy(
            block,
            pStart,
            blockLength);


        block[blockLength] =
            '\0';


        return
            block;
    }


    // ------------------------------------------------------------------------
    // Signed parser for values such as ShiftTimeNs.
    //
    // Supports decimal, 0x..., #x..., and negative decimal/hex.
    // ------------------------------------------------------------------------
    int64_t ParseInt64Flexible(
        const char* text)
    {
        if (text == nullptr)
        {
            return 0;
        }


        while (*text != '\0' &&
            isspace(
                static_cast<unsigned char>(
                    *text)) != 0)
        {
            ++text;
        }


        bool negative =
            false;


        if (*text == '-')
        {
            negative =
                true;

            ++text;
        }


        uint64_t value =
            0;


        if (text[0] == '#' &&
            (text[1] == 'x' ||
                text[1] == 'X'))
        {
            value =
                static_cast<uint64_t>(
                    strtoull(
                        text + 2,
                        nullptr,
                        16));
        }
        else if (text[0] == '0' &&
            (text[1] == 'x' ||
                text[1] == 'X'))
        {
            value =
                static_cast<uint64_t>(
                    strtoull(
                        text + 2,
                        nullptr,
                        16));
        }
        else
        {
            value =
                static_cast<uint64_t>(
                    strtoull(
                        text,
                        nullptr,
                        10));
        }


        if (negative)
        {
            return
                -static_cast<int64_t>(
                    value);
        }


        return
            static_cast<int64_t>(
                value);
    }


    // ------------------------------------------------------------------------
    // Parse comma-separated PDO indexes.
    //
    // Example:
    // 0x1600,0x1601,0x1602
    // ------------------------------------------------------------------------
    void ParseUInt16ListFlexible(
        const char* text,
        std::vector<uint16_t>& values)
    {
        values.clear();


        if (text == nullptr)
        {
            return;
        }


        const char* p =
            text;


        while (*p != '\0')
        {
            while (*p != '\0' &&
                (
                    *p == ',' ||
                    isspace(
                        static_cast<unsigned char>(
                            *p)) != 0
                    ))
            {
                ++p;
            }


            if (*p == '\0')
            {
                break;
            }


            char token[64] =
            {
                0
            };


            size_t tokenLength =
                0;


            while (*p != '\0' &&
                *p != ',' &&
                tokenLength <
                sizeof(token) -
                1)
            {
                token[tokenLength++] =
                    *p;

                ++p;
            }


            while (tokenLength > 0 &&
                isspace(
                    static_cast<unsigned char>(
                        token[tokenLength - 1])) != 0)
            {
                tokenLength--;
            }


            token[tokenLength] =
                '\0';


            if (tokenLength > 0)
            {
                const uint32_t parsed =
                    ParseUInt32Flexible(
                        token);


                if (parsed <=
                    0xFFFFU)
                {
                    values.push_back(
                        static_cast<uint16_t>(
                            parsed));
                }
            }
        }
    }


    // ------------------------------------------------------------------------
    // Advance to the byte immediately after </tag>.
    // ------------------------------------------------------------------------
    const char* MoveAfterXmlElement(
        const char* elementStart,
        const char* tag)
    {
        if (elementStart == nullptr ||
            tag == nullptr)
        {
            return nullptr;
        }


        char endTag[96] =
        {
            0
        };


        sprintf(
            endTag,
            "</%s>",
            tag);


        const char* pEnd =
            strstr(
                elementStart,
                endTag);


        if (pEnd == nullptr)
        {
            return nullptr;
        }


        return
            pEnd +
            strlen(
                endTag);
    }


    // ------------------------------------------------------------------------
    // Convert a compact hex string into raw bytes.
    //
    // Example:
    // "08"     -> { 0x08 }
    // "0100"   -> { 0x01, 0x00 }
    // ------------------------------------------------------------------------
    void ParseHexBytes(
        const char* text,
        std::vector<uint8_t>& bytes)
    {
        bytes.clear();


        if (text == nullptr)
        {
            return;
        }


        char compact[4096] =
        {
            0
        };


        size_t length =
            0;


        for (const char* p = text;
            *p != '\0' &&
            length <
            sizeof(compact) -
            1;
            ++p)
        {
            if (isxdigit(
                static_cast<unsigned char>(
                    *p)) != 0)
            {
                compact[length++] =
                    *p;
            }
        }


        compact[length] =
            '\0';


        if (length == 0 ||
            (length % 2) != 0)
        {
            return;
        }


        for (size_t i = 0;
            i < length;
            i += 2)
        {
            char byteText[3] =
            {
                compact[i],
                compact[i + 1],
                '\0'
            };


            bytes.push_back(
                static_cast<uint8_t>(
                    strtoul(
                        byteText,
                        nullptr,
                        16)));
        }
    }


    void ParseRuntimeSyncManagers(
        const char* slaveContent,
        EtherCatSlave& slave)
    {
        slave.runtimeSyncManagers.clear();


        char* section =
            CopyXmlElementBlockAllocated(
                slaveContent,
                "SyncManagers");


        if (section == nullptr)
        {
            return;
        }


        const char* cursor =
            section;


        while (true)
        {
            char* smStart =
                FindXmlElementStart(
                    const_cast<char*>(
                        cursor),
                    "Sm");


            if (smStart == nullptr)
            {
                break;
            }


            char* smBlock =
                CopyXmlElementBlockAllocated(
                    smStart,
                    "Sm");


            if (smBlock == nullptr)
            {
                break;
            }


            EtherCatRuntimeSyncManagerConfig sm;


            char buffer[256] =
            {
                0
            };


            if (CopyXmlElementValue(
                smBlock,
                "Index",
                buffer,
                sizeof(buffer)))
            {
                sm.index =
                    static_cast<int>(
                        ParseInt64Flexible(
                            buffer));
            }


            CopyXmlElementValue(
                smBlock,
                "Name",
                sm.name,
                sizeof(sm.name));


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                smBlock,
                "StartAddress",
                buffer,
                sizeof(buffer)))
            {
                sm.startAddress =
                    static_cast<uint16_t>(
                        ParseUInt32Flexible(
                            buffer));
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                smBlock,
                "Length",
                buffer,
                sizeof(buffer)))
            {
                sm.length =
                    static_cast<uint16_t>(
                        ParseUInt32Flexible(
                            buffer));
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                smBlock,
                "MinimumSize",
                buffer,
                sizeof(buffer)))
            {
                sm.minimumSize =
                    static_cast<uint16_t>(
                        ParseUInt32Flexible(
                            buffer));
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                smBlock,
                "MaximumSize",
                buffer,
                sizeof(buffer)))
            {
                sm.maximumSize =
                    static_cast<uint16_t>(
                        ParseUInt32Flexible(
                            buffer));
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                smBlock,
                "ControlByte",
                buffer,
                sizeof(buffer)))
            {
                sm.controlByte =
                    static_cast<uint8_t>(
                        ParseUInt32Flexible(
                            buffer));
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                smBlock,
                "Enable",
                buffer,
                sizeof(buffer)))
            {
                sm.enabled =
                    ParseUInt32Flexible(
                        buffer) != 0;
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                smBlock,
                "OpOnly",
                buffer,
                sizeof(buffer)))
            {
                sm.opOnly =
                    ParseUInt32Flexible(
                        buffer) != 0;
            }


            slave.runtimeSyncManagers.push_back(
                sm);


            const char* next =
                MoveAfterXmlElement(
                    smStart,
                    "Sm");


            free(
                smBlock);


            if (next == nullptr)
            {
                break;
            }


            cursor =
                next;
        }


        free(
            section);
    }


    void ParseRuntimeProcessImageBinding(
        const char* slaveContent,
        EtherCatSlave& slave)
    {
        slave.runtimeProcessImageBinding =
            EtherCatRuntimeProcessImageBindingConfig();


        char* section =
            CopyXmlElementBlockAllocated(
                slaveContent,
                "ProcessImageBinding");


        if (section == nullptr)
        {
            return;
        }


        slave.runtimeProcessImageBinding.present =
            true;


        char buffer[256] =
        {
            0
        };


        CopyXmlElementValue(
            section,
            "Kind",
            slave.runtimeProcessImageBinding.kind,
            sizeof(
                slave.runtimeProcessImageBinding.kind));


        CopyXmlElementValue(
            section,
            "Abi",
            slave.runtimeProcessImageBinding.abi,
            sizeof(
                slave.runtimeProcessImageBinding.abi));


        if (CopyXmlElementValue(
            section,
            "OutputOffset",
            buffer,
            sizeof(buffer)))
        {
            slave.runtimeProcessImageBinding.outputOffset =
                static_cast<int32_t>(
                    ParseInt64Flexible(
                        buffer));
        }


        memset(
            buffer,
            0,
            sizeof(buffer));


        if (CopyXmlElementValue(
            section,
            "OutputBytes",
            buffer,
            sizeof(buffer)))
        {
            slave.runtimeProcessImageBinding.outputBytes =
                ParseUInt32Flexible(
                    buffer);
        }


        memset(
            buffer,
            0,
            sizeof(buffer));


        if (CopyXmlElementValue(
            section,
            "InputOffset",
            buffer,
            sizeof(buffer)))
        {
            slave.runtimeProcessImageBinding.inputOffset =
                static_cast<int32_t>(
                    ParseInt64Flexible(
                        buffer));
        }


        memset(
            buffer,
            0,
            sizeof(buffer));


        if (CopyXmlElementValue(
            section,
            "InputBytes",
            buffer,
            sizeof(buffer)))
        {
            slave.runtimeProcessImageBinding.inputBytes =
                ParseUInt32Flexible(
                    buffer);
        }


        memset(
            buffer,
            0,
            sizeof(buffer));


        if (CopyXmlElementValue(
            section,
            "ElementBytes",
            buffer,
            sizeof(buffer)))
        {
            slave.runtimeProcessImageBinding.elementBytes =
                static_cast<uint16_t>(
                    ParseUInt32Flexible(
                        buffer));
        }


        memset(
            buffer,
            0,
            sizeof(buffer));


        if (CopyXmlElementValue(
            section,
            "ChannelCount",
            buffer,
            sizeof(buffer)))
        {
            slave.runtimeProcessImageBinding.channelCount =
                static_cast<uint16_t>(
                    ParseUInt32Flexible(
                        buffer));
        }


        free(
            section);
    }


    // ========================================================================
    // Stage 11A - Composite Application Bindings
    //
    // Parse only. No application list mutation and no EtherCAT hardware write.
    // ========================================================================

    void ParseRuntimeApplicationBindings(
        const char* slaveContent,
        EtherCatSlave& slave)
    {
        slave.runtimeApplicationBindingsSchemaPresent =
            false;


        memset(
            slave.runtimeApplicationBindingsMode,
            0,
            sizeof(
                slave.runtimeApplicationBindingsMode));


        slave.runtimeApplicationBindings.clear();


        char* section =
            CopyXmlElementBlockAllocated(
                slaveContent,
                "ApplicationBindings");


        if (section ==
            nullptr)
        {
            return;
        }


        slave.runtimeApplicationBindingsSchemaPresent =
            true;


        CopyXmlElementValue(
            section,
            "Mode",
            slave.runtimeApplicationBindingsMode,
            sizeof(
                slave.runtimeApplicationBindingsMode));


        const char* cursor =
            section;


        while (true)
        {
            char* bindingStart =
                FindXmlElementStart(
                    const_cast<char*>(
                        cursor),
                    "ApplicationBinding");


            if (bindingStart ==
                nullptr)
            {
                break;
            }


            char* bindingBlock =
                CopyXmlElementBlockAllocated(
                    bindingStart,
                    "ApplicationBinding");


            if (bindingBlock ==
                nullptr)
            {
                break;
            }


            EtherCatRuntimeApplicationBindingConfig
                binding;


            char buffer[256] =
            {
                0
            };


            CopyXmlElementValue(
                bindingBlock,
                "Id",
                binding.id,
                sizeof(
                    binding.id));


            CopyXmlElementValue(
                bindingBlock,
                "Kind",
                binding.kind,
                sizeof(
                    binding.kind));


            CopyXmlElementValue(
                bindingBlock,
                "LegacyKind",
                binding.legacyKind,
                sizeof(
                    binding.legacyKind));


            CopyXmlElementValue(
                bindingBlock,
                "Abi",
                binding.abi,
                sizeof(
                    binding.abi));


            CopyXmlElementValue(
                bindingBlock,
                "Interface",
                binding.interfaceType,
                sizeof(
                    binding.interfaceType));


            CopyXmlElementValue(
                bindingBlock,
                "DataType",
                binding.dataType,
                sizeof(
                    binding.dataType));


            CopyXmlElementValue(
                bindingBlock,
                "SampleMode",
                binding.sampleMode,
                sizeof(
                    binding.sampleMode));


            CopyXmlElementValue(
                bindingBlock,
                "Unit",
                binding.unit,
                sizeof(
                    binding.unit));


            CopyXmlElementValue(
                bindingBlock,
                "AxisRef",
                binding.axisRef,
                sizeof(
                    binding.axisRef));


            if (CopyXmlElementValue(
                bindingBlock,
                "OutputBitOffset",
                buffer,
                sizeof(buffer)))
            {
                binding.outputBitOffset =
                    static_cast<int32_t>(
                        ParseInt64Flexible(
                            buffer));
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                bindingBlock,
                "OutputBitLength",
                buffer,
                sizeof(buffer)))
            {
                binding.outputBitLength =
                    ParseUInt32Flexible(
                        buffer);
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                bindingBlock,
                "InputBitOffset",
                buffer,
                sizeof(buffer)))
            {
                binding.inputBitOffset =
                    static_cast<int32_t>(
                        ParseInt64Flexible(
                            buffer));
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                bindingBlock,
                "InputBitLength",
                buffer,
                sizeof(buffer)))
            {
                binding.inputBitLength =
                    ParseUInt32Flexible(
                        buffer);
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                bindingBlock,
                "ElementBits",
                buffer,
                sizeof(buffer)))
            {
                binding.elementBits =
                    static_cast<uint16_t>(
                        ParseUInt32Flexible(
                            buffer));
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                bindingBlock,
                "ChannelCount",
                buffer,
                sizeof(buffer)))
            {
                binding.channelCount =
                    static_cast<uint16_t>(
                        ParseUInt32Flexible(
                            buffer));
            }


            slave.runtimeApplicationBindings.push_back(
                binding);


            const char* next =
                MoveAfterXmlElement(
                    bindingStart,
                    "ApplicationBinding");


            free(
                bindingBlock);


            if (next ==
                nullptr)
            {
                break;
            }


            cursor =
                next;
        }


        free(
            section);
    }


    // ========================================================================
    // Stage 11C.1 - Persisted Composite Runtime Shadow Parser
    //
    // Parse only.
    // No BuildIoMap mutation.
    // No EtherCAT hardware access.
    // ========================================================================

    void ParseRuntimeCompositeBindingsShadow(
        const char* slaveContent,
        EtherCatSlave& slave)
    {
        slave.runtimeCompositeBindingsShadowSchemaPresent =
            false;


        memset(
            slave.runtimeCompositeBindingsShadowMode,
            0,
            sizeof(
                slave.runtimeCompositeBindingsShadowMode));


        slave.runtimeCompositeBindingsShadowOutputBaseBit =
            -1;


        slave.runtimeCompositeBindingsShadowInputBaseBit =
            -1;


        slave.runtimeCompositeBindingsShadow.clear();


        char* section =
            CopyXmlElementBlockAllocated(
                slaveContent,
                "CompositeApplicationBindingsShadow");


        if (section ==
            nullptr)
        {
            return;
        }


        slave.runtimeCompositeBindingsShadowSchemaPresent =
            true;


        CopyXmlElementValue(
            section,
            "Mode",
            slave.runtimeCompositeBindingsShadowMode,
            sizeof(
                slave.runtimeCompositeBindingsShadowMode));


        char buffer[256] =
        {
            0
        };


        if (CopyXmlElementValue(
            section,
            "OutputBaseBit",
            buffer,
            sizeof(buffer)))
        {
            slave.runtimeCompositeBindingsShadowOutputBaseBit =
                static_cast<int32_t>(
                    ParseInt64Flexible(
                        buffer));
        }


        memset(
            buffer,
            0,
            sizeof(buffer));


        if (CopyXmlElementValue(
            section,
            "InputBaseBit",
            buffer,
            sizeof(buffer)))
        {
            slave.runtimeCompositeBindingsShadowInputBaseBit =
                static_cast<int32_t>(
                    ParseInt64Flexible(
                        buffer));
        }


        const char* cursor =
            section;


        while (true)
        {
            char* bindingStart =
                FindXmlElementStart(
                    const_cast<char*>(
                        cursor),
                    "ApplicationBinding");


            if (bindingStart ==
                nullptr)
            {
                break;
            }


            char* bindingBlock =
                CopyXmlElementBlockAllocated(
                    bindingStart,
                    "ApplicationBinding");


            if (bindingBlock ==
                nullptr)
            {
                break;
            }


            EtherCatRuntimeApplicationBindingConfig
                binding;


            memset(
                buffer,
                0,
                sizeof(buffer));


            CopyXmlElementValue(
                bindingBlock,
                "Id",
                binding.id,
                sizeof(
                    binding.id));


            CopyXmlElementValue(
                bindingBlock,
                "Name",
                binding.name,
                sizeof(
                    binding.name));


            if (CopyXmlElementValue(
                bindingBlock,
                "SortOrder",
                buffer,
                sizeof(buffer)))
            {
                binding.sortOrder =
                    static_cast<int32_t>(
                        ParseInt64Flexible(
                            buffer));
            }


            CopyXmlElementValue(
                bindingBlock,
                "Kind",
                binding.kind,
                sizeof(
                    binding.kind));


            CopyXmlElementValue(
                bindingBlock,
                "LegacyKind",
                binding.legacyKind,
                sizeof(
                    binding.legacyKind));


            CopyXmlElementValue(
                bindingBlock,
                "Abi",
                binding.abi,
                sizeof(
                    binding.abi));


            CopyXmlElementValue(
                bindingBlock,
                "Interface",
                binding.interfaceType,
                sizeof(
                    binding.interfaceType));


            CopyXmlElementValue(
                bindingBlock,
                "DataType",
                binding.dataType,
                sizeof(
                    binding.dataType));


            CopyXmlElementValue(
                bindingBlock,
                "SampleMode",
                binding.sampleMode,
                sizeof(
                    binding.sampleMode));


            CopyXmlElementValue(
                bindingBlock,
                "Unit",
                binding.unit,
                sizeof(
                    binding.unit));


            CopyXmlElementValue(
                bindingBlock,
                "AxisRef",
                binding.axisRef,
                sizeof(
                    binding.axisRef));


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                bindingBlock,
                "OutputRelativeBitOffset",
                buffer,
                sizeof(buffer)))
            {
                binding.outputRelativeBitOffset =
                    static_cast<int32_t>(
                        ParseInt64Flexible(
                            buffer));
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                bindingBlock,
                "OutputBitOffset",
                buffer,
                sizeof(buffer)))
            {
                binding.outputBitOffset =
                    static_cast<int32_t>(
                        ParseInt64Flexible(
                            buffer));
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                bindingBlock,
                "OutputBitLength",
                buffer,
                sizeof(buffer)))
            {
                binding.outputBitLength =
                    ParseUInt32Flexible(
                        buffer);
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                bindingBlock,
                "InputRelativeBitOffset",
                buffer,
                sizeof(buffer)))
            {
                binding.inputRelativeBitOffset =
                    static_cast<int32_t>(
                        ParseInt64Flexible(
                            buffer));
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                bindingBlock,
                "InputBitOffset",
                buffer,
                sizeof(buffer)))
            {
                binding.inputBitOffset =
                    static_cast<int32_t>(
                        ParseInt64Flexible(
                            buffer));
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                bindingBlock,
                "InputBitLength",
                buffer,
                sizeof(buffer)))
            {
                binding.inputBitLength =
                    ParseUInt32Flexible(
                        buffer);
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                bindingBlock,
                "ElementBits",
                buffer,
                sizeof(buffer)))
            {
                binding.elementBits =
                    static_cast<uint16_t>(
                        ParseUInt32Flexible(
                            buffer));
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                bindingBlock,
                "ChannelCount",
                buffer,
                sizeof(buffer)))
            {
                binding.channelCount =
                    static_cast<uint16_t>(
                        ParseUInt32Flexible(
                            buffer));
            }


            slave.runtimeCompositeBindingsShadow.push_back(
                binding);


            const char* next =
                MoveAfterXmlElement(
                    bindingStart,
                    "ApplicationBinding");


            free(
                bindingBlock);


            if (next ==
                nullptr)
            {
                break;
            }


            cursor =
                next;
        }


        free(
            section);
    }


    void ParseRuntimeMailboxDirection(
        const char* mailboxBlock,
        const char* tagName,
        EtherCatRuntimeMailboxDirectionConfig& direction)
    {
        direction =
            EtherCatRuntimeMailboxDirectionConfig();


        char* block =
            CopyXmlElementBlockAllocated(
                mailboxBlock,
                tagName);


        if (block == nullptr)
        {
            return;
        }


        direction.present =
            true;


        char buffer[256] =
        {
            0
        };


        if (CopyXmlElementValue(
            block,
            "SmIndex",
            buffer,
            sizeof(buffer)))
        {
            direction.smIndex =
                static_cast<int>(
                    ParseInt64Flexible(
                        buffer));
        }


        memset(
            buffer,
            0,
            sizeof(buffer));


        if (CopyXmlElementValue(
            block,
            "StartAddress",
            buffer,
            sizeof(buffer)))
        {
            direction.startAddress =
                static_cast<uint16_t>(
                    ParseUInt32Flexible(
                        buffer));
        }


        memset(
            buffer,
            0,
            sizeof(buffer));


        if (CopyXmlElementValue(
            block,
            "Length",
            buffer,
            sizeof(buffer)))
        {
            direction.length =
                static_cast<uint16_t>(
                    ParseUInt32Flexible(
                        buffer));
        }


        memset(
            buffer,
            0,
            sizeof(buffer));


        if (CopyXmlElementValue(
            block,
            "ControlByte",
            buffer,
            sizeof(buffer)))
        {
            direction.controlByte =
                static_cast<uint8_t>(
                    ParseUInt32Flexible(
                        buffer));
        }


        memset(
            buffer,
            0,
            sizeof(buffer));


        if (CopyXmlElementValue(
            block,
            "Enable",
            buffer,
            sizeof(buffer)))
        {
            direction.enabled =
                ParseUInt32Flexible(
                    buffer) !=
                0;
        }


        free(
            block);
    }


    void ParseRuntimeMailbox(
        const char* slaveContent,
        EtherCatSlave& slave)
    {
        slave.runtimeMailbox =
            EtherCatRuntimeMailboxConfig();


        char* section =
            CopyXmlElementBlockAllocated(
                slaveContent,
                "Mailbox");


        if (section == nullptr)
        {
            return;
        }


        char buffer[256] =
        {
            0
        };


        if (CopyXmlElementValue(
            section,
            "Present",
            buffer,
            sizeof(buffer)))
        {
            slave.runtimeMailbox.present =
                ParseUInt32Flexible(
                    buffer) !=
                0;
        }


        ParseRuntimeMailboxDirection(
            section,
            "Out",
            slave.runtimeMailbox.out);


        ParseRuntimeMailboxDirection(
            section,
            "In",
            slave.runtimeMailbox.in);


        if (slave.runtimeMailbox.out.present ||
            slave.runtimeMailbox.in.present)
        {
            slave.runtimeMailbox.present =
                true;
        }


        free(
            section);
    }


    void ParseRuntimeFmmus(
        const char* slaveContent,
        EtherCatSlave& slave)
    {
        slave.runtimeFmmus.clear();

        slave.runtimeFmmuSchemaPresent =
            false;


        char* section =
            CopyXmlElementBlockAllocated(
                slaveContent,
                "Fmmus");


        if (section == nullptr)
        {
            return;
        }


        slave.runtimeFmmuSchemaPresent =
            true;


        const char* cursor =
            section;


        while (true)
        {
            char* fmmuStart =
                FindXmlElementStart(
                    const_cast<char*>(
                        cursor),
                    "Fmmu");


            if (fmmuStart == nullptr)
            {
                break;
            }


            char* fmmuBlock =
                CopyXmlElementBlockAllocated(
                    fmmuStart,
                    "Fmmu");


            if (fmmuBlock == nullptr)
            {
                break;
            }


            EtherCatRuntimeFmmuConfig fmmu;


            char buffer[256] =
            {
                0
            };


            if (CopyXmlElementValue(
                fmmuBlock,
                "Index",
                buffer,
                sizeof(buffer)))
            {
                fmmu.index =
                    static_cast<int>(
                        ParseInt64Flexible(
                            buffer));
            }


            CopyXmlElementValue(
                fmmuBlock,
                "Direction",
                fmmu.direction,
                sizeof(
                    fmmu.direction));


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                fmmuBlock,
                "LogicalStartAddress",
                buffer,
                sizeof(buffer)))
            {
                fmmu.logicalStartAddress =
                    ParseUInt32Flexible(
                        buffer);
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                fmmuBlock,
                "LogicalLength",
                buffer,
                sizeof(buffer)))
            {
                fmmu.logicalLength =
                    static_cast<uint16_t>(
                        ParseUInt32Flexible(
                            buffer));
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                fmmuBlock,
                "LogicalStartBit",
                buffer,
                sizeof(buffer)))
            {
                fmmu.logicalStartBit =
                    static_cast<uint8_t>(
                        ParseUInt32Flexible(
                            buffer));
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                fmmuBlock,
                "LogicalEndBit",
                buffer,
                sizeof(buffer)))
            {
                fmmu.logicalEndBit =
                    static_cast<uint8_t>(
                        ParseUInt32Flexible(
                            buffer));
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                fmmuBlock,
                "PhysicalStartAddress",
                buffer,
                sizeof(buffer)))
            {
                fmmu.physicalStartAddress =
                    static_cast<uint16_t>(
                        ParseUInt32Flexible(
                            buffer));
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                fmmuBlock,
                "PhysicalStartBit",
                buffer,
                sizeof(buffer)))
            {
                fmmu.physicalStartBit =
                    static_cast<uint8_t>(
                        ParseUInt32Flexible(
                            buffer));
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                fmmuBlock,
                "Type",
                buffer,
                sizeof(buffer)))
            {
                fmmu.type =
                    static_cast<uint8_t>(
                        ParseUInt32Flexible(
                            buffer));
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                fmmuBlock,
                "Enable",
                buffer,
                sizeof(buffer)))
            {
                fmmu.enabled =
                    ParseUInt32Flexible(
                        buffer) !=
                    0;
            }


            slave.runtimeFmmus.push_back(
                fmmu);


            const char* next =
                strstr(
                    fmmuStart,
                    "</Fmmu>");


            free(
                fmmuBlock);


            if (next == nullptr)
            {
                break;
            }


            cursor =
                next +
                strlen(
                    "</Fmmu>");
        }


        free(
            section);
    }


    void ParseRuntimePdoSection(
        const char* slaveContent,
        const char* sectionName,
        std::vector<EtherCatRuntimePdoConfig>& pdos)
    {
        pdos.clear();


        char* section =
            CopyXmlElementBlockAllocated(
                slaveContent,
                sectionName);


        if (section == nullptr)
        {
            return;
        }


        const char* cursor =
            section;


        while (true)
        {
            char* pdoStart =
                FindXmlElementStart(
                    const_cast<char*>(
                        cursor),
                    "Pdo");


            if (pdoStart == nullptr)
            {
                break;
            }


            char* pdoBlock =
                CopyXmlElementBlockAllocated(
                    pdoStart,
                    "Pdo");


            if (pdoBlock == nullptr)
            {
                break;
            }


            EtherCatRuntimePdoConfig pdo;


            char buffer[256] =
            {
                0
            };


            if (CopyXmlElementValue(
                pdoBlock,
                "Index",
                buffer,
                sizeof(buffer)))
            {
                pdo.index =
                    static_cast<uint16_t>(
                        ParseUInt32Flexible(
                            buffer));
            }


            CopyXmlElementValue(
                pdoBlock,
                "Name",
                pdo.name,
                sizeof(pdo.name));


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                pdoBlock,
                "SyncManagerIndex",
                buffer,
                sizeof(buffer)))
            {
                pdo.syncManagerIndex =
                    static_cast<int>(
                        ParseInt64Flexible(
                            buffer));
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                pdoBlock,
                "BitSize",
                buffer,
                sizeof(buffer)))
            {
                pdo.bitSize =
                    static_cast<uint16_t>(
                        ParseUInt32Flexible(
                            buffer));
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                pdoBlock,
                "ByteSize",
                buffer,
                sizeof(buffer)))
            {
                pdo.byteSize =
                    static_cast<uint16_t>(
                        ParseUInt32Flexible(
                            buffer));
            }


            char* entriesBlock =
                CopyXmlElementBlockAllocated(
                    pdoBlock,
                    "Entries");


            if (entriesBlock != nullptr)
            {
                const char* entryCursor =
                    entriesBlock;


                while (true)
                {
                    char* entryStart =
                        FindXmlElementStart(
                            const_cast<char*>(
                                entryCursor),
                            "Entry");


                    if (entryStart == nullptr)
                    {
                        break;
                    }


                    char* entryBlock =
                        CopyXmlElementBlockAllocated(
                            entryStart,
                            "Entry");


                    if (entryBlock == nullptr)
                    {
                        break;
                    }


                    EtherCatRuntimePdoEntryConfig entry;


                    memset(
                        buffer,
                        0,
                        sizeof(buffer));


                    if (CopyXmlElementValue(
                        entryBlock,
                        "Index",
                        buffer,
                        sizeof(buffer)))
                    {
                        entry.index =
                            static_cast<uint16_t>(
                                ParseUInt32Flexible(
                                    buffer));
                    }


                    memset(
                        buffer,
                        0,
                        sizeof(buffer));


                    if (CopyXmlElementValue(
                        entryBlock,
                        "SubIndex",
                        buffer,
                        sizeof(buffer)))
                    {
                        entry.subIndex =
                            static_cast<uint8_t>(
                                ParseUInt32Flexible(
                                    buffer));
                    }


                    memset(
                        buffer,
                        0,
                        sizeof(buffer));


                    if (CopyXmlElementValue(
                        entryBlock,
                        "BitLength",
                        buffer,
                        sizeof(buffer)))
                    {
                        entry.bitLength =
                            static_cast<uint8_t>(
                                ParseUInt32Flexible(
                                    buffer));
                    }


                    memset(
                        buffer,
                        0,
                        sizeof(buffer));


                    if (CopyXmlElementValue(
                        entryBlock,
                        "MappingValue",
                        buffer,
                        sizeof(buffer)))
                    {
                        entry.mappingValue =
                            ParseUInt32Flexible(
                                buffer);
                    }


                    CopyXmlElementValue(
                        entryBlock,
                        "Name",
                        entry.name,
                        sizeof(entry.name));


                    CopyXmlElementValue(
                        entryBlock,
                        "DataType",
                        entry.dataType,
                        sizeof(entry.dataType));


                    pdo.entries.push_back(
                        entry);


                    const char* nextEntry =
                        MoveAfterXmlElement(
                            entryStart,
                            "Entry");


                    free(
                        entryBlock);


                    if (nextEntry == nullptr)
                    {
                        break;
                    }


                    entryCursor =
                        nextEntry;
                }


                free(
                    entriesBlock);
            }


            pdos.push_back(
                pdo);


            const char* nextPdo =
                MoveAfterXmlElement(
                    pdoStart,
                    "Pdo");


            free(
                pdoBlock);


            if (nextPdo == nullptr)
            {
                break;
            }


            cursor =
                nextPdo;
        }


        free(
            section);
    }


    void ParseRuntimeInitCommands(
        const char* slaveContent,
        EtherCatSlave& slave)
    {
        slave.runtimeInitCommands.clear();


        char* section =
            CopyXmlElementBlockAllocated(
                slaveContent,
                "InitCommands");


        if (section == nullptr)
        {
            return;
        }


        const char* cursor =
            section;


        while (true)
        {
            char* commandStart =
                FindXmlElementStart(
                    const_cast<char*>(
                        cursor),
                    "InitCommand");


            if (commandStart == nullptr)
            {
                break;
            }


            char* commandBlock =
                CopyXmlElementBlockAllocated(
                    commandStart,
                    "InitCommand");


            if (commandBlock == nullptr)
            {
                break;
            }


            EtherCatRuntimeInitCommandConfig command;


            char buffer[4096] =
            {
                0
            };


            CopyXmlElementValue(
                commandBlock,
                "Source",
                command.source,
                sizeof(command.source));


            memset(
                buffer,
                0,
                sizeof(buffer));


            command.hasApply =
                CopyXmlElementValue(
                    commandBlock,
                    "Apply",
                    buffer,
                    sizeof(buffer));


            if (command.hasApply)
            {
                command.apply =
                    ParseUInt32Flexible(
                        buffer) != 0;
            }


            CopyXmlElementValue(
                commandBlock,
                "Transition",
                command.transition,
                sizeof(command.transition));


            if (CopyXmlElementValue(
                commandBlock,
                "Index",
                buffer,
                sizeof(buffer)))
            {
                command.index =
                    static_cast<uint16_t>(
                        ParseUInt32Flexible(
                            buffer));
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                commandBlock,
                "SubIndex",
                buffer,
                sizeof(buffer)))
            {
                command.subIndex =
                    static_cast<uint8_t>(
                        ParseUInt32Flexible(
                            buffer));
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            if (CopyXmlElementValue(
                commandBlock,
                "Data",
                buffer,
                sizeof(buffer)))
            {
                ParseHexBytes(
                    buffer,
                    command.data);
            }


            CopyXmlElementValue(
                commandBlock,
                "Comment",
                command.comment,
                sizeof(command.comment));


            slave.runtimeInitCommands.push_back(
                command);


            const char* next =
                MoveAfterXmlElement(
                    commandStart,
                    "InitCommand");


            free(
                commandBlock);


            if (next == nullptr)
            {
                break;
            }


            cursor =
                next;
        }


        free(
            section);
    }
}


// ============================================================================
// EtherCatEni
// ============================================================================

EtherCatEni::EtherCatEni()
{
}


EtherCatEni::~EtherCatEni()
{
}


// ============================================================================
// GetSlaves
// ============================================================================

const std::vector<EtherCatSlave>&
EtherCatEni::GetSlaves() const
{
    return
        m_slaves;
}


// ============================================================================
// Parse_XML_Value
//
// 保留原 Header API。
// 內部已升級為可接受 Opening Tag Attribute。
// ============================================================================

void EtherCatEni::Parse_XML_Value(
    const char* content,
    const char* tag,
    char* outBuffer)
{
    CopyXmlElementValue(
        content,
        tag,
        outBuffer,
        128);
}


// ============================================================================
// LoadXml
//
// Runtime XML Parser
//
// 支援：
//
// <Slave>
// <Slave Position="0" ConfiguredAddress="4097">
//
// 因此 Configurator 未來不需要為了舊 Parser
// 強制把 Slave Attribute 移除。
//
// 重要安全修正：
//
// EtherCatSlave 內含 std::vector<InitCmd>。
// 不可再使用：
//
// memset(&newSlave, 0, sizeof(EtherCatSlave));
//
// 改由正常 C++ value initialization。
// ============================================================================

int EtherCatEni::LoadXml(
    const char* filename)
{
    m_slaves.clear();


    if (filename == nullptr ||
        filename[0] == '\0')
    {
        DEBUG_PRINT(
            "[ENI] ERROR: Empty Runtime XML filename.\n");

        return 0;
    }


    FILE* f =
        fopen(
            filename,
            "rb");


    if (f == nullptr)
    {
        DEBUG_PRINT(
            "[ENI] ERROR: Cannot open Runtime XML:\n%s\n",
            filename);

        return 0;
    }


    fseek(
        f,
        0,
        SEEK_END);

    const long fileSize =
        ftell(
            f);

    fseek(
        f,
        0,
        SEEK_SET);


    if (fileSize <= 0 ||
        fileSize >
        MAX_RUNTIME_XML_FILE_SIZE)
    {
        DEBUG_PRINT(
            "[ENI] ERROR: Invalid Runtime XML size: %ld bytes.\n",
            fileSize);

        fclose(
            f);

        return 0;
    }


    char* xmlBuffer =
        static_cast<char*>(
            malloc(
                static_cast<size_t>(
                    fileSize) +
                1));


    if (xmlBuffer == nullptr)
    {
        DEBUG_PRINT(
            "[ENI] ERROR: Runtime XML allocation failed. Size:%ld\n",
            fileSize);

        fclose(
            f);

        return 0;
    }


    const size_t readSize =
        fread(
            xmlBuffer,
            1,
            static_cast<size_t>(
                fileSize),
            f);


    fclose(
        f);


    if (readSize == 0)
    {
        DEBUG_PRINT(
            "[ENI] ERROR: Runtime XML read returned 0 bytes.\n");

        free(
            xmlBuffer);

        return 0;
    }


    xmlBuffer[readSize] =
        '\0';


    DEBUG_PRINT(
        ">>> Loading OSCARMAX EtherCAT Runtime Config...\n");

    DEBUG_PRINT(
        "[ENI] File:%s | Size:%u bytes\n",
        filename,
        static_cast<unsigned int>(
            readSize));


    int count =
        0;


    char* pCurrent =
        xmlBuffer;


    while (true)
    {
        // --------------------------------------------------------------------
        // 支援：
        //
        // <Slave>
        //
        // 以及：
        //
        // <Slave Position="0" ConfiguredAddress="4097">
        // --------------------------------------------------------------------

        char* pSlaveStart =
            FindXmlElementStart(
                pCurrent,
                "Slave");


        if (pSlaveStart == nullptr)
        {
            break;
        }


        char* pSlaveOpenEnd =
            strchr(
                pSlaveStart,
                '>');


        if (pSlaveOpenEnd == nullptr)
        {
            DEBUG_PRINT(
                "[ENI] ERROR: Slave opening tag is incomplete.\n");

            break;
        }


        char* pSlaveEnd =
            strstr(
                pSlaveOpenEnd,
                "</Slave>");


        if (pSlaveEnd == nullptr)
        {
            DEBUG_PRINT(
                "[ENI] ERROR: Missing </Slave> at Slot:%d\n",
                count);

            break;
        }


        // --------------------------------------------------------------------
        // 不再使用固定 4096-byte Slave Buffer。
        //
        // 未來 Profile / PDO Summary / InitCmd 增加時，
        // 單一 Slave Block 超過 4095 bytes 也不會被靜默截斷。
        //
        // 這裡只在初始化期間 malloc/free，不在 RT PDO Loop 中執行。
        // --------------------------------------------------------------------

        const size_t slaveLength =
            static_cast<size_t>(
                pSlaveEnd -
                pSlaveStart);


        char* slaveContent =
            static_cast<char*>(
                malloc(
                    slaveLength +
                    1));


        if (slaveContent == nullptr)
        {
            DEBUG_PRINT(
                "[ENI] ERROR: Slave buffer allocation failed. Slot:%d Size:%u\n",
                count,
                static_cast<unsigned int>(
                    slaveLength));

            break;
        }


        memcpy(
            slaveContent,
            pSlaveStart,
            slaveLength);

        slaveContent[slaveLength] =
            '\0';


        // --------------------------------------------------------------------
        // 正常 C++ 初始化。
        //
        // EtherCatSlave 內含 std::vector，
        // 不可以對整個 Object memset。
        // --------------------------------------------------------------------

        EtherCatSlave newSlave = {};


        char buffer[128] =
        {
            0
        };


        // ====================================================================
        // 1. Basic Identity
        // ====================================================================

        Parse_XML_Value(
            slaveContent,
            "Name",
            newSlave.name);

        Parse_XML_Value(
            slaveContent,
            "Type",
            newSlave.type);


        memset(
            buffer,
            0,
            sizeof(buffer));

        Parse_XML_Value(
            slaveContent,
            "ProductCode",
            buffer);

        newSlave.productCode =
            ParseUInt32Flexible(
                buffer);


        memset(
            buffer,
            0,
            sizeof(buffer));

        Parse_XML_Value(
            slaveContent,
            "VendorId",
            buffer);

        newSlave.vendorId =
            ParseUInt32Flexible(
                buffer);


        // --------------------------------------------------------------------
        // Runtime Config Revision
        // --------------------------------------------------------------------

        memset(
            buffer,
            0,
            sizeof(buffer));

        Parse_XML_Value(
            slaveContent,
            "Revision",
            buffer);

        newSlave.hasRevision =
            buffer[0] != '\0';

        if (newSlave.hasRevision)
        {
            newSlave.revision =
                ParseUInt32Flexible(
                    buffer);
        }


        // --------------------------------------------------------------------
        // Runtime Configured Station Address
        //
        // 目前 Configurator 會在 <Info> 內輸出：
        //
        // <ConfiguredAddress>4097</ConfiguredAddress>
        //
        // Parser 仍同時支援未來 <Slave ...> Attribute，
        // 但 Stage 1B 正式比對使用這個 Runtime 欄位。
        // --------------------------------------------------------------------

        memset(
            buffer,
            0,
            sizeof(buffer));

        Parse_XML_Value(
            slaveContent,
            "ConfiguredAddress",
            buffer);

        newSlave.hasConfiguredAddress =
            buffer[0] != '\0';

        if (newSlave.hasConfiguredAddress)
        {
            newSlave.configuredAddress =
                static_cast<uint16_t>(
                    ParseUInt32Flexible(
                        buffer));
        }


        // ====================================================================
        // 2. Process Data Input
        //
        // Slave -> Master / TxPDO
        // ====================================================================

        newSlave.configAddrIn =
            0;

        newSlave.inputBitLength =
            0;


        char* pInputStart =
            FindXmlElementStart(
                slaveContent,
                "Input");


        if (pInputStart != nullptr)
        {
            char* pInputEnd =
                strstr(
                    pInputStart,
                    "</Input>");


            if (pInputEnd != nullptr)
            {
                const size_t inputLength =
                    static_cast<size_t>(
                        pInputEnd -
                        pInputStart);


                char* inputBlock =
                    static_cast<char*>(
                        malloc(
                            inputLength +
                            1));


                if (inputBlock != nullptr)
                {
                    memcpy(
                        inputBlock,
                        pInputStart,
                        inputLength);

                    inputBlock[inputLength] =
                        '\0';


                    memset(
                        buffer,
                        0,
                        sizeof(buffer));

                    Parse_XML_Value(
                        inputBlock,
                        "PhysAddr",
                        buffer);

                    newSlave.configAddrIn =
                        static_cast<uint16_t>(
                            ParseUInt32Flexible(
                                buffer));


                    memset(
                        buffer,
                        0,
                        sizeof(buffer));

                    Parse_XML_Value(
                        inputBlock,
                        "BitSize",
                        buffer);

                    newSlave.inputBitLength =
                        ParseUInt32Flexible(
                            buffer);


                    free(
                        inputBlock);
                }
            }
        }


        // ====================================================================
        // 3. Process Data Output
        //
        // Master -> Slave / RxPDO
        // ====================================================================

        newSlave.configAddrOut =
            0;

        newSlave.outputBitLength =
            0;


        char* pOutputStart =
            FindXmlElementStart(
                slaveContent,
                "Output");


        if (pOutputStart != nullptr)
        {
            char* pOutputEnd =
                strstr(
                    pOutputStart,
                    "</Output>");


            if (pOutputEnd != nullptr)
            {
                const size_t outputLength =
                    static_cast<size_t>(
                        pOutputEnd -
                        pOutputStart);


                char* outputBlock =
                    static_cast<char*>(
                        malloc(
                            outputLength +
                            1));


                if (outputBlock != nullptr)
                {
                    memcpy(
                        outputBlock,
                        pOutputStart,
                        outputLength);

                    outputBlock[outputLength] =
                        '\0';


                    memset(
                        buffer,
                        0,
                        sizeof(buffer));

                    Parse_XML_Value(
                        outputBlock,
                        "PhysAddr",
                        buffer);

                    newSlave.configAddrOut =
                        static_cast<uint16_t>(
                            ParseUInt32Flexible(
                                buffer));


                    memset(
                        buffer,
                        0,
                        sizeof(buffer));

                    Parse_XML_Value(
                        outputBlock,
                        "BitSize",
                        buffer);

                    newSlave.outputBitLength =
                        ParseUInt32Flexible(
                            buffer);


                    free(
                        outputBlock);
                }
            }
        }


        // ====================================================================
        // Stage 5A - Runtime XML V2 Metadata
        //
        // Current Configurator already exports:
        // Profile / PdoSummary / Dc / Watchdog.
        //
        // Parse + Store + Diagnostic only.
        // No hardware write behavior changes in Stage 5A.
        // ====================================================================

        // --------------------------------------------------------------------
        // Profile
        // --------------------------------------------------------------------

        char* profileBlock =
            CopyXmlElementBlockAllocated(
                slaveContent,
                "Profile");


        if (profileBlock != nullptr)
        {
            newSlave.runtimeProfile.present =
                true;


            CopyXmlElementValue(
                profileBlock,
                "Name",
                newSlave.runtimeProfile.name,
                sizeof(
                    newSlave.runtimeProfile.name));


            newSlave.runtimeProfile.hasPdoMappingMode =
                CopyXmlElementValue(
                    profileBlock,
                    "PdoMappingMode",
                    newSlave.runtimeProfile.pdoMappingMode,
                    sizeof(
                        newSlave.runtimeProfile.pdoMappingMode));


            memset(
                buffer,
                0,
                sizeof(buffer));


            newSlave.runtimeProfile.hasRxPdoAssignIndex =
                CopyXmlElementValue(
                    profileBlock,
                    "RxPdoAssignIndex",
                    buffer,
                    sizeof(buffer));


            if (newSlave.runtimeProfile.hasRxPdoAssignIndex)
            {
                newSlave.runtimeProfile.rxPdoAssignIndex =
                    static_cast<uint16_t>(
                        ParseUInt32Flexible(
                            buffer));
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            newSlave.runtimeProfile.hasTxPdoAssignIndex =
                CopyXmlElementValue(
                    profileBlock,
                    "TxPdoAssignIndex",
                    buffer,
                    sizeof(buffer));


            if (newSlave.runtimeProfile.hasTxPdoAssignIndex)
            {
                newSlave.runtimeProfile.txPdoAssignIndex =
                    static_cast<uint16_t>(
                        ParseUInt32Flexible(
                            buffer));
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            newSlave.runtimeProfile.hasOutputSmControl =
                CopyXmlElementValue(
                    profileBlock,
                    "OutputSmControl",
                    buffer,
                    sizeof(buffer));


            if (newSlave.runtimeProfile.hasOutputSmControl)
            {
                newSlave.runtimeProfile.outputSmControl =
                    static_cast<uint8_t>(
                        ParseUInt32Flexible(
                            buffer));
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            newSlave.runtimeProfile.hasInputSmControl =
                CopyXmlElementValue(
                    profileBlock,
                    "InputSmControl",
                    buffer,
                    sizeof(buffer));


            if (newSlave.runtimeProfile.hasInputSmControl)
            {
                newSlave.runtimeProfile.inputSmControl =
                    static_cast<uint8_t>(
                        ParseUInt32Flexible(
                            buffer));
            }


            free(
                profileBlock);
        }


        // --------------------------------------------------------------------
        // PDO Summary
        // --------------------------------------------------------------------

        char* pdoSummaryBlock =
            CopyXmlElementBlockAllocated(
                slaveContent,
                "PdoSummary");


        if (pdoSummaryBlock != nullptr)
        {
            char pdoListBuffer[2048] =
            {
                0
            };


            if (CopyXmlElementValue(
                pdoSummaryBlock,
                "RxPdoIndexes",
                pdoListBuffer,
                sizeof(pdoListBuffer)))
            {
                ParseUInt16ListFlexible(
                    pdoListBuffer,
                    newSlave.runtimeProfile.rxPdoIndexes);
            }


            memset(
                pdoListBuffer,
                0,
                sizeof(pdoListBuffer));


            if (CopyXmlElementValue(
                pdoSummaryBlock,
                "TxPdoIndexes",
                pdoListBuffer,
                sizeof(pdoListBuffer)))
            {
                ParseUInt16ListFlexible(
                    pdoListBuffer,
                    newSlave.runtimeProfile.txPdoIndexes);
            }


            free(
                pdoSummaryBlock);
        }


        // --------------------------------------------------------------------
        // DC
        // --------------------------------------------------------------------

        char* dcBlock =
            CopyXmlElementBlockAllocated(
                slaveContent,
                "Dc");


        if (dcBlock != nullptr)
        {
            newSlave.runtimeDc.present =
                true;


            CopyXmlElementValue(
                dcBlock,
                "Mode",
                newSlave.runtimeDc.mode,
                sizeof(
                    newSlave.runtimeDc.mode));


            memset(
                buffer,
                0,
                sizeof(buffer));


            newSlave.runtimeDc.hasCycleTimeNs =
                CopyXmlElementValue(
                    dcBlock,
                    "CycleTimeNs",
                    buffer,
                    sizeof(buffer));


            if (newSlave.runtimeDc.hasCycleTimeNs)
            {
                newSlave.runtimeDc.cycleTimeNs =
                    ParseUInt32Flexible(
                        buffer);
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            newSlave.runtimeDc.hasShiftTimeNs =
                CopyXmlElementValue(
                    dcBlock,
                    "ShiftTimeNs",
                    buffer,
                    sizeof(buffer));


            if (newSlave.runtimeDc.hasShiftTimeNs)
            {
                newSlave.runtimeDc.shiftTimeNs =
                    ParseInt64Flexible(
                        buffer);
            }


            memset(
                buffer,
                0,
                sizeof(buffer));


            newSlave.runtimeDc.hasReferenceClock =
                CopyXmlElementValue(
                    dcBlock,
                    "ReferenceClock",
                    buffer,
                    sizeof(buffer));


            if (newSlave.runtimeDc.hasReferenceClock)
            {
                newSlave.runtimeDc.referenceClock =
                    ParseUInt32Flexible(
                        buffer) != 0;
            }


            free(
                dcBlock);
        }


        // --------------------------------------------------------------------
        // Watchdog
        // --------------------------------------------------------------------

        char* watchdogBlock =
            CopyXmlElementBlockAllocated(
                slaveContent,
                "Watchdog");


        if (watchdogBlock != nullptr)
        {
            newSlave.runtimeWatchdog.present =
                true;


            memset(
                buffer,
                0,
                sizeof(buffer));


            newSlave.runtimeWatchdog.hasProcessDataTimeoutMs =
                CopyXmlElementValue(
                    watchdogBlock,
                    "ProcessDataTimeoutMs",
                    buffer,
                    sizeof(buffer));


            if (newSlave.runtimeWatchdog.hasProcessDataTimeoutMs)
            {
                newSlave.runtimeWatchdog.processDataTimeoutMs =
                    ParseUInt32Flexible(
                        buffer);
            }


            free(
                watchdogBlock);
        }


        // --------------------------------------------------------------------
        // Stage 5B complete executable schema.
        //
        // Parse only. No EtherCAT hardware write is performed here.
        // --------------------------------------------------------------------

        ParseRuntimeSyncManagers(
            slaveContent,
            newSlave);


        // Stage 7A Runtime Process Image Binding.
        // Parse only. BuildIoMap() remains unchanged.
        ParseRuntimeProcessImageBinding(
            slaveContent,
            newSlave);


        // Stage 11A Composite Application Bindings.
        // Parse only. Current Stage7B simple binding remains active.
        ParseRuntimeApplicationBindings(
            slaveContent,
            newSlave);


        // Stage 11C.1 Project Composite Runtime Shadow.
        // Parse only. Active Stage11A/Stage7B binding remains unchanged.
        ParseRuntimeCompositeBindingsShadow(
            slaveContent,
            newSlave);


        // Stage 6A Runtime Transport Schema.
        // Parse only. No EtherCAT hardware write is performed here.
        ParseRuntimeMailbox(
            slaveContent,
            newSlave);


        ParseRuntimeFmmus(
            slaveContent,
            newSlave);


        ParseRuntimePdoSection(
            slaveContent,
            "RxPdos",
            newSlave.runtimeRxPdos);


        ParseRuntimePdoSection(
            slaveContent,
            "TxPdos",
            newSlave.runtimeTxPdos);


        ParseRuntimeInitCommands(
            slaveContent,
            newSlave);


        size_t rxEntryCount =
            0;


        for (const auto& pdo :
            newSlave.runtimeRxPdos)
        {
            rxEntryCount +=
                pdo.entries.size();
        }


        size_t txEntryCount =
            0;


        for (const auto& pdo :
            newSlave.runtimeTxPdos)
        {
            txEntryCount +=
                pdo.entries.size();
        }


        size_t appliedInitCommandCount =
            0;


        for (const auto& command :
            newSlave.runtimeInitCommands)
        {
            if (command.apply)
            {
                appliedInitCommandCount++;
            }
        }


        DEBUG_PRINT(
            "[ENI-V2-DETAIL] S%d | "
            "SM:%u | "
            "RxPDO:%u RxEntry:%u | "
            "TxPDO:%u TxEntry:%u | "
            "InitCmd:%u Apply:%u\n",

            count,

            (unsigned int)
            newSlave.runtimeSyncManagers.size(),

            (unsigned int)
            newSlave.runtimeRxPdos.size(),

            (unsigned int)
            rxEntryCount,

            (unsigned int)
            newSlave.runtimeTxPdos.size(),

            (unsigned int)
            txEntryCount,

            (unsigned int)
            newSlave.runtimeInitCommands.size(),

            (unsigned int)
            appliedInitCommandCount);


        DEBUG_PRINT(
            "[ENI-V2-TRANSPORT] S%d | "
            "Mailbox:%s "
            "Out:%s SM:%d 0x%04X/%u | "
            "In:%s SM:%d 0x%04X/%u | "
            "FMMUSchema:%s FMMU:%u\n",

            count,

            newSlave.runtimeMailbox.present
            ? "YES"
            : "NO",

            newSlave.runtimeMailbox.out.present
            ? "YES"
            : "NO",

            newSlave.runtimeMailbox.out.smIndex,

            (unsigned int)
            newSlave.runtimeMailbox.out.startAddress,

            (unsigned int)
            newSlave.runtimeMailbox.out.length,

            newSlave.runtimeMailbox.in.present
            ? "YES"
            : "NO",

            newSlave.runtimeMailbox.in.smIndex,

            (unsigned int)
            newSlave.runtimeMailbox.in.startAddress,

            (unsigned int)
            newSlave.runtimeMailbox.in.length,

            newSlave.runtimeFmmuSchemaPresent
            ? "YES"
            : "NO",

            (unsigned int)
            newSlave.runtimeFmmus.size());


        DEBUG_PRINT(
            "[ENI-V2-BINDING] S%d | "
            "Present:%s | Kind:%s | Abi:%s | "
            "Out:%d/%u | In:%d/%u | "
            "Element:%u | Channels:%u\n",

            count,

            newSlave.runtimeProcessImageBinding.present
            ? "YES"
            : "NO",

            newSlave.runtimeProcessImageBinding.kind[0] != '\0'
            ? newSlave.runtimeProcessImageBinding.kind
            : "N/A",

            newSlave.runtimeProcessImageBinding.abi[0] != '\0'
            ? newSlave.runtimeProcessImageBinding.abi
            : "N/A",

            newSlave.runtimeProcessImageBinding.outputOffset,

            (unsigned int)
            newSlave.runtimeProcessImageBinding.outputBytes,

            newSlave.runtimeProcessImageBinding.inputOffset,

            (unsigned int)
            newSlave.runtimeProcessImageBinding.inputBytes,

            (unsigned int)
            newSlave.runtimeProcessImageBinding.elementBytes,

            (unsigned int)
            newSlave.runtimeProcessImageBinding.channelCount);


        DEBUG_PRINT(
            "[ENI-V2-APP-BINDINGS] S%d | "
            "Schema:%s | Mode:%s | Bindings:%u\n",

            count,

            newSlave.runtimeApplicationBindingsSchemaPresent
            ? "YES"
            : "NO",

            newSlave.runtimeApplicationBindingsMode[0] != '\0'
            ? newSlave.runtimeApplicationBindingsMode
            : "N/A",

            (unsigned int)
            newSlave.runtimeApplicationBindings.size());


        for (size_t bindingIndex = 0;
            bindingIndex <
            newSlave.runtimeApplicationBindings.size();
            bindingIndex++)
        {
            const auto& binding =
                newSlave.runtimeApplicationBindings[
                    bindingIndex];


            DEBUG_PRINT(
                "[ENI-V2-APP-BINDING] S%d B%u | "
                "Id:%s Kind:%s Legacy:%s Abi:%s If:%s Type:%s Sample:%s Unit:%s Axis:%s | "
                "OutBit:%d/%u | InBit:%d/%u | "
                "ElementBits:%u Channels:%u\n",

                count,

                (unsigned int)
                bindingIndex,

                binding.id[0] != '\0'
                ? binding.id
                : "N/A",

                binding.kind[0] != '\0'
                ? binding.kind
                : "N/A",

                binding.legacyKind[0] != '\0'
                ? binding.legacyKind
                : "N/A",

                binding.abi[0] != '\0'
                ? binding.abi
                : "N/A",

                binding.interfaceType[0] != '\0'
                ? binding.interfaceType
                : "N/A",

                binding.dataType[0] != '\0'
                ? binding.dataType
                : "N/A",

                binding.sampleMode[0] != '\0'
                ? binding.sampleMode
                : "N/A",

                binding.unit[0] != '\0'
                ? binding.unit
                : "N/A",

                binding.axisRef[0] != '\0'
                ? binding.axisRef
                : "-",

                binding.outputBitOffset,

                (unsigned int)
                binding.outputBitLength,

                binding.inputBitOffset,

                (unsigned int)
                binding.inputBitLength,

                (unsigned int)
                binding.elementBits,

                (unsigned int)
                binding.channelCount);
        }


        DEBUG_PRINT(
            "[ENI-V2-COMPOSITE-SHADOW] S%d | "
            "Schema:%s | Mode:%s | Bindings:%u | "
            "OutBase:%d | InBase:%d | "
            "Source:PROJECT_V1_3 | ShadowOnly:YES\n",

            count,

            newSlave.runtimeCompositeBindingsShadowSchemaPresent
            ? "YES"
            : "NO",

            newSlave.runtimeCompositeBindingsShadowMode[0] != '\0'
            ? newSlave.runtimeCompositeBindingsShadowMode
            : "N/A",

            (unsigned int)
            newSlave.runtimeCompositeBindingsShadow.size(),

            newSlave.runtimeCompositeBindingsShadowOutputBaseBit,

            newSlave.runtimeCompositeBindingsShadowInputBaseBit);


        for (size_t bindingIndex = 0;
            bindingIndex <
            newSlave.runtimeCompositeBindingsShadow.size();
            bindingIndex++)
        {
            const auto& binding =
                newSlave.runtimeCompositeBindingsShadow[
                    bindingIndex];


            DEBUG_PRINT(
                "[ENI-V2-COMPOSITE-SHADOW-BINDING] "
                "S%d B%u | Id:%s Name:%s Sort:%d | "
                "Kind:%s If:%s Type:%s Sample:%s Unit:%s Axis:%s | "
                "OutRel:%d Abs:%d/%u | "
                "InRel:%d Abs:%d/%u | "
                "Elem:%u Ch:%u\n",

                count,

                (unsigned int)
                bindingIndex,

                binding.id[0] != '\0'
                ? binding.id
                : "N/A",

                binding.name[0] != '\0'
                ? binding.name
                : "-",

                binding.sortOrder,

                binding.kind[0] != '\0'
                ? binding.kind
                : "N/A",

                binding.interfaceType[0] != '\0'
                ? binding.interfaceType
                : "N/A",

                binding.dataType[0] != '\0'
                ? binding.dataType
                : "N/A",

                binding.sampleMode[0] != '\0'
                ? binding.sampleMode
                : "N/A",

                binding.unit[0] != '\0'
                ? binding.unit
                : "N/A",

                binding.axisRef[0] != '\0'
                ? binding.axisRef
                : "-",

                binding.outputRelativeBitOffset,

                binding.outputBitOffset,

                (unsigned int)
                binding.outputBitLength,

                binding.inputRelativeBitOffset,

                binding.inputBitOffset,

                (unsigned int)
                binding.inputBitLength,

                (unsigned int)
                binding.elementBits,

                (unsigned int)
                binding.channelCount);
        }


        // --------------------------------------------------------------------
        // Stage 5A diagnostic.
        //
        // Confirms HMI Runtime XML -> Parser -> C++ Data Model.
        // --------------------------------------------------------------------

        DEBUG_PRINT(
            "[ENI-V2] S%d | "
            "Profile:%s Name:%s MapMode:%s "
            "RxAssign:0x%04X TxAssign:0x%04X "
            "OutSMCtrl:0x%02X InSMCtrl:0x%02X | "
            "PDO Rx:%u Tx:%u | "
            "DC:%s Mode:%s Cycle:%u Shift:%lld Ref:%d | "
            "WD:%s Timeout:%u ms\n",

            count,

            newSlave.runtimeProfile.present
            ? "YES"
            : "NO",

            newSlave.runtimeProfile.name[0] != '\0'
            ? newSlave.runtimeProfile.name
            : "N/A",

            newSlave.runtimeProfile.pdoMappingMode[0] != '\0'
            ? newSlave.runtimeProfile.pdoMappingMode
            : "N/A",

            (unsigned int)
            newSlave.runtimeProfile.rxPdoAssignIndex,

            (unsigned int)
            newSlave.runtimeProfile.txPdoAssignIndex,

            (unsigned int)
            newSlave.runtimeProfile.outputSmControl,

            (unsigned int)
            newSlave.runtimeProfile.inputSmControl,

            (unsigned int)
            newSlave.runtimeProfile.rxPdoIndexes.size(),

            (unsigned int)
            newSlave.runtimeProfile.txPdoIndexes.size(),

            newSlave.runtimeDc.present
            ? "YES"
            : "NO",

            newSlave.runtimeDc.mode[0] != '\0'
            ? newSlave.runtimeDc.mode
            : "N/A",

            (unsigned int)
            newSlave.runtimeDc.cycleTimeNs,

            (long long)
            newSlave.runtimeDc.shiftTimeNs,

            newSlave.runtimeDc.referenceClock
            ? 1
            : 0,

            newSlave.runtimeWatchdog.present
            ? "YES"
            : "NO",

            (unsigned int)
            newSlave.runtimeWatchdog.processDataTimeoutMs);


        // ====================================================================
        // 4. Vendor / Product Compatibility Hooks
        //
        // 目前 Runtime XML 已提供真實 PhysAddr / BitSize，
        // 因此這裡不再修改 Mapping。
        //
        // 未來如果某裝置需要特殊 Runtime Compatibility，
        // 可在此加入明確的 Vendor + Product 規則。
        // ====================================================================

        switch (newSlave.vendorId)
        {
        case 0x000001DD:
            // Delta Electronics

            switch (newSlave.productCode)
            {
            case 0x00005500:
                // R1-EC5500
                break;

            case 0x00006002:
                // R1-EC6002
                break;

            case 0x00007062:
                // R1-EC7062
                break;

            case 0x00008124:
                // R1-EC8124
                break;

            case 0x00006010:
                // ASDA-A3-E
                break;

            default:
                break;
            }

            break;


        default:
            break;
        }


        // ====================================================================
        // 5. Add Slave
        // ====================================================================

        m_slaves.push_back(
            newSlave);


        DEBUG_PRINT(
            "[ENI] S%d | %s | Type:%s | Vendor:0x%08X Product:0x%08X "
            "Revision:0x%08X Cfg:0x%04X | "
            "Out:%u bits @0x%04X | In:%u bits @0x%04X\n",
            count,
            newSlave.name,
            newSlave.type,
            newSlave.vendorId,
            newSlave.productCode,
            newSlave.revision,
            newSlave.configuredAddress,
            newSlave.outputBitLength,
            newSlave.configAddrOut,
            newSlave.inputBitLength,
            newSlave.configAddrIn);


        ++count;


        free(
            slaveContent);


        pCurrent =
            pSlaveEnd +
            strlen(
                "</Slave>");
    }


    free(
        xmlBuffer);


    if (count <= 0)
    {
        DEBUG_PRINT(
            "[ENI] ERROR: No <Slave> nodes were parsed.\n");

        return 0;
    }


    DEBUG_PRINT(
        "[ENI] Runtime Config loaded successfully. SlaveCount:%d\n",
        count);


    return
        count;
}
