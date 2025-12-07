#pragma once
#include <SPI.h>
#include <cstdint>

namespace bmy::w25q128 {
// global parameters
inline constexpr uint32_t kClkSpeed = 8000000; // 8MHz
inline constexpr uint8_t kChipSelectPin = 9U;

enum class StatusRegister { kRegister1, kRegister2, kRegister3 };
// Memory map
inline constexpr uint32_t kMemorySize = 16 * 1024 * 1024; // 16MB
inline constexpr uint16_t kPageSize = 256;                // 256B
inline constexpr uint16_t kSector4KBSize = 4 * 1024;      // 4KB
inline constexpr uint16_t kSector32KBSize = 32 * 1024;    // 32KB
inline constexpr uint32_t kSector64KBSize = 64 * 1024;    // 32KB
inline constexpr uint32_t kPageNumber = kMemorySize / kPageSize;
inline constexpr uint32_t kSector4KBNumber = kMemorySize / kSector4KBSize;
inline constexpr uint32_t kBlock32KBNumber = kMemorySize / kSector32KBSize;
inline constexpr uint32_t kBlock64KBNumber = kMemorySize / kSector64KBSize;
enum class SectorSize { k4KB = kSector4KBSize, k32KB = kSector32KBSize, k64KB = kSector64KBSize };
inline constexpr uint32_t kMaxAddr = kMemorySize - 1;
// TODO: Enable test partion only for test builds
//~80% of memory is used for  production data and 20% for test data
inline constexpr uint32_t kProductionkBlock64KB = kBlock64KBNumber * 0.8;
inline constexpr uint32_t kProductionMemorySize = kProductionkBlock64KB * kSector64KBSize;
// 2*64KB sectors will be used for meta data(product ID, software version, user ID, log,...)
inline constexpr uint32_t kMetaDataSize = 2 * kSector64KBSize;
inline constexpr uint32_t kDataMemAddrStart = 0x000000;
inline constexpr uint32_t kDataMemAddrEnd =
    kDataMemAddrStart + (kProductionMemorySize - kMetaDataSize) - 1;
inline constexpr uint32_t kMetaDataMemAddrStart = kDataMemAddrEnd + 1;
inline constexpr uint32_t kMetaDataMemAddrEnd = kMetaDataMemAddrStart + kMetaDataSize - 1;
} // namespace bmy::w25q128

namespace bmy::test::w25q128 {
// 2*64KB sectors will be used for meta data(product ID, software version, user ID, log,...)
inline constexpr uint32_t kTestMemorySize =
    bmy::w25q128::kMemorySize - bmy::w25q128::kProductionMemorySize;
inline constexpr uint32_t kTestMetaDataSize = 2 * bmy::w25q128::kSector64KBSize;
inline constexpr uint32_t kTestDataSize = kTestMemorySize - kTestMetaDataSize;
inline constexpr uint32_t kTestDataMemAddrStart = bmy::w25q128::kProductionMemorySize;
inline constexpr uint32_t kTestDataMemAddrEnd = kTestDataMemAddrStart + kTestDataSize - 1;
inline constexpr uint32_t kTestMetaDataMemAddrStart = kTestDataMemAddrEnd + 1;
inline constexpr uint32_t kTestMetaDataMemAddrEnd =
    kTestMetaDataMemAddrStart + kTestMetaDataSize - 1;
} // namespace bmy::test::w25q128