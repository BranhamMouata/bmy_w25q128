#include "spi_concept.h"
#include "w25q128_instructions.h"
namespace bmy {
template <spi_com SPI_COM, wire_handler WIRE>
/**
 * @brief Destructor - ensure writes are disabled when the driver is destroyed.
 */
W25Q128FlashMem<SPI_COM, WIRE>::~W25Q128FlashMem() {
  write_disable();
}
template <spi_com SPI_COM, wire_handler WIRE>
/**
 * @brief Initialize driver hardware bindings and reset the device.
 *
 * Configures the provided chip-select pin as an output, drives it high
 * (device deselected) and performs a reset sequence.
 */
void W25Q128FlashMem<SPI_COM, WIRE>::init(uint8_t chip_select_pin, uint32_t clock_speed) {
  //------- Init the device
  clock_speed_ = clock_speed;
  chip_select_pin_ = chip_select_pin;
  mem_write_pos_ = 0;
  wire_->mode(chip_select_pin_, wire::PinMode::OUTPUT);
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  // reset the device
  reset_device();
}
template <spi_com SPI_COM, wire_handler WIRE>
/**
 * @brief Close the driver and reset the device to a known state.
 */
void W25Q128FlashMem<SPI_COM, WIRE>::close() {
  reset_device();
}
template <spi_com SPI_COM, wire_handler WIRE>
/**
 * @brief Send the Write Enable command to set the WEL bit in the status register.
 */
void W25Q128FlashMem<SPI_COM, WIRE>::write_enable() const {
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  // transfer instruction
  auto instruct = static_cast<uint8_t>(w25q128::Instruction::kWriteEnable);
  spi_->transfer(&instruct, 1);
  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
}
template <spi_com SPI_COM, wire_handler WIRE>
/**
 * @brief Return true if the Write Enable Latch (WEL) is set in status reg1.
 */
bool W25Q128FlashMem<SPI_COM, WIRE>::is_write_enable() const {
  return ((read_status_register(w25q128::StatusRegister::kRegister1) & 0x02) >> 1) == 1;
}
template <spi_com SPI_COM, wire_handler WIRE>
/**
 * @brief Enable volatile write to the status register (device-specific).
 */
void W25Q128FlashMem<SPI_COM, WIRE>::volatile_sr_write_enable() const {
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  // transfer instruction
  auto instruct = static_cast<uint8_t>(w25q128::Instruction::kVolatieSRWriteEnable);
  spi_->transfer(&instruct, 1);
  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
}
template <spi_com SPI_COM, wire_handler WIRE>
/**
 * @brief Send the Write Disable command (clears WEL).
 */
void W25Q128FlashMem<SPI_COM, WIRE>::write_disable() const {
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  // transfer instruction
  auto instruct = static_cast<uint8_t>(w25q128::Instruction::kWriteDisable);
  spi_->transfer(&instruct, 1);
  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
}
template <spi_com SPI_COM, wire_handler WIRE>
/**
 * @brief Read the Device ID (single-byte) using the Device ID instruction.
 * @return Device ID byte returned by the flash.
 */
uint8_t W25Q128FlashMem<SPI_COM, WIRE>::read_device_id() const {
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  //  transfer instruction
  auto instruct = static_cast<uint8_t>(w25q128::Instruction::kDeviceID);
  spi_->transfer(&instruct, 1);
  // transfer 4 dummy bytes (device returns ID after dummy cycles)
  uint32_t dummy{};
  spi_->transfer(&dummy, sizeof(dummy));
  //  retrieve the value
  uint8_t device_id{};
  spi_->transfer(&device_id, 1);
  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
  return device_id;
}
template <spi_com SPI_COM, wire_handler WIRE>
/**
 * @brief Read the 3-byte JEDEC ID (manufacturer, memory type, capacity).
 * @return std::array with 3 ID bytes.
 */
std::array<uint8_t, 3> W25Q128FlashMem<SPI_COM, WIRE>::read_jedec_id() const {
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  // transfer instruction
  auto instruct = static_cast<uint8_t>(w25q128::Instruction::kJEDECID);
  spi_->transfer(&instruct, 1);
  // retrieve the value
  std::array<uint8_t, 3> value{};
  auto *value_ptr = reinterpret_cast<uint8_t *>(&value[0]);
  spi_->transfer(value_ptr, 3);
  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
  return value;
}
template <spi_com SPI_COM, wire_handler WIRE>
/**
 * @brief Convert a 24-bit address (LSB in the low bytes of the value) into
 * a big-endian byte-packed uint32_t suitable for MSB-first transfer.
 */
uint32_t W25Q128FlashMem<SPI_COM, WIRE>::to_msb(uint32_t addr_lsb) {
  auto *addr_ptr = reinterpret_cast<uint8_t *>(&addr_lsb) + addr_size_ - 1;
  uint32_t addr_msb{};
  auto *addr_msb_ptr = reinterpret_cast<uint8_t *>(&addr_msb);
  for (uint8_t idx = 0; idx < addr_size_; idx++) {
    *addr_msb_ptr = *addr_ptr;
    --addr_ptr;
    ++addr_msb_ptr;
  }
  return addr_msb;
}
template <spi_com SPI_COM, wire_handler WIRE>
/**
 * @brief Read `size` bytes starting at `addr` using the standard read command.
 */
void W25Q128FlashMem<SPI_COM, WIRE>::read(uint32_t addr, uint8_t *buffer, uint32_t size) const {
  auto instruct = static_cast<uint8_t>(w25q128::Instruction::kReadData);
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  // transfer the instruction
  spi_->transfer(&instruct, 1);
  // transfer address to read MSB first
  auto addr_msb = to_msb(addr);
  auto *addr_msb_ptr = reinterpret_cast<uint8_t *>(&addr_msb);
  spi_->transfer(addr_msb_ptr, addr_size_);
  // read data
  spi_->transfer(buffer, size);
  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
}
template <spi_com SPI_COM, wire_handler WIRE>
/**
 * @brief Fast read variant which issues a dummy cycle and then bursts data.
 */
void W25Q128FlashMem<SPI_COM, WIRE>::fast_read(uint32_t addr, uint8_t *buffer,
                                               uint32_t size) const {
  auto instruct = static_cast<uint8_t>(w25q128::Instruction::kFastReadData);
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  // transfer the instruction
  spi_->transfer(&instruct, 1);
  // transfer address to read MSB first
  auto addr_msb = to_msb(addr);
  auto *addr_msb_ptr = reinterpret_cast<uint8_t *>(&addr_msb);
  spi_->transfer(addr_msb_ptr, addr_size_);
  // read data
  spi_->transfer(buffer, size);
  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
}
// NB: The SPI communication is full duplex, that means when writting the buffer to external memory,
// the input buffer will be overwritten with data (dummy 0) received from the device. If it's
// necessary, byte data can be copied and transfert to preserve the input buffer.
template <spi_com SPI_COM, wire_handler WIRE>
uint16_t W25Q128FlashMem<SPI_COM, WIRE>::page_program(uint32_t addr, uint8_t *buffer,
                                                      uint16_t size) {
  // check if address is clear by computing the corresponding sector
  const auto sector_end =
      compute_sector_addr(mem_write_pos_, w25q128::SectorSize::k4KB) + w25q128::kSector4KBSize - 1;
  if (addr <= mem_write_pos_ || addr > sector_end) {
    sector_erase(addr, w25q128::SectorSize::k4KB);
  }
  while (is_busy()) {
    // Wait until the device is free
  }
  // TODO: check that programming the address space is permitted
  //  split the 24-bit address into 3 8-bit address with the MSB first
  // check that write is enabled
  if (!is_write_enable()) {
    write_enable();
  }
  auto instruct = static_cast<uint8_t>(w25q128::Instruction::kPageProgram);
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  // transfert the intruction
  spi_->transfer(&instruct, 1);
  // transfert address to write MSB first
  auto addr_msb = to_msb(addr);
  auto *addr_msb_ptr = reinterpret_cast<uint8_t *>(&addr_msb);
  spi_->transfer(addr_msb_ptr, addr_size_);
  // write data
  // compute the number of bytes to be written to avoid overwritting the page
  const auto addr_lsb = static_cast<uint8_t>(addr & 0xFF);
  const auto max_size = std::min(static_cast<uint16_t>(w25q128::kPageSize - addr_lsb), size);
  spi_->transfer(buffer, max_size);
  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
  // update memory info
  mem_write_pos_ = addr;
  // return the number of bytes written
  return max_size;
}
// NB: The SPI communication is full duplex, that means when writting the buffer to external memory,
// the input buffer will be overwritten with data (dummy 0) received from the device. If it's
// necessary, byte data can be copied and transfert to preserve the input buffer.
template <spi_com SPI_COM, wire_handler WIRE>
void W25Q128FlashMem<SPI_COM, WIRE>::multi_pages_program(uint32_t addr, uint8_t *buffer,
                                                         uint32_t size) {
  uint32_t nbytes_written{};
  while (nbytes_written < size) {
    nbytes_written +=
        page_program(addr + nbytes_written, buffer + nbytes_written, size - nbytes_written);
  }
}
template <spi_com SPI_COM, wire_handler WIRE>
void W25Q128FlashMem<SPI_COM, WIRE>::sector_erase(uint32_t addr, w25q128::SectorSize sector) const {
  // compute the addr corresponding to the sector
  const auto sec_addr = compute_sector_addr(addr, sector);
  while (is_busy()) {
    // Wait until the device is free
  }
  // check that write is enabled
  if (!is_write_enable()) {
    write_enable();
  }
  // TODO : check that erasing the address space is permitted.
  uint8_t instruct;
  switch (sector) {
  case w25q128::SectorSize::k4KB:
    instruct = static_cast<uint8_t>(w25q128::Instruction::kSectorErase4KB);
    break;
  case w25q128::SectorSize::k32KB:
    instruct = static_cast<uint8_t>(w25q128::Instruction::kBlockErase32KB);
    break;
  case w25q128::SectorSize::k64KB:
    instruct = static_cast<uint8_t>(w25q128::Instruction::kBlockErase64KB);
    break;
  }
  // select instruction and send erase command for the computed sector
  static_cast<uint8_t>(w25q128::Instruction::kReadData);
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  // transfert the intruction
  spi_->transfer(&instruct, 1);
  // transfert address to read MSB first
  auto addr_msb = to_msb(addr);
  auto *addr_msb_ptr = reinterpret_cast<uint8_t *>(&addr_msb);
  spi_->transfer(addr_msb_ptr, addr_size_);
  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
}
template <spi_com SPI_COM, wire_handler WIRE>
bool W25Q128FlashMem<SPI_COM, WIRE>::erase(uint32_t addr, uint32_t size) const {
  switch (size) {
  case static_cast<uint32_t>(w25q128::SectorSize::k4KB):
    sector_erase(addr, w25q128::SectorSize::k4KB);
    break;
  case static_cast<uint32_t>(w25q128::SectorSize::k32KB):
    sector_erase(addr, w25q128::SectorSize::k32KB);
    break;
  case static_cast<uint32_t>(w25q128::SectorSize::k64KB):
    sector_erase(addr, w25q128::SectorSize::k64KB);
    break;
  default:
    return false;
  }
  return true;
}
template <spi_com SPI_COM, wire_handler WIRE>
void W25Q128FlashMem<SPI_COM, WIRE>::chip_erase() const {
  while (is_busy()) {
    // Wait until the device is free
  }
  auto instruct = static_cast<uint8_t>(w25q128::Instruction::kChipErase);
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  // transfert the intruction
  spi_->transfer(&instruct, 1);
  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
}
template <spi_com SPI_COM, wire_handler WIRE>
uint8_t W25Q128FlashMem<SPI_COM, WIRE>::read_status_register(w25q128::StatusRegister reg) const {
  // transfer instruction and read a single status byte
  uint8_t instruct;
  switch (reg) {
  case w25q128::StatusRegister::kRegister1:
    instruct = static_cast<uint8_t>(w25q128::Instruction::kReadStatusRegister1);
    break;
  case w25q128::StatusRegister::kRegister2:
    instruct = static_cast<uint8_t>(w25q128::Instruction::kReadStatusRegister2);
    break;
  case w25q128::StatusRegister::kRegister3:
    instruct = static_cast<uint8_t>(w25q128::Instruction::kReadStatusRegister3);
    break;
  }
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  // transfert instruction
  spi_->transfer(&instruct, 1);
  // read status
  uint8_t value{};
  spi_->transfer(&value, 1);
  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
  return value;
}
template <spi_com SPI_COM, wire_handler WIRE>
void W25Q128FlashMem<SPI_COM, WIRE>::write_status_register(uint8_t data,
                                                           w25q128::StatusRegister reg) const {
  while (is_busy()) {
    // Wait until the device is free
  }
  // transfert intruction and data to write
  uint16_t instruct{data};
  switch (reg) {
  case w25q128::StatusRegister::kRegister1:
    instruct |= static_cast<uint16_t>(w25q128::Instruction::kWriteStatusRegister1) << 8;
    break;
  case w25q128::StatusRegister::kRegister2:
    instruct |= static_cast<uint16_t>(w25q128::Instruction::kWriteStatusRegister2) << 8;
    break;
  case w25q128::StatusRegister::kRegister3:
    instruct |= static_cast<uint16_t>(w25q128::Instruction::kWriteStatusRegister3) << 8;
    break;
  }
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  spi_->transfer(&instruct, 2);
  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
}
template <spi_com SPI_COM, wire_handler WIRE>
uint8_t W25Q128FlashMem<SPI_COM, WIRE>::read_sfdp_register(uint8_t addr) const {
  auto instruct = static_cast<uint8_t>(w25q128::Instruction::kReadSFDPRegister);
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  // transfert the intruction
  spi_->transfer(&instruct, 1);
  // transfert address to read MSB first
  auto addr_msb = to_msb(addr);
  auto *addr_msb_ptr = reinterpret_cast<uint8_t *>(&addr_msb);
  spi_->transfer(addr_msb_ptr, addr_size_);
  // transfer a dummy byte, then read the register value
  uint8_t value{};
  spi_->transfer(&value, 1);
  // read register
  spi_->transfer(&value, 1);
  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
  return value;
}
/**
 * @brief Erase a security register page starting at `addr`.
 */
template <spi_com SPI_COM, wire_handler WIRE>
void W25Q128FlashMem<SPI_COM, WIRE>::erase_security_register(uint8_t addr) const {
  while (is_busy()) {
    // Wait until the device is free
  }
  auto instruct = static_cast<uint8_t>(w25q128::Instruction::kEraseSecurityRegister);
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  // transfert the intruction
  spi_->transfer(&instruct, 1);
  // transfert address to read MSB first
  auto addr_msb = to_msb(addr);
  auto *addr_msb_ptr = reinterpret_cast<uint8_t *>(&addr_msb);
  spi_->transfer(addr_msb_ptr, addr_size_);
  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
}
/**
 * @brief Program a security register page.
 */
template <spi_com SPI_COM, wire_handler WIRE>
void W25Q128FlashMem<SPI_COM, WIRE>::program_security_register(uint32_t addr, uint8_t *buffer,
                                                               uint16_t size) const {
  while (is_busy()) {
    // Wait until the device is free
  }
  // split the 24-bit address into 3 8-bit address with the MSB first
  auto instruct = static_cast<uint8_t>(w25q128::Instruction::kProgramSecurityRegister);
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  // transfert the intruction
  spi_->transfer(&instruct, 1);
  // transfert address to read MSB first
  auto addr_msb = to_msb(addr);
  auto *addr_msb_ptr = reinterpret_cast<uint8_t *>(&addr_msb);
  spi_->transfer(addr_msb_ptr, addr_size_);
  // write data
  // compute the number of bytes to be written to avoid overwritting the page
  const auto max_size = std::min(static_cast<uint16_t>(w25q128::kPageSize - (addr & 0xFF)), size);
  spi_->transfer(*buffer, max_size);
  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
}
/**
 * @brief Read from a security register page into `buffer`.
 */
template <spi_com SPI_COM, wire_handler WIRE>
void W25Q128FlashMem<SPI_COM, WIRE>::read_security_register(uint32_t addr, uint8_t *buffer,
                                                            uint16_t size) const {
  auto instruct = static_cast<uint8_t>(w25q128::Instruction::kReadSecurityRegister);
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  // transfert the intruction
  spi_->transfer(&instruct, 1);
  // transfert address to read MSB first
  auto addr_msb = to_msb(addr);
  auto *addr_msb_ptr = reinterpret_cast<uint8_t *>(&addr_msb);
  spi_->transfer(addr_msb_ptr, addr_size_);
  // transfert a dummy
  uint8_t dummy{};
  spi_->transfer(&dummy, 1);
  // read data
  // compute the number of bytes to be written to avoid overwritting the page
  const auto max_size = std::min(static_cast<uint16_t>(w25q128::kPageSize - (addr & 0xFF)), size);
  spi_->transfer(buffer, max_size);

  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
}
/**
 * @brief Lock all blocks globally (write-protect entire device).
 */
template <spi_com SPI_COM, wire_handler WIRE>
void W25Q128FlashMem<SPI_COM, WIRE>::global_block_lock() const {
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  // transfert intruction
  auto instruct = static_cast<uint8_t>(w25q128::Instruction::kGlobalBlockLock);
  spi_->transfer(&instruct, 1);
  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
}
/**
 * @brief Unlock all blocks globally.
 */
template <spi_com SPI_COM, wire_handler WIRE>
void W25Q128FlashMem<SPI_COM, WIRE>::global_block_unlock() const {
  // check that write is enabled
  if (!is_write_enable()) {
    write_enable();
  }
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  // transfert intruction
  uint8_t instruct = static_cast<uint8_t>(w25q128::Instruction::kGlobalBlockUnLock);
  spi_->transfer(&instruct, 1);
  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
}
/**
 * @brief Read protection status for the block at `addr`.
 * @return true if the block is locked.
 */
template <spi_com SPI_COM, wire_handler WIRE>
bool W25Q128FlashMem<SPI_COM, WIRE>::read_block_lock(uint32_t addr) const {
  auto instruct = static_cast<uint8_t>(w25q128::Instruction::kReadBlockLock);
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  // transfert the intruction
  spi_->transfer(&instruct, 1);
  // transfert address to write MSB first
  auto addr_msb = to_msb(addr);
  auto *addr_msb_ptr = reinterpret_cast<uint8_t *>(&addr_msb);
  spi_->transfer(addr_msb_ptr, addr_size_);
  // read value
  bool value{};
  spi_->transfer(&value, 1);
  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
  return value;
}
/**
 * @brief Lock an individual block (block index provided in `addr`).
 */
template <spi_com SPI_COM, wire_handler WIRE>
void W25Q128FlashMem<SPI_COM, WIRE>::individual_block_lock(uint8_t addr) const {
  auto instruct = static_cast<uint8_t>(w25q128::Instruction::kIndividualBlockLock);
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  // transfert the intruction
  spi_->transfer(&instruct, 1);
  // transfert address to write MSB first
  auto addr_msb = to_msb(addr);
  auto *addr_msb_ptr = reinterpret_cast<uint8_t *>(&addr_msb);
  spi_->transfer(addr_msb_ptr, addr_size_);
  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
}
/**
 * @brief Unlock an individual block (block index provided in `addr`).
 */
template <spi_com SPI_COM, wire_handler WIRE>
void W25Q128FlashMem<SPI_COM, WIRE>::individual_block_unlock(uint8_t addr) const {
  auto instruct = static_cast<uint8_t>(w25q128::Instruction::kIndividualBlockUnLock);
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  // transfert the intruction
  spi_->transfer(&instruct, 1);
  // transfert address to write MSB first
  auto addr_msb = to_msb(addr);
  auto *addr_msb_ptr = reinterpret_cast<uint8_t *>(&addr_msb);
  spi_->transfer(addr_msb_ptr, addr_size_);
  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
}
/**
 * @brief Suspend an ongoing program/erase operation (if supported by device).
 */
template <spi_com SPI_COM, wire_handler WIRE> void W25Q128FlashMem<SPI_COM, WIRE>::suspend() const {
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  // transfer instruction
  auto instruct = static_cast<uint8_t>(w25q128::Instruction::kSuspend);
  spi_->transfer(&instruct, 1);
  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
}
/**
 * @brief Resume a previously suspended program/erase operation.
 */
template <spi_com SPI_COM, wire_handler WIRE> void W25Q128FlashMem<SPI_COM, WIRE>::resume() const {
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  // transfer instruction
  auto instruct = static_cast<uint8_t>(w25q128::Instruction::kResume);
  spi_->transfer(&instruct, 1);
  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
}
/**
 * @brief Put the device into low-power (power-down) mode.
 */
template <spi_com SPI_COM, wire_handler WIRE>
void W25Q128FlashMem<SPI_COM, WIRE>::power_down() const {
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  // transfer instruction
  auto instruct = static_cast<uint8_t>(w25q128::Instruction::kPowerDown);
  spi_->transfer(&instruct, 1);
  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
}
/**
 * @brief Enter QPI (4-wire) mode if the device supports it.
 */
template <spi_com SPI_COM, wire_handler WIRE>
void W25Q128FlashMem<SPI_COM, WIRE>::qpi_mode() const {
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  // transfer instruction
  auto instruct = static_cast<uint8_t>(w25q128::Instruction::kQPImode);
  spi_->transfer(&instruct, 1);
  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
}
/**
 * @brief Send the Enable Reset command required before issuing a reset.
 */
template <spi_com SPI_COM, wire_handler WIRE>
void W25Q128FlashMem<SPI_COM, WIRE>::enable_reset() const {
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  // transfer instruction
  auto instruct = static_cast<uint8_t>(w25q128::Instruction::kEnableReset);
  spi_->transfer(&instruct, 1);
  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
}
/**
 * @brief Compute the base address of the sector that contains `addr`.
 */
template <spi_com SPI_COM, wire_handler WIRE>
uint32_t W25Q128FlashMem<SPI_COM, WIRE>::compute_sector_addr(uint32_t addr,
                                                             w25q128::SectorSize sector) const {
  return addr & (~static_cast<uint32_t>(sector) + 1);
}
/**
 * @brief Perform the device reset sequence (enable reset + reset command).
 */
template <spi_com SPI_COM, wire_handler WIRE>
void W25Q128FlashMem<SPI_COM, WIRE>::reset_device() const {
  // firstly enable reset
  enable_reset();
  spi_->beginTransaction(clock_speed_, spi::BitOrder::MSBFIRST, spi::Mode::SPI_MODE3);
  wire_->write(chip_select_pin_, wire::PinStatus::LOW);
  // transfer instruction
  auto instruct = static_cast<uint8_t>(w25q128::Instruction::kReset);
  spi_->transfer(&instruct, 1);
  // disable the device
  wire_->write(chip_select_pin_, wire::PinStatus::HIGH);
  spi_->endTransaction();
}
} // namespace bmy