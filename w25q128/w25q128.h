#pragma once
#include "spi_concept.h"
#include "w25q128_params.h"
#include "wire_concept.h"
#include <stdint.h>
#include <array>
namespace bmy {
template <spi_com SPI_COM, wire_handler WIRE> class W25Q128FlashMemTest;

/**
 * @brief Driver wrapper for the W25Q128 SPI flash device.
 *
 * This is a small, hardware-abstracted driver implemented as a template so it
 * can work with different SPI and wire (GPIO) interface implementations.
 *
 * @tparam SPI_COM   Type implementing the SPI communication concept (must
 *                   provide transfer/beginTransaction/endTransaction semantics
 *                   expected by the implementation).
 * @tparam WIRE      Type implementing simple wire operations (digitalWrite,
 *                   pinMode, delay, ...). Used for chip-select and control
 *                   signalling.
 *
 * Usage example:
 * @code{.cpp}
 * bmy::W25Q128FlashMem<SPIWrapper, WireImpl> flash(&SPI0, &WireImpl::instance());
 * flash.init(CHIP_SELECT_PIN);
 * uint8_t id = flash.read_device_id();
 * @endcode
 *
 * The class exposes common operations: read, write (multi-page), erase,
 * device control (suspend/resume/power down), and lower-level helpers used by
 * the public API. The implementation expects a 24-bit address space and
 * provides methods to operate on pages and sectors.
 */
template <spi_com SPI_COM, wire_handler WIRE> class W25Q128FlashMem {
  friend W25Q128FlashMemTest<SPI_COM, WIRE>;

public:
  using SPI_TYPE = SPI_COM;
  using WIRE_TYPE = WIRE;
  W25Q128FlashMem(SPI_COM *spi, WIRE *wire) : spi_(spi), wire_(wire) {}
  W25Q128FlashMem(const W25Q128FlashMem &) = delete;
  W25Q128FlashMem(W25Q128FlashMem &&) = delete;
  ~W25Q128FlashMem();
  W25Q128FlashMem &operator=(const W25Q128FlashMem &) = delete;
  W25Q128FlashMem &operator=(W25Q128FlashMem &&) = delete;
  /**
   * @brief Initialize the driver and configure the chip-select pin.
   *
   * This method must be called before any other operation. It sets up the
   * provided chip select pin and performs a device reset sequence.
   *
   * @param chip_select_pin GPIO pin number used for chip select.
   */
  void init(uint8_t chip_select_pin, uint32_t clock_speed);

  /**
   * @brief Read the manufacturer/device ID using the Device ID instruction.
   * @return 8-bit device identifier.
   */
  uint8_t read_device_id() const;

  /**
   * @brief Read `size` bytes starting from `addr` into `buffer` using the
   *        standard (slow) read command.
   *
   * @param addr 24-bit address to read from.
   * @param buffer Destination buffer, must be at least `size` bytes.
   * @param size Number of bytes to read.
   */
  void read(uint32_t addr, uint8_t *buffer, uint32_t size) const;

  /**
   * @brief Write `size` bytes starting at `addr`.
   *
   * This uses the internal multi-page programming helper so the write can span
   * multiple flash pages as needed.
   *
   * @param addr 24-bit address to write to.
   * @param buffer Source buffer containing data to program.
   * @param size Number of bytes to write.
   */
  void write(uint32_t addr, uint8_t *buffer, uint32_t size) {
    multi_pages_program(addr, buffer, size);
  }

  /**
   * @brief Erase a region starting at `addr` covering `size` bytes.
   * @return true if the erase command was issued successfully.
   */
  bool erase(uint32_t addr, uint32_t size) const;

  /**
   * @brief Issue a full chip erase (may take significant time).
   */
  void chip_erase() const;

  /**
   * @brief Query the device BUSY status bit.
   * @return true if the device is busy performing an internal operation.
   */
  bool is_busy() const {
    return (read_status_register(w25q128::StatusRegister::kRegister1) & 0x01) == 1;
  }

  /**
   * @brief Suspend an ongoing program/erase operation (if supported).
   */
  void suspend() const;

  /**
   * @brief Resume a previously suspended program/erase operation.
   */
  void resume() const;

  /**
   * @brief Put the device into low-power (power-down) mode.
   */
  void power_down() const;

  /**
   * @brief Enter QPI (4-wire) mode if the device supports it.
   */
  void qpi_mode() const;

  /**
   * @brief Reset the device (use enable_reset() + reset sequence).
   */
  void reset_device() const;

  /**
   * @brief Close the driver and release hardware resources (put device
   *        in a safe/idle state).
   */
  void close();

private:
  /**
   * @brief Read the JEDEC ID (3 bytes: manufacturer, memory type, capacity).
   * @return array of 3 bytes containing JEDEC ID.
   */
  std::array<uint8_t, 3> read_jedec_id() const;

  /**
   * @brief Fast read variant that uses a dummy cycle for higher speed.
   */
  void fast_read(uint32_t addr, uint8_t *buffer, uint32_t size) const;

  /**
   * @brief Program up to a single page starting at `addr`.
   * @return number of bytes actually written in the page.
   */
  uint16_t page_program(uint32_t addr, uint8_t *buffer, uint16_t size);

  /**
   * @brief Write data spanning multiple pages by invoking `page_program` as
   * needed.
   */
  void multi_pages_program(uint32_t addr, uint8_t *buffer, uint32_t size);

  /**
   * @brief Enable write operations on the device (sets WEL in status register).
   */
  void write_enable() const;

  /**
   * @brief Query whether write-enable latch is set.
   * @return true if writes are enabled.
   */
  bool is_write_enable() const;

  /**
   * @brief Enable volatile write to status register (if supported).
   */
  void volatile_sr_write_enable() const;

  /**
   * @brief Disable write operations on the device (clears WEL).
   */
  void write_disable() const;

  /**
   * @brief Read one of the device status registers.
   * @param reg Status register selector.
   * @return 8-bit register value.
   */
  uint8_t read_status_register(w25q128::StatusRegister reg) const;

  /**
   * @brief Write to a status register.
   */
  void write_status_register(uint8_t data, w25q128::StatusRegister reg) const;

  /**
   * @brief Read a SFDP register byte.
   */
  uint8_t read_sfdp_register(uint8_t addr) const;

  /**
   * @brief Erase a security register at `addr`.
   */
  void erase_security_register(uint8_t addr) const;

  /**
   * @brief Program a security register page.
   */
  void program_security_register(uint32_t addr, uint8_t *buffer, uint16_t size) const;

  /**
   * @brief Read from a security register.
   */
  void read_security_register(uint32_t addr, uint8_t *buffer, uint16_t size) const;

  /**
   * @brief Global block protection: lock all blocks.
   */
  void global_block_lock() const;

  /**
   * @brief Global block protection: unlock all blocks.
   */
  void global_block_unlock() const;

  /**
   * @brief Read per-block protection status for the block containing `addr`.
   * @return true if the block is locked.
   */
  bool read_block_lock(uint32_t addr) const;

  /**
   * @brief Lock an individual block (address provided as block identifier).
   */
  void individual_block_lock(uint8_t addr) const;

  /**
   * @brief Unlock an individual block.
   */
  void individual_block_unlock(uint8_t addr) const;

  /**
   * @brief Send the enable-reset command required before issuing a device
   * reset.
   */
  void enable_reset() const;

  /**
   * @brief Compute the sector-aligned address for the given sector size.
   */
  uint32_t compute_sector_addr(uint32_t addr, w25q128::SectorSize sector) const;

  /**
   * @brief Erase a sector at the computed sector address with the given size.
   */
  void sector_erase(uint32_t addr, w25q128::SectorSize sector) const;

  /**
   * @brief Helper to convert a 24-bit address to MSB-first byte order.
   */
  static uint32_t to_msb(uint32_t addr_lsb);

  uint32_t mem_write_pos_{};
  uint32_t clock_speed_;
  static constexpr uint8_t addr_size_ = 3; // 24-bit address
  uint8_t chip_select_pin_;
  // spi interface
  SPI_COM *spi_;
  // wire interface
  WIRE *wire_;
};
} // namespace bmy
#include "w25q128.tpp"