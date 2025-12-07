#include "arduino_spi.h"
#include "arduino_wire.h"
#include <Arduino.h>
using namespace bmy;

ArduinoSpiAdapter arduino_spi{};
ArduinoWireAdapter arduino_wire{};
W25Q128FlashMem flash_mem(&arduino_spi, &arduino_wire);
void setup(void) {
  Serial.begin(9600);
  // begin spi
  arduino_spi.begin();
  flash_mem.init(w25q128::kChipSelectPin, w25q128::kClkSpeed);
}

void loop(void) {
  uint8_t id = flash_mem.read_device_id();
  Serial.print("W25Q128 Device ID: 0x");
  Serial.println(id, HEX);
  delay(2000);
}