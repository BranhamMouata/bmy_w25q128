#include "arduino_iohandler.h"
#include "arduino_spi.h"
#include "w25q128.h"
#include <Arduino.h>
using namespace bmy;

ArduinoSpiAdapter arduino_spi{};
ArduinoIoHandlerAdapter arduino_iohandler{};
W25Q128FlashMem flash_mem(&arduino_spi, &arduino_iohandler);
void setup(void) {
  Serial.begin(9600);
  // begin spi
  arduino_spi.begin();
  flash_mem.init(w25q128::kChipSelectPin, w25q128::kClkSpeed);
}

void loop(void) {
  Serial.println("ID : ");
  Serial.println(flash_mem.read_device_id());
  delay(5);
}