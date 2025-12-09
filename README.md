# W25Q128 flash memory driver
The BMY W25Q128 driver is written in C++ 20. It's templated to support any SPI and iohandler that implements expected functions. Therefore this driver can be used in your Arduino, STM32, ... based projects.

## About W25Q128
The W25Q128 is a 16MB SPI flash memory that support regular SPI, dual SPI, quad SPI and QPI as well as double data rate. SPI clock frequencies of up to 133MHz are supported allowing equivalent clock rates of 266MHz (133MHz x 2) for Dual I/O and 532MHz (133MHz x 4) for Quad I/O when using the Fast Read Dual/Quad I/O and QPI instructions.

More information about W25Q128 can be found in the datasheet : https://www.winbond.com/resource-files/w25q128jv%20revf%2003272018%20plus.pdf

## Driver operation
The BMY W25Q128 driver uses 4 wires SPI mode (mode 3). When instantiated, SPI and iohandler must be provided in the constructor. Then to initialize writing and reading, chip select pin and SPI clock rate must be provided in init function.

## SPI implementation

The user must provide a SPI implementation to handle SPI communication.
The SPI class must implement :

    void begin() : to initialize SPI.
    void beginTransaction(uint32_t clock_speed, spi::BitOrder bit_order, spi::Mode spi_mode) : to start the transaction.
    void transfer(void *buf, size_t count) : transfer data. Note that SPI communication.
    void endTransaction(): end transaction.

If the user SPI API doesnt provide these implementations, an adapter can be used.
### Iohandler implementation

The user must provide a IOHANDLER implementation to set pin and read from pin.
The IOHANDLER class must implement:

    void mode(uint8_t pin, iohandler::PinMode mode) : to set pin mode.
    void write(uint8_t pin, iohandler::PinStatus status): to do a digital write to the corresponding pin.
    iohandler::PinStatus read(uint8_t pin): read the value of the pin.

If the user IOHANDLER API doesnt provide these implementations, an adapter can be used.
