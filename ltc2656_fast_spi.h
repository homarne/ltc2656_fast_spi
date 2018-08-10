#ifndef ltc2656_fast_spix
#define ltc2656_fast_spix


#include "Arduino.h"

/*
 http://www.analog.com/media/en/technical-documentation/data-sheets/2656fa.pdf
 LTC2656 Commands
 0 0 0 0 Write to Input Register n
 0 0 0 1 Update (Power Up) DAC Register n
 0 0 1 0 Write to Input Register n, Update (Power Up) All 
 0 0 1 1 Write to and Update (Power Up) n
 0 1 0 0 Power Down n
 0 1 0 1 Power Down Chip (All DACs and Reference)
 0 1 1 0 Select Internal Reference (Power-Up Reference)
 0 1 1 1 Select External Reference (Power-Down)#define WRITE_INPUT_REG_N 0  //0 0 0 0 Write to Input Register n
 0 0 0 1 Update (Power Up) DAC Register n
 1 1 1 1 No Operation
*/
#define WRITE_INPUT_REG_N 0
#define UPDATE_DAC_REGISTER_N 2
#define WRITE_INPUT_REG_N_UPDATE_ALL 2
#define WRITE_INPUT_REG_N_UPDATE_N 3
#define POWER_DOWN_N 4
#define POWER_DOWN_ALL 5
#define SELECT_INT_REF 6
#define SELECT_EXT_REF 7
#define NO_OP 15

//DAC Channel Addressing
#define DAC_A 0
#define DAC_B 1
#define DAC_C 2
#define DAC_D 3
#define DAC_E 4
#define DAC_F 5
#define DAC_G 6
#define DAC_H 7
#define DAC_ALL 15

// define explicitly to eliminate ORing of command and channel data
// command is top nibble, address is bottom nibble
#define WRITE_AND_UPDATE_A 0x30
#define WRITE_AND_UPDATE_B 0x31
#define WRITE_AND_UPDATE_C 0x32
#define WRITE_AND_UPDATE_D 0x33
#define WRITE_AND_UPDATE_E 0x34
#define WRITE_AND_UPDATE_F 0x35
#define WRITE_AND_UPDATE_G 0x36
#define WRITE_AND_UPDATE_H 0x37

// To do: Figure out how to include Arduino device, instance, and component .h files here
// so that corresnding source code does not have to be copied below 

// from ArduinoCore-sam3-master/system/CMSIS/Device/ATMEL/sam3n/include/sam3n00a.h
// typedef volatile       uint32_t WoReg; /**< Write only 32-bit register (volatile unsigned int) */
// typedef volatile       uint32_t RwReg;
// typedef volatile       uint32_t RoReg;

// derived from ArduinoCore-sam3-master/system/CMSIS/Device/ATMEL/sam3u/include/component/component_spi.h
struct SPI_PORTS {
  WoReg SPI_CR;        /**< \brief (Spi Offset: 0x00) Control Register */
  RwReg SPI_MR;        /**< \brief (Spi Offset: 0x04) Mode Register */
  RoReg SPI_RDR;       /**< \brief (Spi Offset: 0x08) Receive Data Register */
  WoReg SPI_TDR;       /**< \brief (Spi Offset: 0x0C) Transmit Data Register */
  RoReg SPI_SR;        /**< \brief (Spi Offset: 0x10) Status Register */
  WoReg SPI_IER;       /**< \brief (Spi Offset: 0x14) Interrupt Enable Register */
  WoReg SPI_IDR;       /**< \brief (Spi Offset: 0x18) Interrupt Disable Register */
  RoReg SPI_IMR;       /**< \brief (Spi Offset: 0x1C) Interrupt Mask Register */
  RoReg Reserved1[4];
  RwReg SPI_CSR[4];    /**< \brief (Spi Offset: 0x30) Chip Select Register */
  RoReg Reserved2[41];
  RwReg SPI_WPMR;      /**< \brief (Spi Offset: 0xE4) Write Protection Control Register */
  RoReg SPI_WPSR;      /**< \brief (Spi Offset: 0xE8) Write Protection Status Register */
};

// 0x40008000 is base adress of SPI registers
// marking the reference as volitile tells the compiler that the value of the data structure 
// may change outsde of its control, and to not load the value into registers
volatile struct SPI_PORTS * const spi_reg = (struct SPI_PORTS *)0x40008000U;

// note: a register to struct mapping example with detailed discussion can be found here:
/// http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.faqs/ka3750.html

// from ArduinoCore-sam3-master/system/CMSIS/Device/ATMEL/sam3n/include/component/component_spi.h
// define SAM3 SPI status register masks
// #define SPI_SR_TDRE (0x1u << 1) /**< \brief (SPI_SR) Transmit Data Register Empty */
// #define SPI_SR_RDRF (0x1u << 0) /**< \brief (SPI_SR) Receive Data Register Full */
// #define SPI_SR_TXEMPTY (0x1u << 9) /**< \brief (SPI_SR) Transmission Registers Empty */

#define SPI_CHIP_SELECT_3_MASK 0x00070000 // selects SPI chip select 3 when or'ed with transmit data

// direct port manipulation
// using Arduino Due pin 31 for chip select
// per grey nomad Due pinout diagram, pin 31 is port A.7
// will have to change this is different pin is used for chip select
// to do: make this programable (but without the overhead in the native libraries)
#define DAC_SELECT_PIN 31
#define DAC_CS_MASK (1<<7)
//const int dacSelectPin = 31;

// note: an disussion of  fast pin manipulation on Due can be found here:
// http://forum.arduino.cc/index.php?topic=260731.0
// a detailed tutotial on port manipulation (Arduino, but not Due) can be found here:
// http://tronixstuff.com/2011/10/22/tutorial-arduino-port-manipulation/

// not guaranteed to be portable across architectures, but....
// allows speedy access to low and high bytes of word
// dac_byte[0] is low order byte
// dac_byte[1] is high order byte
typedef union 
   {  uint16_t  dac_word;
      uint8_t   dac_byte[2];
   } dac_data;

class LTC2656FastSPI {
  public:
    LTC2656FastSPI();
    void Begin();
    void WriteDac(uint8_t cmd_addr, uint8_t * data);
    void WriteDac8(dac_data * data);
};

#endif


