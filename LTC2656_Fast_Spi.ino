/*
 This schetch demonstrates use of the ltc2656 fast spi library to write 8 channels of data to an 
 an attached LTC2656 8-channel DAC.  This example requires an Arduino Due and an LTC2656 DAC.

 The following connections are expected:

    LTC2656   SDI -> Arduino Due MISO
    LTC2656  SCLK -> Arduino Due SCK
    LTC2656   CS/ -> Arduino Due pin 31
    LTC2656 LDAC/ -> Arduino Due ground
    LTC2656  CLR/ -> Arduino Due +3.3
    LTC2656   GND -> Arduino Due ground
    LTC2656   VCC -> Arduino Due +3.3
    LTC2656 REFLO -> Arduino Due ground
 
*/

#include <SPI.h>
#include "ltc2656_fast_spi.h"

/*
 * Note: The 12bit DACs on the Arduino Due use the lowest 12 bits of the data word
 * In contrast, the 12-bit version of the LTC2656 uses the uppermost 12 bits of the data word, 
 * and simple drop the four LSB of the word.
 * So Full Scale for word is 0X0000FFFF where the least significant four bits are dropped
 */
 
#define    DAC_FS 0x0000FFFF
#define STEP_SIZE 0x00000800  // (DAC_FS+1)/STEP_SIZE = 32 steps

uint32_t out_value = 0;

// data buffer for one channel of LTC2656 data
uint8_t dac_data_1ch[3];
// data buffer for eight channels of LTC2656 data
dac_data dac_data_8ch[8];

LTC2656FastSPI FASTDAC;

void setup() {
  
  Serial.begin(9600);
  // set the dacSelectPin as an output:
  
  // initialize SPI:
  SPI.begin();
  // Set SPI data rate to 50 MHz (default is 8 MHz)
  // This is used only using the default Arduino SPI calls as shown in WriteDaceData below
  // Otherwise, use LTC2656FastSPI function FASTDAC.Begin() to configure for 50 MHz 
  // and zero delay between SPI transmits
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  
  // some handy code to debug/explore the SAM3 SPI registers
  // not required for normal use of LTC2656FastSPI library
 
 #define EXPLORE 0
 
  if (EXPLORE) {
    
    // explore SPI register space mapping
    Serial.println(0x40008000U, HEX); // SPI registers start here
    Serial.println();
    Serial.println((long)&spi_reg, HEX); // this is the address the pointer to the struct is located
    Serial.println((long)spi_reg, HEX); // this is the address the pointer to the struct contains
    Serial.println("MR, SR, CSR 0 through 3:"); // check the SPI mode, status, and chip select registers
    Serial.println(spi_reg->SPI_MR, HEX);
    Serial.println(spi_reg->SPI_SR, HEX);
    Serial.println(spi_reg->SPI_CSR[0], HEX);
    Serial.println(spi_reg->SPI_CSR[1], HEX);
    Serial.println(spi_reg->SPI_CSR[2], HEX);
    Serial.println(spi_reg->SPI_CSR[3], HEX); // note that the arduino library only initializes chips slect register 3
    Serial.println();
  
    // a place to dump unneeded SPI read data
    uint32_t reg_read = 0;
  
    // Demonstrate that the SPI can be written to (and read from) directly
    // This verifies that the register mapping, initialization,and transmit data formatting 
    // have all be done correctly. 
    
    // Note that the Arduno SPI code configures chip select register 3 for use
    // The transmit data sent to SPI transmit register (SPI_SR) must have 
    // chip select 3 selected in order for data to be transmitted 
    // This funtion is performed by calling FASTDAC.Begin() in setup() - see below
    spi_reg->SPI_TDR = 0x00000022 | SPI_CHIP_SELECT_3_MASK;
    
    // When SPI status register (SPI_SR) is read immediatly after transmit is initiated,
    // will show transmit and receive status as busy
    reg_read = spi_reg->SPI_SR;
    Serial.println(reg_read, HEX);
    
    // when SPI_SR is read long after te transmit is complete, it will show
    // transmit and receive status as idle (but with receive data unread)
    delay(10);
    reg_read = spi_reg->SPI_SR;
    Serial.println(reg_read, HEX);
    
    // this loop will initiate repeated SPI transmit with alternating data
    // handy for debugging - to use, set while() to 1
    while(0){
      spi_reg->SPI_TDR = 0x000700ff;
      delay(10);
      spi_reg->SPI_TDR = 0x00070000;
      delay(10);
    }
  }

  // Call to set 50 MHz, no delay between consecutive SPI bytes
  FASTDAC.Begin();
  
}

             
void loop() {

// set one of either SQUARE or RAMP to 1
#define SQUARE 0
#define RAMP 1

  // set to 1 to toggle DAC data between zero and DAC_FS
  if (SQUARE) {
    if (out_value == 0){
      out_value = DAC_FS;
    } else {
      out_value = 0;
    }
  }

  // set to 1 to ramp DAC data between zero and DAC_FS
  // STEP_SIZE sets ramp rate - (DAC_FS+1)/STEP_SIZE = number of steps in ramp
  if (RAMP) {
    out_value = out_value + STEP_SIZE;
    if (out_value >= DAC_FS+1){
      out_value = out_value - (DAC_FS+1);
    } 
  }

  // set DAC A data to out_vallue
  dac_data_8ch[DAC_A].dac_word = out_value;

  // set all DACs to the same value
  dac_data_8ch[DAC_B] = 
  dac_data_8ch[DAC_C] = 
  dac_data_8ch[DAC_D] = 
  dac_data_8ch[DAC_E] = 
  dac_data_8ch[DAC_F] = 
  dac_data_8ch[DAC_G] = 
  dac_data_8ch[DAC_H] = dac_data_8ch[0];

// Set one of either SLOW, FAST, or FAST8 to 1
#define SLOW 0
#define FAST 1
#define FAST8 1

  // write to one DAC using Arduino APIs
  if (SLOW) {
    dac_data_1ch[0] = WRITE_AND_UPDATE_A;      
    dac_data_1ch[1] = dac_data_8ch[0].dac_byte[1];                
    dac_data_1ch[2] = dac_data_8ch[0].dac_byte[0];    
    WriteDacData(dac_data_1ch, 3);
  }

  // write to one DAC fast
  if (FAST) {
    FASTDAC.WriteDac(WRITE_AND_UPDATE_A, dac_data_8ch[0].dac_byte);
  }
  
  //write to all eight DACs fast
  if (FAST8) {
    FASTDAC.WriteDac8(dac_data_8ch);
  }
  

}

void WriteDacData(uint8_t * data, int byte_count) {
  // This function takes a bit more than 5 uS from bit select low to bit select high
  
  // Take DAC chip select pin low to select the chip:
  digitalWrite(DAC_SELECT_PIN, LOW);
  // send bytes to SPI
  SPI.transfer(data, byte_count);
  // Take DAC chip select pin high to unselect the chip:
  digitalWrite(DAC_SELECT_PIN, HIGH);
}






