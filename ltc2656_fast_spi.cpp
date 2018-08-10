#include "Arduino.h"
#include "ltc2656_fast_spi.h"


LTC2656FastSPI::LTC2656FastSPI() {
  }

void LTC2656FastSPI::Begin() {

  // By default the Arduino software configures chip select 3 register to add a delay between SPI transmits
  // The following line overrides the Arduino default (0x0100020A), and removes the 
  // delay before transmission.  This also sets the chip select 3 frequency to 50 MHz.  It is equivelent 
  // (for speed setting) to "SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0))"
  //
  spi_reg->SPI_CSR[3]=0x0000020A;

  //pinMode(dacSelectPin, OUTPUT);
  pinMode(DAC_SELECT_PIN, OUTPUT);
  
}


void LTC2656FastSPI::WriteDac(uint8_t cmd_addr, uint8_t * data) {
  // This function takes a bit les than 1uS from bit select low to bit select high

  // cmd_addr is combined dac command (high nibble)  and address (low nibble)
  // data[0] is low order DAC data
  // data[1] is high order DAC data
  
  // take DAC chip select pin low to select the chip:

  PIOA -> PIO_CODR = DAC_CS_MASK ;  // clear pin

  
  uint32_t spi_data = cmd_addr | SPI_CHIP_SELECT_3_MASK;

  /*
  This code assumes that SPI transmit will be idle when this routine is called - note that 
  this routine ensures that SPI transmit is idle before returning.  
  However, if SPI is also being used for other devices, if it cannot be guaranteed that 
  SPI transmit is idle when this routine is called, then the first and second check for TDR empty 
  should be commented in below so as to prevent collisions.
  */

  // spi transmit is expected to be idle - no need to check if TDR empty on first byte
  //while ((spi_reg->SPI_SR & SPI_SR_TDRE) == 0)
  //  ;
  spi_reg->SPI_TDR = spi_data;
  
  // hide mask ORing in transmit time
  spi_data = data[1] | SPI_CHIP_SELECT_3_MASK;
  
  // TDR is effectively double-buffered - no need to check TDR empty on second byte
  //while ((spi_reg->SPI_SR & SPI_SR_TDRE) == 0)
  //  ;
  spi_reg->SPI_TDR = spi_data;

  spi_data = data[0] | SPI_CHIP_SELECT_3_MASK;
  while ((spi_reg->SPI_SR & SPI_SR_TDRE) == 0)
    ;
  spi_reg->SPI_TDR = spi_data;
  
  // for some reason the SPI_SR_RDRF flag, which is used in SPI.cpp for thsi purpose,
  // goes true before the traansmission is complete; use SPI_SR_TXEMPTY instead
  while ((spi_reg->SPI_SR & SPI_SR_TXEMPTY) == 0)
    ;
    
  // take DAC chip select pin high to de-select the chip:

    PIOA -> PIO_SODR = DAC_CS_MASK ;   // set pin

    // this addtional code demonstates that direct manipulation can create 60nS pulse 
    //PIOA -> PIO_CODR = DAC_CS_MASK ;  // clear pin
    //PIOA -> PIO_SODR = DAC_CS_MASK ;   // set pin
}

void LTC2656FastSPI::WriteDac8(dac_data * data) {
  // writes 8 channels of data to LTC2656
  // 16 uS to update 8 channels
  WriteDac(WRITE_AND_UPDATE_A, data[0].dac_byte);
  WriteDac(WRITE_AND_UPDATE_B, data[1].dac_byte);
  WriteDac(WRITE_AND_UPDATE_C, data[2].dac_byte);
  WriteDac(WRITE_AND_UPDATE_D, data[3].dac_byte);
  WriteDac(WRITE_AND_UPDATE_E, data[4].dac_byte);
  WriteDac(WRITE_AND_UPDATE_F, data[5].dac_byte);
  WriteDac(WRITE_AND_UPDATE_G, data[6].dac_byte);
  WriteDac(WRITE_AND_UPDATE_H, data[7].dac_byte);
}

