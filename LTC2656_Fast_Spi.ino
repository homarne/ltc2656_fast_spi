/*
 Fast SPI workbench for LTC2656 8-channel DAC

 This code grew out of a project to build an 8-channel LFO for audio synthesis.  
 An 8KHz sample rate will support voice-grade audio channels up to 4KHz (with a high-order filter) 
 or 500Hz LFOs with a simple 2nd order filter.
 
 At an interrupt rate of 8KHz, the avialble time per interupt is ~125 uS
 Using the stock Arduino APIs, toggling a select pin and sending three bytes via SPI to a DAC takes ~5 uS
 For eight DAC channels, thats 40 uS, or nearly 1/3 of the avialble compute time per interrupt
 
 Optimizations implimented here reduce the time to send three bytes to under 1 uS (including chip select).
 For eight DAC channels, that's less than 8 uS, i.e. less than 7% of avialable compute time per interupt.
 In the current implimentation, the time to update 8 channels is 12 uS, likely due to call overhead; 
 this could be further reduced through judicious use of in-lining.

 This schetch demonstated optimize SPI write to generate square wave on DAC A 
 of an attached LTC2656 8-channel DAC.

 The following connections are required:

    LTC2656   SDI -> Arduino Due MISO
    LTC2656  SCLK -> Arduino Due SCK
    LTC2656   CS/ -> Arduino Due pin 31
    LTC2656 LDAC/ -> Arduino Due ground
    LTC2656  CLR/ -> Arduino Due +3.3
    LTC2656   GND -> Arduino Due ground
    LTC2656   VCC -> Arduino Due +3.3
    LTC2656 REFLO -> Arduino Due ground
 
 Thanks to Tom Igoe for the Digital Pot tutorial, 2010.
 Other references are identified inline.

*/

// include the SPI library:
#include <SPI.h>

// To do: Figure out how to include Arduino device, instance, and component .h files here
// so that corresnding source code does not have to be copied below 

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


// from ArduinoCore-sam3-master/system/CMSIS/Device/ATMEL/sam3n/include/sam3n00a.h
typedef volatile       uint32_t WoReg; /**< Write only 32-bit register (volatile unsigned int) */
typedef volatile       uint32_t RwReg;

// from ArduinoCore-sam3-master/system/CMSIS/Device/ATMEL/sam3n/include/instance/instance_spi.h
// Define SAM3 SPI register addreeses
// Not used in this optimized code, but good reference
#define REG_SPI_CR     (*(WoReg*)0x40008000U) /**< \brief (SPI) Control Register */
#define REG_SPI_MR     (*(RwReg*)0x40008004U) /**< \brief (SPI) Mode Register */
#define REG_SPI_RDR    (*(RoReg*)0x40008008U) /**< \brief (SPI) Receive Data Register */
#define REG_SPI_TDR    (*(WoReg*)0x4000800CU) /**< \brief (SPI) Transmit Data Register */
#define REG_SPI_SR     (*(RoReg*)0x40008010U) /**< \brief (SPI) Status Register */
#define REG_SPI_IER    (*(WoReg*)0x40008014U) /**< \brief (SPI) Interrupt Enable Register */
#define REG_SPI_IDR    (*(WoReg*)0x40008018U) /**< \brief (SPI) Interrupt Disable Register */
#define REG_SPI_IMR    (*(RoReg*)0x4000801CU) /**< \brief (SPI) Interrupt Mask Register */
#define REG_SPI_CSR    (*(RwReg*)0x40008030U) /**< \brief (SPI) Chip Select Register */
#define REG_SPI_WPMR   (*(RwReg*)0x400080E4U) /**< \brief (SPI) Write Protection Control Register */
#define REG_SPI_WPSR   (*(RoReg*)0x400080E8U) /**< \brief (SPI) Write Protection Status Register */
#define REG_SPI_RPR    (*(RwReg*)0x40008100U) /**< \brief (SPI) Receive Pointer Register */
#define REG_SPI_RCR    (*(RwReg*)0x40008104U) /**< \brief (SPI) Receive Counter Register */
#define REG_SPI_TPR    (*(RwReg*)0x40008108U) /**< \brief (SPI) Transmit Pointer Register */
#define REG_SPI_TCR    (*(RwReg*)0x4000810CU) /**< \brief (SPI) Transmit Counter Register */
#define REG_SPI_RNPR   (*(RwReg*)0x40008110U) /**< \brief (SPI) Receive Next Pointer Register */
#define REG_SPI_RNCR   (*(RwReg*)0x40008114U) /**< \brief (SPI) Receive Next Counter Register */
#define REG_SPI_TNPR   (*(RwReg*)0x40008118U) /**< \brief (SPI) Transmit Next Pointer Register */
#define REG_SPI_TNCR   (*(RwReg*)0x4000811CU) /**< \brief (SPI) Transmit Next Counter Register */
#define REG_SPI_PTCR   (*(WoReg*)0x40008120U) /**< \brief (SPI) Transfer Control Register */
#define REG_SPI_PTSR   (*(RoReg*)0x40008124U) /**< \brief (SPI) Transfer Status Register */

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

// from ArduinoCore-sam3-master/system/CMSIS/Device/ATMEL/sam3n/include/component/component_spi.h
// define SAM3 SPI status register masks
#define SPI_SR_TDRE (0x1u << 1) /**< \brief (SPI_SR) Transmit Data Register Empty */
#define SPI_SR_RDRF (0x1u << 0) /**< \brief (SPI_SR) Receive Data Register Full */
#define SPI_SR_TXEMPTY (0x1u << 9) /**< \brief (SPI_SR) Transmission Registers Empty */

#define SPI_CHIP_SELECT_3_MASK 0x00070000 // selects SPI chip select 3 when or'ed with transmit data

// note: a simpler register to struct mapping example can be found here:
/// http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.faqs/ka3750.html

// 0x40008000 is base adress of SPI registers
volatile struct SPI_PORTS * const spi_reg = (struct SPI_PORTS *)0x40008000U;

// data buffer for one channel of LTC2656 data
uint8_t dac_data_1ch[3];

//uint8_t *buffer;
uint32_t spi_data;

// an example of fast pin manipulation on Due can be found here:
// http://forum.arduino.cc/index.php?topic=260731.0

// set pin 10 as the dac select pin:
const int dacSelectPin = 31;

// using pin 31 for chip select
// per grey nomad Due pinout diagram, pin 31 is port A.7
// for direct port manipulation
#define DAC_CS_MASK (1<<7)

/*
 * Note: The 12bit DACs on the Arduino Due use the lowest 12 bits of the data word
 * In contrast, the 12-bit version of the LTC2656 uses the uppermost 12 bits of the data word, 
 * and simple drop the four LSB of the word.
 * So Full Scale for word is 0X0000FFFF where the least significant four bits are dropped
 */
 
#define    DAC_FS 0x0000FFFF
#define STEP_SIZE 0x00000800  // (DAC_FS+1)/STEP_SIZE = 32 steps

// not guaranteed to be portable across architectures, but....
// allows speedy access to low and high bytes of word
// dac_byte[0] is low order byte
// dac_byte[1] is high order byte
union dac_data
   {  uint16_t  dac_word;
      uint8_t   dac_byte[2];
   };

uint32_t out_value = 0;
dac_data dac_data_8ch[8];

void setup() {
  
  Serial.begin(9600);
  // set the dacSelectPin as an output:
  pinMode(dacSelectPin, OUTPUT);
  // initialize SPI:
  SPI.begin();
  // set SPI data rate to 50 MHz (default is 8 MHz)
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));

  // debug code
  if (0) {
    
    // verify correct SPI register space mapping
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

  // By default the Arduino software configures chip select 3 register to add a delay between SPI transmits
  // The following line overrides the Arduino default (0x0100020A), and removes the 
  // delay before transmission.  This also sets the chip select 3 frequency - see the SAM3 data sheet
  spi_reg->SPI_CSR[3]=0x0000020A;
  
}

             
void loop() {

  // set to 1 to toggle DAC data between zero and DAC_FS
  if (0) {
    if (out_value == 0){
      out_value = DAC_FS;
    } else {
      out_value = 0;
    }
  }

  // set ot 1 to ramp DAC data between zero and DAC_FS
  // STEP_SIZE sets ramp rate - (DAC_FS+1)/STEP_SIZE = number of steps in ramp
  if (1) {
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

  // write to one DAC using Arduino APIs
  if (0) {
    dac_data_1ch[0] = WRITE_AND_UPDATE_A;      
    dac_data_1ch[1] = dac_data_8ch[0].dac_byte[1];                
    dac_data_1ch[2] = dac_data_8ch[0].dac_byte[0];    
    WriteDacData(dac_data_1ch, 3);
  }

  // write to one DAC fast
  if (0) {
    WriteDacDataFast(WRITE_AND_UPDATE_A, dac_data_8ch[0].dac_byte);
  }
  
  //write to all DACs fast
  if (1) {
    WriteDacDataFast_8(dac_data_8ch);
  }
  

}


void WriteDacData(uint8_t * data, int byte_count) {
  // This function takes a bit more than 5 uS from bit select low to bit select high
  
  // Take DAC chip select pin low to select the chip:
  digitalWrite(dacSelectPin, LOW);
  // send bytes to SPI
  SPI.transfer(data, byte_count);
  // Take DAC chip select pin high to unselect the chip:
  digitalWrite(dacSelectPin, HIGH);
}


void WriteDacDataFast(uint8_t cmd_addr, uint8_t * data) {
  // This function takes a bit les than 1uS from bit select low to bit select high

  // cmd_addr is combined dac command (high nibble)  and address (low nibble)
  // data[0] is low order DAC data
  // data[1] is high order DAC data
  
  // take DAC chip select pin low to select the chip:

  PIOA -> PIO_CODR = DAC_CS_MASK ;  // clear pin

  
  spi_data = cmd_addr | SPI_CHIP_SELECT_3_MASK;

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

void WriteDacDataFast_8(dac_data * data) {
  // writes 8 channels of data to LTC2656
  // 16 uS to update 8 channels
  WriteDacDataFast(WRITE_AND_UPDATE_A, data[0].dac_byte);
  WriteDacDataFast(WRITE_AND_UPDATE_B, data[1].dac_byte);
  WriteDacDataFast(WRITE_AND_UPDATE_C, data[2].dac_byte);
  WriteDacDataFast(WRITE_AND_UPDATE_D, data[3].dac_byte);
  WriteDacDataFast(WRITE_AND_UPDATE_E, data[4].dac_byte);
  WriteDacDataFast(WRITE_AND_UPDATE_F, data[5].dac_byte);
  WriteDacDataFast(WRITE_AND_UPDATE_G, data[6].dac_byte);
  WriteDacDataFast(WRITE_AND_UPDATE_H, data[7].dac_byte);
}





