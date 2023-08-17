#include "nrf24l01.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

uint8_t RX_BUF[NRF24L01_PLOAD_WIDTH];
uint8_t TX_BUF[NRF24L01_PLOAD_WIDTH];


static void NRF24L01_SPI_Init(void){
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_AFIO);
  rcc_periph_clock_enable(RCC_SPI1);

  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO5 | GPIO6 | GPIO7 );

  rcc_periph_reset_pulse(RST_SPI1);
  spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_8, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE, SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
  spi_enable_software_slave_management(SPI1);
  spi_set_nss_high(SPI1);
  spi_enable(SPI1);
}

void NRF24L01_Init(void){
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  
  //CE CSN Initialize
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO0);
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO4);
 
  //IRQ Initialize
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0);
  gpio_set(GPIOA, GPIO0);

  NRF24L01_SPI_Init();
  CSN(1);
}

/**
* Basic SPI operation: Write to SPIx and read
*/
static uint8_t SPI_Write_Then_Read(uint8_t data){
  return spi_xfer(SPI1, data);
}

/**
* Read a 1-bit register
*/
uint8_t NRF24L01_Read_Reg(uint8_t reg){
  uint8_t value;
  CSN(0);
  SPI_Write_Then_Read(reg);
  value = SPI_Write_Then_Read(NRF24L01_CMD_NOP);
  CSN(1);
  return value;
}

/**
* Write a 1-byte register
*/
uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value){
  uint8_t status;
  CSN(0);
  if (reg < NRF24L01_CMD_REGISTER_W) {
    // This is a register access
    status = SPI_Write_Then_Read(NRF24L01_CMD_REGISTER_W | (reg & NRF24L01_MASK_REG_MAP));
    SPI_Write_Then_Read(value);

  } else {
    // This is a single byte command or future command/register
    status = SPI_Write_Then_Read(reg);
    if ((reg != NRF24L01_CMD_FLUSH_TX) 
        && (reg != NRF24L01_CMD_FLUSH_RX) 
        && (reg != NRF24L01_CMD_REUSE_TX_PL) 
        && (reg != NRF24L01_CMD_NOP)) {
      // Send register value
      SPI_Write_Then_Read(value);
    }
  }
  CSN(1);
  return status; 
}

/**
* Read a multi-byte register
*  reg  - register to read
*  buf  - pointer to the buffer to write
*  len  - number of bytes to read
*/
uint8_t NRF24L01_Read_To_Buf(uint8_t reg, uint8_t *buf, uint8_t len){
  CSN(0);
  uint8_t status = SPI_Write_Then_Read(reg);
  while (len--) {
    *buf++ = SPI_Write_Then_Read(NRF24L01_CMD_NOP);
  }
  CSN(1);
  return status;
}

/**
* Write a multi-byte register
*  reg - register to write
*  buf - pointer to the buffer with data
*  len - number of bytes to write
*/
uint8_t NRF24L01_Write_From_Buf(uint8_t reg, uint8_t *buf, uint8_t len){
  CSN(0);
  uint8_t status = SPI_Write_Then_Read(reg);
  while (len--) {
    SPI_Write_Then_Read(*buf++);
  }
  CSN(1);
  return status;
}


uint8_t NRF24L01_Check(void){
  uint8_t rxbuf[5];
  uint8_t i;
  uint8_t *ptr = (uint8_t *)NRF24L01_TEST_ADDR;

  // Write test TX address and read TX_ADDR register
  NRF24L01_Write_From_Buf(NRF24L01_CMD_REGISTER_W | NRF24L01_REG_TX_ADDR, ptr, 5);
  NRF24L01_Read_To_Buf(NRF24L01_CMD_REGISTER_R | NRF24L01_REG_TX_ADDR, rxbuf, 5);

  // Compare buffers, return error on first mismatch
  for (i = 0; i < 5; i++) {
    if (rxbuf[i] != *ptr++) return 1;
  }
  return 0;
}

/**
* Flush the RX FIFO
*/
void NRF24L01_FlushRX(void){
  NRF24L01_Write_Reg(NRF24L01_CMD_FLUSH_RX, NRF24L01_CMD_NOP);
}

/**
* Flush the TX FIFO
*/
void NRF24L01_FlushTX(void){
  NRF24L01_Write_Reg(NRF24L01_CMD_FLUSH_TX, NRF24L01_CMD_NOP);
}

/**
* Clear IRQ bit of the STATUS register
*   reg - NRF24L01_FLAG_RX_DREADY
*         NRF24L01_FLAG_TX_DSENT
*         NRF24L01_FLAG_MAX_RT
*/
void NRF24L01_ClearIRQFlag(uint8_t reg) {
  NRF24L01_Write_Reg(NRF24L01_CMD_REGISTER_W + NRF24L01_REG_STATUS, reg);
}

/**
* Clear RX_DR, TX_DS and MAX_RT bits of the STATUS register
*/
void NRF24L01_ClearIRQFlags(void) {
  uint8_t reg;
  reg  = NRF24L01_Read_Reg(NRF24L01_REG_STATUS);
  reg |= NRF24L01_MASK_STATUS_IRQ;
  NRF24L01_Write_Reg(NRF24L01_REG_STATUS, reg);
}

/**
* Common configurations of RX and TX, internal function
*/
static void _NRF24L01_Config(uint8_t *tx_addr){
  // TX Address
  NRF24L01_Write_From_Buf(NRF24L01_CMD_REGISTER_W + NRF24L01_REG_TX_ADDR, tx_addr, NRF24L01_ADDR_WIDTH);
  // RX P0 Payload Width
  NRF24L01_Write_Reg(NRF24L01_CMD_REGISTER_W + NRF24L01_REG_RX_PW_P0, NRF24L01_PLOAD_WIDTH);
  // Enable Auto ACK
  NRF24L01_Write_Reg(NRF24L01_CMD_REGISTER_W + NRF24L01_REG_EN_AA, 0x3f);
  // Enable RX channels
  NRF24L01_Write_Reg(NRF24L01_CMD_REGISTER_W + NRF24L01_REG_EN_RXADDR, 0x3f);
  // RF channel: 2.400G  + 0.001 * x
  NRF24L01_Write_Reg(NRF24L01_CMD_REGISTER_W + NRF24L01_REG_RF_CH, 40);
  // 000+0+[0:1Mbps,1:2Mbps]+[00:-18dbm,01:-12dbm,10:-6dbm,11:0dbm]+[0:LNA_OFF,1:LNA_ON]
  // 01:1Mbps,-18dbm; 03:1Mbps,-12dbm; 05:1Mbps,-6dbm; 07:1Mbps,0dBm
  // 09:2Mbps,-18dbm; 0b:2Mbps,-12dbm; 0d:2Mbps,-6dbm; 0f:2Mbps,0dBm, 
  NRF24L01_Write_Reg(NRF24L01_CMD_REGISTER_W + NRF24L01_REG_RF_SETUP, 0x03);
  // 0A:delay=250us,count=10, 1A:delay=500us,count=10
  NRF24L01_Write_Reg(NRF24L01_CMD_REGISTER_W + NRF24L01_REG_SETUP_RETR, 0x0a);
}

/**
* Switch NRF24L01 to RX mode
*/
void NRF24L01_RX_Mode(uint8_t *rx_addr, uint8_t *tx_addr){
  CE(0);
  _NRF24L01_Config(tx_addr);
  // RX Address of P0
  NRF24L01_Write_From_Buf(NRF24L01_CMD_REGISTER_W + NRF24L01_REG_RX_ADDR_P0, rx_addr, NRF24L01_ADDR_WIDTH);
  /**
  REG 0x00: 
  0)PRIM_RX     0:TX             1:RX
  1)PWR_UP      0:OFF            1:ON
  2)CRCO        0:8bit CRC       1:16bit CRC
  3)EN_CRC      Enabled if any of EN_AA is high
  4)MASK_MAX_RT 0:IRQ low        1:NO IRQ
  5)MASK_TX_DS  0:IRQ low        1:NO IRQ
  6)MASK_RX_DR  0:IRQ low        1:NO IRQ
  7)Reserved    0
  */
  NRF24L01_Write_Reg(NRF24L01_CMD_REGISTER_W + NRF24L01_REG_CONFIG, 0x0f); //RX,PWR_UP,CRC16,EN_CRC
  CE(1);
}

/**
* Switch NRF24L01 to TX mode
*/
void NRF24L01_TX_Mode(uint8_t *rx_addr, uint8_t *tx_addr){
  (void) rx_addr;
  CE(0);
  _NRF24L01_Config(tx_addr);
  // On the PTX the **TX_ADDR** must be the same as the **RX_ADDR_P0** and as the pipe address for the designated pipe
  // RX_ADDR_P0 will be used for receiving ACK
  NRF24L01_Write_From_Buf(NRF24L01_CMD_REGISTER_W + NRF24L01_REG_RX_ADDR_P0, tx_addr, NRF24L01_ADDR_WIDTH);
  NRF24L01_Write_Reg(NRF24L01_CMD_REGISTER_W + NRF24L01_REG_CONFIG, 0x0e); //TX,PWR_UP,CRC16,EN_CRC
  CE(1);
}

/**
* Hold till data received and written to rx_buf
*/
uint8_t NRF24L01_RxPacket(uint8_t *rx_buf){
  uint8_t status, result = 0;
  while(IRQ);
  CE(0);
  status = NRF24L01_Read_Reg(NRF24L01_REG_STATUS);

  if(status & NRF24L01_FLAG_RX_DREADY) {
    NRF24L01_Read_To_Buf(NRF24L01_CMD_RX_PLOAD_R, rx_buf, NRF24L01_PLOAD_WIDTH);
    result = 1;
    NRF24L01_ClearIRQFlag(NRF24L01_FLAG_RX_DREADY);
  }
  CE(1);
  return result;
}

/**
* Send data in tx_buf and wait till data is sent or max re-tr reached
*/
uint8_t NRF24L01_TxPacket(uint8_t *tx_buf, uint8_t len){
  uint8_t status = 0x00;
  CE(0);
  len = len > NRF24L01_PLOAD_WIDTH? NRF24L01_PLOAD_WIDTH : len;
  NRF24L01_Write_From_Buf(NRF24L01_CMD_TX_PLOAD_W, tx_buf, len);
  CE(1);
  while(IRQ != 0); // Waiting send finish

  CE(0);
  status = NRF24L01_Read_Reg(NRF24L01_REG_STATUS);
  if(status & NRF24L01_FLAG_TX_DSENT) {
    NRF24L01_ClearIRQFlag(NRF24L01_FLAG_TX_DSENT);
  } else if(status & NRF24L01_FLAG_MAX_RT) {
    NRF24L01_FlushTX();
    NRF24L01_ClearIRQFlag(NRF24L01_FLAG_MAX_RT);
  }
  CE(1);
  return status;
}
