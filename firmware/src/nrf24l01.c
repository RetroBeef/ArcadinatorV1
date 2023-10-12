#include "nrf24l01.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>


static void nrf24_spi_init(void){
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_AFIO);
  rcc_periph_clock_enable(RCC_SPI1);

  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO5 | GPIO6 | GPIO7 );

  rcc_periph_reset_pulse(RST_SPI1);
  spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_8, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
  spi_enable_software_slave_management(SPI1);
  spi_set_nss_high(SPI1);
  spi_enable(SPI1);
}

void nrf24_init(void){
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  
  //CE CSN Initialize
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO0);
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO4);
 
  //IRQ Initialize
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0);
  gpio_set(GPIOA, GPIO0);

  nrf24_spi_init();
  CSN(1);
}

static uint8_t spi_write_then_read(uint8_t data){
  return spi_xfer(SPI1, data);
}

uint8_t nrf24_read_reg(uint8_t reg){
  uint8_t value;
  CSN(0);
  spi_write_then_read(reg);
  value = spi_write_then_read(NRF24L01_CMD_NOP);
  CSN(1);
  return value;
}

uint8_t nrf24_write_reg(uint8_t reg, uint8_t value){
  uint8_t status;
  CSN(0);
  if (reg < NRF24L01_CMD_REGISTER_W) {
    status = spi_write_then_read(NRF24L01_CMD_REGISTER_W | (reg & NRF24L01_MASK_REG_MAP));
    spi_write_then_read(value);

  } else {
    status = spi_write_then_read(reg);
    if ((reg != NRF24L01_CMD_FLUSH_TX) && (reg != NRF24L01_CMD_FLUSH_RX) && (reg != NRF24L01_CMD_REUSE_TX_PL) && (reg != NRF24L01_CMD_NOP)) {
      spi_write_then_read(value);
    }
  }
  CSN(1);
  return status; 
}

uint8_t nrf24_read(uint8_t reg, uint8_t *buffer, uint8_t size){
  CSN(0);
  uint8_t status = spi_write_then_read(reg);
  while (size--) {
    *buffer++ = spi_write_then_read(NRF24L01_CMD_NOP);
  }
  CSN(1);
  return status;
}

uint8_t nrf24_write(uint8_t reg, uint8_t *buffer, uint8_t size){
  CSN(0);
  uint8_t status = spi_write_then_read(reg);
  while (size--) {
    spi_write_then_read(*buffer++);
  }
  CSN(1);
  return status;
}


uint8_t nrf24_check(void){
  uint8_t rxbuf[5];
  uint8_t i;
  uint8_t *ptr = (uint8_t *)NRF24L01_TEST_ADDR;

  nrf24_write(NRF24L01_CMD_REGISTER_W | NRF24L01_REG_TX_ADDR, ptr, 5);
  nrf24_read(NRF24L01_CMD_REGISTER_R | NRF24L01_REG_TX_ADDR, rxbuf, 5);

  for (i = 0; i < 5; i++) {
    if (rxbuf[i] != *ptr++) return 1;
  }
  return 0;
}

void nrf24_rx_flush(void){
  nrf24_write_reg(NRF24L01_CMD_FLUSH_RX, NRF24L01_CMD_NOP);
}

void nrf24_tx_flush(void){
  nrf24_write_reg(NRF24L01_CMD_FLUSH_TX, NRF24L01_CMD_NOP);
}

void nrf24_clear_flag(uint8_t reg) {
  nrf24_write_reg(NRF24L01_CMD_REGISTER_W + NRF24L01_REG_STATUS, reg);
}

void nrf24_clear_all_flags(void) {
  uint8_t reg;
  reg  = nrf24_read_reg(NRF24L01_REG_STATUS);
  reg |= NRF24L01_MASK_STATUS_IRQ;
  nrf24_write_reg(NRF24L01_REG_STATUS, reg);
}

static void nrf24_config(uint8_t *tx_addr){
  //TX Address
  nrf24_write(NRF24L01_CMD_REGISTER_W + NRF24L01_REG_TX_ADDR, tx_addr, NRF24L01_ADDR_WIDTH);
  //RX P0 Payload Width
  nrf24_write_reg(NRF24L01_CMD_REGISTER_W + NRF24L01_REG_RX_PW_P0, NRF24L01_PLOAD_WIDTH);
  //Enable Auto ACK
  //nrf24_write_reg(NRF24L01_CMD_REGISTER_W + NRF24L01_REG_EN_AA, 0x3f);
  //Enable RX channels
  nrf24_write_reg(NRF24L01_CMD_REGISTER_W + NRF24L01_REG_EN_RXADDR, 0x3f);
  //RF channel: 2.400G  + 0.001 * x
  nrf24_write_reg(NRF24L01_CMD_REGISTER_W + NRF24L01_REG_RF_CH, 40);
  //000+0+[0:1Mbps,1:2Mbps]+[00:-18dbm,01:-12dbm,10:-6dbm,11:0dbm]+[0:LNA_OFF,1:LNA_ON]
  //01:1Mbps,-18dbm; 03:1Mbps,-12dbm; 05:1Mbps,-6dbm; 07:1Mbps,0dBm
  //09:2Mbps,-18dbm; 0b:2Mbps,-12dbm; 0d:2Mbps,-6dbm; 0f:2Mbps,0dBm,
  nrf24_write_reg(NRF24L01_CMD_REGISTER_W + NRF24L01_REG_RF_SETUP, 0x0f);
  //0A:delay=250us,count=10, 1A:delay=500us,count=10
  nrf24_write_reg(NRF24L01_CMD_REGISTER_W + NRF24L01_REG_SETUP_RETR, 0x0a);
}

void nrf24_rx_mode(uint8_t *rx_addr, uint8_t *tx_addr){
  CE(0);
  nrf24_config(tx_addr);
  nrf24_write(NRF24L01_CMD_REGISTER_W + NRF24L01_REG_RX_ADDR_P0, rx_addr, NRF24L01_ADDR_WIDTH);
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
  nrf24_write_reg(NRF24L01_CMD_REGISTER_W + NRF24L01_REG_CONFIG, 0x0f);//RX,PWR_UP,CRC16,EN_CRC
  CE(1);
}

void nrf24_tx_mode(uint8_t *rx_addr, uint8_t *tx_addr){
  (void) rx_addr;
  CE(0);
  nrf24_config(tx_addr);
  //On the PTX the **TX_ADDR** must be the same as the **RX_ADDR_P0** and as the pipe address for the designated pipe
  //RX_ADDR_P0 will be used for receiving ACK
  nrf24_write(NRF24L01_CMD_REGISTER_W + NRF24L01_REG_RX_ADDR_P0, tx_addr, NRF24L01_ADDR_WIDTH);
  nrf24_write_reg(NRF24L01_CMD_REGISTER_W + NRF24L01_REG_CONFIG, 0x0e); //TX,PWR_UP,CRC16,EN_CRC
  CE(1);
}

uint8_t nrf24_rx_packet(uint8_t *buffer, uint8_t size){
  uint8_t status, result = 0;
  while(IRQ);
  CE(0);
  status = nrf24_read_reg(NRF24L01_REG_STATUS);

  if(status & NRF24L01_FLAG_RX_DREADY) {
    nrf24_read(NRF24L01_CMD_RX_PLOAD_R, buffer, size);
    result = 1;
    nrf24_clear_flag(NRF24L01_FLAG_RX_DREADY);
  }
  CE(1);
  return result;
}

uint8_t nrf24_tx_packet(uint8_t *buffer, uint8_t size){
  uint8_t status = 0x00;
  CE(0);
  size = size > NRF24L01_PLOAD_WIDTH? NRF24L01_PLOAD_WIDTH : size;
  nrf24_write(NRF24L01_CMD_TX_PLOAD_W, buffer, size);
  CE(1);
  while(IRQ != 0);

  CE(0);
  status = nrf24_read_reg(NRF24L01_REG_STATUS);
  if(status & NRF24L01_FLAG_TX_DSENT) {
    nrf24_clear_flag(NRF24L01_FLAG_TX_DSENT);
  } else if(status & NRF24L01_FLAG_MAX_RT) {
    nrf24_tx_flush();
    nrf24_clear_flag(NRF24L01_FLAG_MAX_RT);
  }
  CE(1);
  return status;
}
