#pragma once
#include <stdint.h>

//CE Pin & CSN Pin & IRQ Pin
#define CE(x) x ? gpio_set(GPIOB,GPIO0) : gpio_clear(GPIOB,GPIO0);
#define CSN(x) x ? gpio_set(GPIOA,GPIO4) : gpio_clear(GPIOA,GPIO4);
#define IRQ gpio_get(GPIOA,GPIO0)

//SPI(nRF24L01) commands
#define NRF24L01_CMD_REGISTER_R 0x00//Register read
#define NRF24L01_CMD_REGISTER_W 0x20//Register write
#define NRF24L01_CMD_ACTIVATE 0x50//(De)Activates R_RX_PL_WID, W_ACK_PAYLOAD, W_TX_PAYLOAD_NOACK features
#define NRF24L01_CMD_RX_PLOAD_WID_R 0x60//Read RX-payload width for the top R_RX_PAYLOAD in the RX FIFO.
#define NRF24L01_CMD_RX_PLOAD_R 0x61//Read RX payload
#define NRF24L01_CMD_TX_PLOAD_W 0xA0//Write TX payload
#define NRF24L01_CMD_ACK_PAYLOAD_W 0xA8//Write ACK payload
#define NRF24L01_CMD_TX_PAYLOAD_NOACK_W 0xB0 //Write TX payload and disable AUTOACK
#define NRF24L01_CMD_FLUSH_TX 0xE1//Flush TX FIFO
#define NRF24L01_CMD_FLUSH_RX 0xE2//Flush RX FIFO
#define NRF24L01_CMD_REUSE_TX_PL 0xE3//Reuse TX payload
#define NRF24L01_CMD_LOCK_UNLOCK 0x50//Lock/unlock exclusive features
#define NRF24L01_CMD_NOP 0xFF//No operation (used for reading status register)

// SPI(nRF24L01) register address definitions
#define NRF24L01_REG_CONFIG 0x00//Configuration register
#define NRF24L01_REG_EN_AA 0x01//Enable "Auto acknowledgment"
#define NRF24L01_REG_EN_RXADDR 0x02//Enable RX addresses
#define NRF24L01_REG_SETUP_AW 0x03//Setup of address widths
#define NRF24L01_REG_SETUP_RETR 0x04//Setup of automatic re-transmit
#define NRF24L01_REG_RF_CH 0x05//RF channel
#define NRF24L01_REG_RF_SETUP 0x06//RF setup
#define NRF24L01_REG_STATUS 0x07//Status register
#define NRF24L01_REG_OBSERVE_TX 0x08//Transmit observe register
#define NRF24L01_REG_RPD 0x09//Received power detector
#define NRF24L01_REG_RX_ADDR_P0 0x0A//Receive address data pipe 0
#define NRF24L01_REG_RX_ADDR_P1 0x0B//Receive address data pipe 1
#define NRF24L01_REG_RX_ADDR_P2 0x0C//Receive address data pipe 2
#define NRF24L01_REG_RX_ADDR_P3 0x0D//Receive address data pipe 3
#define NRF24L01_REG_RX_ADDR_P4 0x0E//Receive address data pipe 4
#define NRF24L01_REG_RX_ADDR_P5 0x0F//Receive address data pipe 5
#define NRF24L01_REG_TX_ADDR 0x10//Transmit address
#define NRF24L01_REG_RX_PW_P0 0x11//Number of bytes in RX payload in data pipe 0
#define NRF24L01_REG_RX_PW_P1 0x12//Number of bytes in RX payload in data pipe 1
#define NRF24L01_REG_RX_PW_P2 0x13//Number of bytes in RX payload in data pipe 2
#define NRF24L01_REG_RX_PW_P3 0x14//Number of bytes in RX payload in data pipe 3
#define NRF24L01_REG_RX_PW_P4 0x15//Number of bytes in RX payload in data pipe 4
#define NRF24L01_REG_RX_PW_P5 0x16//Number of bytes in RX payload in data pipe 5
#define NRF24L01_REG_FIFO_STATUS 0x17//FIFO status register
#define NRF24L01_REG_DYNPD 0x1C//Enable dynamic payload length
#define NRF24L01_REG_FEATURE 0x1D//Feature register

// Register bits definitions
#define NRF24L01_CONFIG_PRIM_RX 0x01//PRIM_RX bit in CONFIG register
#define NRF24L01_CONFIG_PWR_UP 0x02//PWR_UP bit in CONFIG register
#define NRF24L01_FEATURE_EN_DYN_ACK 0x01//EN_DYN_ACK bit in FEATURE register
#define NRF24L01_FEATURE_EN_ACK_PAY 0x02//EN_ACK_PAY bit in FEATURE register
#define NRF24L01_FEATURE_EN_DPL 0x04//EN_DPL bit in FEATURE register
#define NRF24L01_FLAG_RX_DREADY 0x40//RX_DR bit (data ready RX FIFO interrupt)
#define NRF24L01_FLAG_TX_DSENT 0x20//TX_DS bit (data sent TX FIFO interrupt)
#define NRF24L01_FLAG_MAX_RT 0x10//MAX_RT bit (maximum number of TX re-transmits interrupt)

// Register masks definitions
#define NRF24L01_MASK_REG_MAP 0x1F//Mask bits[4:0] for CMD_RREG and CMD_WREG commands
#define NRF24L01_MASK_CRC 0x0C//Mask for CRC bits [3:2] in CONFIG register
#define NRF24L01_MASK_STATUS_IRQ 0x70//Mask for all IRQ bits in STATUS register
#define NRF24L01_MASK_RF_PWR 0x06//Mask RF_PWR[2:1] bits in RF_SETUP register
#define NRF24L01_MASK_RX_P_NO 0x0E//Mask RX_P_NO[3:1] bits in STATUS register
#define NRF24L01_MASK_DATARATE 0x28//Mask RD_DR_[5,3] bits in RF_SETUP register
#define NRF24L01_MASK_EN_RX 0x3F//Mask ERX_P[5:0] bits in EN_RXADDR register
#define NRF24L01_MASK_RX_PW 0x3F//Mask [5:0] bits in RX_PW_Px register
#define NRF24L01_MASK_RETR_ARD 0xF0//Mask for ARD[7:4] bits in SETUP_RETR register
#define NRF24L01_MASK_RETR_ARC 0x0F//Mask for ARC[3:0] bits in SETUP_RETR register
#define NRF24L01_MASK_RXFIFO 0x03//Mask for RX FIFO status bits [1:0] in FIFO_STATUS register
#define NRF24L01_MASK_TXFIFO 0x30//Mask for TX FIFO status bits [5:4] in FIFO_STATUS register
#define NRF24L01_MASK_PLOS_CNT 0xF0//Mask for PLOS_CNT[7:4] bits in OBSERVE_TX register
#define NRF24L01_MASK_ARC_CNT 0x0F//Mask for ARC_CNT[3:0] bits in OBSERVE_TX register

// Register masks definitions
#define NRF24L01_MASK_REG_MAP 0x1F//Mask bits[4:0] for CMD_RREG and CMD_WREG commands

#define NRF24L01_ADDR_WIDTH 5 //RX/TX address width
#define NRF24L01_PLOAD_WIDTH 32//Payload width
#define NRF24L01_TEST_ADDR    "nRF24"

void nrf24_init(void);
uint8_t nrf24_check(void);
uint8_t nrf24_read_reg(uint8_t reg);
uint8_t nrf24_write_reg(uint8_t reg, uint8_t value);
uint8_t nrf24_read(uint8_t reg, uint8_t* buffer, uint8_t size);
uint8_t nrf24_write(uint8_t reg, uint8_t* buffer, uint8_t size);
uint8_t nrf24_rx_packet(uint8_t* buffer, uint8_t size);
uint8_t nrf24_tx_packet(uint8_t* buffer, uint8_t size);
void nrf24_rx_mode(uint8_t *rx_addr, uint8_t *tx_addr);
void nrf24_tx_mode(uint8_t *rx_addr, uint8_t *tx_addr);
void nrf24_rx_flush(void);
void nrf24_tx_flush(void);
void nrf24_clear_flag(uint8_t reg);
void nrf24_clear_all_flags(void);

