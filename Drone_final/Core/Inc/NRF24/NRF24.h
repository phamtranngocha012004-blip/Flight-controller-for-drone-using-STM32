/*
 * NRF24.h
 *
 *  Created on: Jan 16, 2026
 *      Author: phamt
 */

#ifndef _NRF24_H
#define _NRF24_H

/**********************************************************************************************
 * INCLUDE
 *********************************************************************************************/

#include <SPI/spi.h>
#include "stm32f103xb.h"

/**********************************************************************************************
 * VARIABLE
 *********************************************************************************************/

/* ---------- nRF24L01 registers/commands ---------- */
#define CONFIG      0x00              // Configuration register
#define EN_AA       0x01              // Enable Auto Acknowledgment
#define EN_RXADDR   0x02              // Enable RX data pipes
#define SETUP_AW    0x03              // Address width setup
#define SETUP_RETR  0x04              // Automatic retransmission setup
#define RF_CH       0x05              // RF channel
#define RF_SETUP    0x06              // RF configuration
#define STATUS      0x07              // Status register
#define OBSERVE_TX  0x08              // Transmit observe register
#define CD          0x09              // Carrier detect
#define RX_ADDR_P0  0x0A              // RX address pipe 0
#define RX_ADDR_P1  0x0B              // RX address pipe 1
#define RX_ADDR_P2  0x0C              // RX address pipe 2
#define RX_ADDR_P3  0x0D              // RX address pipe 3
#define RX_ADDR_P4  0x0E              // RX address pipe 4
#define RX_ADDR_P5  0x0F              // RX address pipe 5
#define TX_ADDR     0x10              // TX address
#define RX_PW_P0    0x11              // RX payload width pipe 0
#define RX_PW_P1    0x12              // RX payload width pipe 1
#define RX_PW_P2    0x13              // RX payload width pipe 2
#define RX_PW_P3    0x14              // RX payload width pipe 3
#define RX_PW_P4    0x15              // RX payload width pipe 4
#define RX_PW_P5    0x16              // RX payload width pipe 5
#define FIFO_STATUS 0x17              // FIFO status register
#define DYNPD	    0x1C              // Dynamic payload length enable
#define FEATURE	    0x1D              // Feature register


#define R_REGISTER    0x00            // Read register command
#define W_REGISTER    0x20            // Write register command
#define REGISTER_MASK 0x1F            // Register address mask
#define ACTIVATE      0x50            // Activate features command
#define R_RX_PL_WID   0x60            // Read RX payload width
#define R_RX_PAYLOAD  0x61            // Read RX payload
#define W_TX_PAYLOAD  0xA0            // Write TX payload
#define W_ACK_PAYLOAD 0xA8            // Write ACK payload
#define FLUSH_TX      0xE1            // Flush TX FIFO
#define FLUSH_RX      0xE2            // Flush RX FIFO
#define REUSE_TX_PL   0xE3            // Reuse last transmitted payload
#define NOP           0xFF            // No operation

/* helper macros */
#define W_REG(x)   (W_REGISTER | ((x) & REGISTER_MASK))   // Macro to write a register
#define R_REG(x)   (R_REGISTER | ((x) & REGISTER_MASK))   // Macro to read a register


/**********************************************************************************************
 * API
 *********************************************************************************************/


/* ---------- CSN / CE control ---------- */

/*
 * @brief Pull CSN pin low to select nRF24
 * @param None
 * @retval None
 */
void CS_Select(void);

/*
 * @brief Pull CSN pin high to unselect nRF24
 * @param None
 * @retval None
 */
void CS_UnSelect(void);

/*
 * @brief Set CE pin high to enable TX/RX
 * @param None
 * @retval None
 */
void CE_Enable(void);

/*
 * @brief Set CE pin low to disable nRF24
 * @param None
 * @retval None
 */
void CE_Disable(void);


/* ---------- SPI register access ---------- */

/*
 * @brief Write one byte to nRF24 register
 * @param Reg Register address
 * @param Data Data to write
 * @retval None
 */
void nrf24_WriteReg(uint8_t Reg, uint8_t Data);

/*
 * @brief Write multiple bytes to nRF24 register
 * @param Reg Register address
 * @param data Pointer to data buffer
 * @param size Number of bytes to write
 * @retval None
 */
void nrf24_WriteRegMulti(uint8_t Reg, uint8_t *data, int size);

/*
 * @brief Read one byte from nRF24 register
 * @param Reg Register address
 * @retval Register value
 */
uint8_t nrf24_ReadReg(uint8_t Reg);

/*
 * @brief Read multiple bytes from nRF24 register
 * @param Reg Register address
 * @param data Pointer to receive buffer
 * @param size Number of bytes to read
 * @retval None
 */
void nrf24_ReadReg_Multi(uint8_t Reg, uint8_t *data, int size);

/*
 * @brief Send a single SPI command to nRF24
 * @param cmd Command byte
 * @retval None
 */
void nrfsendCmd(uint8_t cmd);


/* ---------- GPIO initialization ---------- */

/*
 * @brief Initialize GPIO pins for CSN (PA4) and CE (PB0)
 * @param None
 * @retval None
 */
void ncs_cs_init(void);


/* ---------- LED control (PC13) ---------- */

/*
 * @brief Initialize LED GPIO (PC13)
 * @param None
 * @retval None
 */
void led_init(void);

/*
 * @brief Turn LED on
 * @param None
 * @retval None
 */
void led_on(void);

/*
 * @brief Turn LED off
 * @param None
 * @retval None
 */
void led_off(void);

/*
 * @brief Toggle LED state
 * @param None
 * @retval None
 */
void toggle_led(void);


/* ---------- nRF24 configuration ---------- */

/*
 * @brief Reset nRF24 registers or apply default configuration
 * @param REG Register selector (STATUS, FIFO_STATUS or 0 for full reset)
 * @retval None
 */
void nrf24_reset(uint8_t REG);

/*
 * @brief Initialize SPI, GPIO and nRF24 module
 * @param None
 * @retval None
 */
void NRF24_Init(void);

/*
 * @brief Configure nRF24 to RX mode
 * @param Address Pointer to 5-byte RX address
 * @param channel RF channel number
 * @retval None
 */
void NRF24_RxMode(uint8_t *Address, uint8_t channel);

/*
 * @brief Check if RX data is available
 * @param pipenum Pipe number (reserved, not used internally)
 * @retval 1 if data available, 0 otherwise
 */
uint8_t isDataAvailable(int pipenum);

/*
 * @brief Receive data from RX FIFO
 * @param data Pointer to receive buffer
 * @param size Number of bytes to read
 * @retval None
 */
void NRF24_Receive(uint8_t *data, uint8_t size);

/*
 * @brief Transmit data using nRF24
 * @param data Pointer to transmit buffer
 * @param len Payload length (1–32 bytes)
 * @retval 1 if transmission successful, 0 if failed
 */
uint8_t NRF24_Transmit(uint8_t *data, uint8_t len);



#endif /* SRC_NRF24_NRF24_H_ */
