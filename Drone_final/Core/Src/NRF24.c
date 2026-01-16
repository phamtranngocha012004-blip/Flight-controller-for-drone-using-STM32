/*
 * NRF24.c
 *
 *  Created on: Jan 16, 2026
 *      Author: phamt
 */

#include <NRF24/NRF24.h>
#include "stm32f103xb.h"



/* ============================================================
 * nRF24L01 Driver (STM32F103 – Bare Metal)
 * SPI interface + GPIO control
 * ============================================================
 */


/* ---------- nRF24 control pin helpers ---------- */
/* CSN = PA4 (active low), CE = PB0 */

/* Pull CSN pin low to select the nRF24 device */
void CS_Select(void)   { GPIOA->BRR  = GPIO_BRR_BR4; }    // CSN = 0 → select nRF24
void CS_UnSelect(void) { GPIOA->BSRR = GPIO_BSRR_BS4; }   // CSN = 1 → deselect nRF24
void CE_Enable(void)   { GPIOB->BSRR = GPIO_BSRR_BS0; }  // CE = 1 → enable TX/RX
void CE_Disable(void)  { GPIOB->BRR  = GPIO_BRR_BR0; }   // CE = 0 → disable nRF24

/* ---------- nRF24 SPI helpers ---------- */
void nrf24_WriteReg(uint8_t Reg, uint8_t Data)
{
    uint8_t cmd = W_REG(Reg);                            // Write register command
    CS_Select();                                         // Pull CSN LOW
    spi1_transmit(&cmd, 1);                              // Send command
    spi1_transmit(&Data, 1);                             // Send data
    CS_UnSelect();                                       // Pull CSN HIGH
}

void nrf24_WriteRegMulti(uint8_t Reg, uint8_t *data, int size)
{
    uint8_t cmd = W_REG(Reg);                            // Write multi-byte register command
    CS_Select();                                         // Pull CSN LOW
    spi1_transmit(&cmd, 1);                              // Send command
    spi1_transmit(data, size);                           // Send data buffer
    CS_UnSelect();                                       // Pull CSN HIGH
}

uint8_t nrf24_ReadReg(uint8_t Reg)
{
    uint8_t cmd = R_REG(Reg);                            // Read register command
    uint8_t val = 0;                                     // Return value
    CS_Select();                                         // Pull CSN LOW
    spi1_transmit(&cmd, 1);                              // Send command
    spi1_receive(&val, 1);                               // Read data
    CS_UnSelect();                                       // Pull CSN HIGH
    return val;                                          // Return result
}

void nrf24_ReadReg_Multi(uint8_t Reg, uint8_t *data, int size)
{
    uint8_t cmd = R_REG(Reg);                            // Read multi-byte register command
    CS_Select();                                         // Pull CSN LOW
    spi1_transmit(&cmd, 1);                              // Send command
    spi1_receive(data, size);                            // Read data buffer
    CS_UnSelect();                                       // Pull CSN HIGH
}

void nrfsendCmd(uint8_t cmd)
{
    CS_Select();                                         // Pull CSN LOW
    spi1_transmit(&cmd, 1);                              // Send single command
    CS_UnSelect();                                       // Pull CSN HIGH
}

/* ---------- GPIO init for CSN/CE and LED ---------- */
void ncs_cs_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN |                  // Enable GPIOA clock
                    RCC_APB2ENR_IOPBEN;                  // Enable GPIOB clock

    /* PA4: CSN output push-pull (50MHz) */
    GPIOA->CRL &= ~(GPIO_CRL_MODE4_Msk | GPIO_CRL_CNF4_Msk);
    GPIOA->CRL |=  (GPIO_CRL_MODE4_1 | GPIO_CRL_MODE4_0); // Output 50MHz, Push-Pull

    /* PB0: CE output push-pull (50MHz) */
    GPIOB->CRL &= ~(GPIO_CRL_MODE0_Msk | GPIO_CRL_CNF0_Msk);
    GPIOB->CRL |=  (GPIO_CRL_MODE0_1 | GPIO_CRL_MODE0_0); // Output 50MHz, Push-Pull

    /* Default safe states */
    GPIOA->BSRR = GPIO_BSRR_BS4;                          // CSN = HIGH (idle)
    GPIOB->BRR  = GPIO_BRR_BR0;                           // CE = LOW (disable)
}

/* ---------- LED PC13 ---------- */
void led_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;                   // Enable GPIOC clock
    GPIOC->CRH &= ~(GPIO_CRH_MODE13_Msk | GPIO_CRH_CNF13_Msk);
    GPIOC->CRH |=  GPIO_CRH_MODE13_1;                     // Output 2MHz
}

void led_on()
{
	GPIOC->BRR = GPIO_BRR_BR13;                            // PC13 = 0 → LED ON
}

void led_off()
{
	GPIOC->BSRR = GPIO_BSRR_BS13;                          // PC13 = 1 → LED OFF
}

void toggle_led()
{
	GPIOC->ODR ^= GPIO_ODR_ODR13;                          // Toggle LED state
}


/* ---------- reset / default setup ---------- */
void nrf24_reset(uint8_t REG)
{
    if (REG == STATUS)
    {
        nrf24_WriteReg(STATUS, 0x70);                     // Clear all IRQ flags
    }
    else if (REG == FIFO_STATUS)
    {
        nrf24_WriteReg(FIFO_STATUS, 0x11);                // Reset TX/RX FIFO
    }
    else
    {
        nrf24_WriteReg(CONFIG, 0x08);                     // CRC enable, PWR_DOWN
        nrf24_WriteReg(EN_AA, 0x3F);                      // Enable Auto-ACK
        nrf24_WriteReg(EN_RXADDR, 0x03);                  // Enable pipe 0 & 1
        nrf24_WriteReg(SETUP_AW, 0x03);                   // Address width = 5 bytes
        nrf24_WriteReg(SETUP_RETR, 0x03);                 // Auto retransmit setup
        nrf24_WriteReg(RF_CH, 0x02);                      // RF channel setup
        nrf24_WriteReg(RF_SETUP, 0x0E);                   // 2Mbps, 0dBm
        nrf24_WriteReg(STATUS, 0x70);                     // Clear IRQ flags
        nrf24_WriteReg(OBSERVE_TX, 0x00);                 // Reset TX observe register
        nrf24_WriteReg(CD, 0x00);                         // Clear carrier detect

        uint8_t rx0[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
        nrf24_WriteRegMulti(RX_ADDR_P0, rx0, 5);           // Set RX pipe 0 address

        uint8_t rx1[5] = {0xC2,0xC2,0xC2,0xC2,0xC2};
        nrf24_WriteRegMulti(RX_ADDR_P1, rx1, 5);           // Set RX pipe 1 address

        nrf24_WriteReg(RX_ADDR_P2, 0xC3);                  // Set RX pipe 2 LSB
        nrf24_WriteReg(RX_ADDR_P3, 0xC4);
        nrf24_WriteReg(RX_ADDR_P4, 0xC5);
        nrf24_WriteReg(RX_ADDR_P5, 0xC6);

        uint8_t tx0[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
        nrf24_WriteRegMulti(TX_ADDR, tx0, 5);              // Set TX address

        nrf24_WriteReg(RX_PW_P0, 2);                       // Payload width pipe 0 = 2 bytes
        nrf24_WriteReg(RX_PW_P1, 0);
        nrf24_WriteReg(RX_PW_P2, 0);
        nrf24_WriteReg(RX_PW_P3, 0);
        nrf24_WriteReg(RX_PW_P4, 0);
        nrf24_WriteReg(RX_PW_P5, 0);

        nrf24_WriteReg(FIFO_STATUS, 0x11);                 // Reset FIFO
        nrf24_WriteReg(DYNPD, 0);                          // Disable dynamic payload
        nrf24_WriteReg(FEATURE, 0);                        // Disable features
    }
}


/* ---------- init ---------- */
void NRF24_Init(void)
{
    spi_init();                                         // Initialize SPI1
    ncs_cs_init();                                      // Initialize CSN, CE pins

    CE_Disable();                                       // CE LOW → disable radio
    nrf24_reset(0);                                     // Reset all registers

    /* minimal default config */
    nrf24_WriteReg(CONFIG, 0x00);                       // PWR_DOWN, PRIM_RX = 0 (TX mode)
    nrf24_WriteReg(EN_AA, 0x00);                        // Disable Auto ACK
    nrf24_WriteReg(EN_RXADDR, 0x00);                    // Disable RX pipes
    nrf24_WriteReg(SETUP_AW, 0x03);                     // 5-byte address width
    nrf24_WriteReg(SETUP_RETR, 0x00);                   // Disable retransmission
    nrf24_WriteReg(RF_CH, 0x00);                        // RF channel = 0
    nrf24_WriteReg(RF_SETUP, 0x0E);                     // 2Mbps, 0dBm

    /* Keep CE LOW, only enable when entering RX or TX mode */
}

/* ---------- set RX mode ---------- */
void NRF24_RxMode(uint8_t *Address, uint8_t channel)
{
    CE_Disable();                                       // Disable CE before configuration

    nrf24_reset(STATUS);                                // Clear IRQ flags

    nrf24_WriteReg(RF_CH, channel);                     // Set RF channel

    uint8_t en_rxaddr = nrf24_ReadReg(EN_RXADDR);       // Read enabled RX pipes
    en_rxaddr |= (1 << 2);                              // Enable pipe 2
    nrf24_WriteReg(EN_RXADDR, en_rxaddr);               // Write back

    /* Pipe 2 shares the 4 MSB bytes with Pipe 1 */
    nrf24_WriteRegMulti(RX_ADDR_P1, Address, 5);        // Write pipe 1 address
    nrf24_WriteReg(RX_ADDR_P2, 0xEE);                   // Write LSB for pipe 2 address

    nrf24_WriteReg(RX_PW_P2, 1);                        // Payload width pipe 2 = 1 byte

    uint8_t config = nrf24_ReadReg(CONFIG);             // Read CONFIG register
    config |= (1 << 1) | (1 << 0);                      // Set PWR_UP + PRIM_RX bits
    nrf24_WriteReg(CONFIG, config);                     // Write CONFIG register

    CE_Enable();                                        // Enable RX mode
}

/* ---------- check data available ---------- */
uint8_t isDataAvailable(int pipenum)
{
    uint8_t status = nrf24_ReadReg(STATUS);             // Read STATUS register

    if ((status & (1 << 6)))                            // RX_DR bit = 1 → data available
    {
        nrf24_WriteReg(STATUS, (1 << 6));               // Clear RX_DR flag
        return 1;                                       // Data exists
    }

    return 0;                                           // No data
}

/* ---------- receive data ---------- */
void NRF24_Receive(uint8_t *data, uint8_t size)
{
    uint8_t cmdtosend = 0;

    CS_Select();                                        // Pull CSN LOW
    cmdtosend = R_RX_PAYLOAD;                            // Read RX payload command
    spi1_transmit(&cmdtosend, 1);                       // Send command
    spi1_receive(data, size);                           // Read payload data
    CS_UnSelect();                                      // Pull CSN HIGH


    cmdtosend = FLUSH_RX;                               // Flush RX FIFO command
    nrfsendCmd(cmdtosend);                              // Send command
}

/* ---------- transmit data ---------- */
uint8_t NRF24_Transmit(uint8_t *data, uint8_t len)
{
    if (len == 0 || len > 32) return 0;                 // Valid payload is 1–32 bytes

    uint8_t cmd = W_TX_PAYLOAD;                         // Write TX payload command
    CS_Select();                                        // Pull CSN LOW
    spi1_transmit(&cmd, 1);                             // Send command
    spi1_transmit(data, len);                           // Send payload
    CS_UnSelect();                                      // Pull CSN HIGH

    CE_Enable();                                        // Start transmission
    CE_Disable();                                       // End transmission pulse


    uint8_t status = nrf24_ReadReg(STATUS);             // Read status register

    if (status & (1<<5))                                // TX_DS bit → transmission successful
    {
        nrf24_WriteReg(STATUS, (1<<5));                 // Clear TX_DS flag
        return 1;                                       // Success
    }
    else if (status & (1<<4))                           // MAX_RT bit → error (max retransmits)
    {
        nrf24_WriteReg(STATUS, (1<<4));                 // Clear MAX_RT flag
        nrfsendCmd(FLUSH_TX);                           // Flush TX FIFO
        return 0;                                       // Failed
    }

    return 0;                                           // Other status
}




