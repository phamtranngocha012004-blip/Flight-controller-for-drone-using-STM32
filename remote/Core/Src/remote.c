/* main.c - NRF24L01 Basic TX example (STM32F103)
   - SPI PA5=SCK, PA6=MISO, PA7=MOSI
   - CSN = PA4, CE = PB0
   - LED on PC13
   - Assumes HCLK/PCLK2 = 8 MHz -> SPI1 BR = /4 -> SCK = 2 MHz
*/

#include "stm32f103xb.h"
#include "stdint.h"

/* ---------- nRF24L01 registers/commands ---------- */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD	    0x1C
#define FEATURE	    0x1D

#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

/* helper macros */
#define W_REG(x)   (W_REGISTER | ((x) & REGISTER_MASK))
#define R_REG(x)   (R_REGISTER | ((x) & REGISTER_MASK))

/* system tick */
uint32_t mtick = 0;
void SysTick_Init(void)
{
    SysTick->LOAD = 8000 - 1; // 1 ms tick if HCLK = 8 MHz
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;
}
void SysTick_Handler(void) { mtick++; }
void delay(uint32_t ms)
{
    uint32_t s = mtick;
    while ((mtick - s) < ms);
}

/* ---------- SPI (polled) ---------- */
void spi_init(void)
{
    /* enable GPIOA and SPI1 clocks */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_SPI1EN;

    /* PA5 = SCK (AF push-pull)
       PA6 = MISO (input floating)
       PA7 = MOSI (AF push-pull)
       Set MODE = 10 (2MHz) by setting MODEy_1 and clearing MODEy_0
       For AF push-pull: CNFy = 10 (CNFy_1 = 1, CNFy_0 = 0)
    */
    GPIOA->CRL &= ~(GPIO_CRL_MODE5_Msk | GPIO_CRL_CNF5_Msk |
                    GPIO_CRL_MODE6_Msk | GPIO_CRL_CNF6_Msk |
                    GPIO_CRL_MODE7_Msk | GPIO_CRL_CNF7_Msk);

    /* PA5: MODE5 = 10 (2MHz) -> MODE5_1 = 1, MODE5_0 = 0
       CNF5 = 10 -> CNF5_1 = 1, CNF5_0 = 0
       PA6: MODE6 = 00 (input), CNF6 = 01 (floating input) -> CNF6_0 = 1
       PA7: same as PA5
    */
    GPIOA->CRL |= (GPIO_CRL_MODE5_1) | (GPIO_CRL_CNF5_1) |
                  (GPIO_CRL_CNF6_0) |
                  (GPIO_CRL_MODE7_1) | (GPIO_CRL_CNF7_1);

    /* SPI1 config: Master, Software NSS, BR = /4 (BR[2:0] = 001), enable */
    SPI1->CR1 = 0;
    SPI1->CR1 |= SPI_CR1_MSTR;
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR1 |= SPI_CR1_BR_0; // BR = 001 -> fPCLK/4
    SPI1->CR1 |= SPI_CR1_SPE;
}

/* Transmit buffer (polling) */
void spi1_transmit(uint8_t *data, uint32_t size)
{
    uint32_t i = 0;
    volatile uint8_t tmp;
    while (i < size)
    {
        while (!(SPI1->SR & SPI_SR_TXE)); // wait TXE
        SPI1->DR = data[i++];
    }
    while (!(SPI1->SR & SPI_SR_TXE));
    while (SPI1->SR & SPI_SR_BSY); // wait not busy

    /* clear possible OVR by reading DR then SR */
    tmp = SPI1->DR;
    tmp = SPI1->SR;
    (void)tmp;
}

/* Receive size bytes into data (dummy write 0xFF) */
void spi1_receive(uint8_t *data, uint32_t size)
{
    while (size)
    {
        SPI1->DR = 0xFF;
        while (!(SPI1->SR & SPI_SR_RXNE));
        *data++ = (uint8_t)SPI1->DR;
        size--;
    }
}

/* ---------- nRF24 control pin helpers ---------- */
/* CSN = PA4 (active low), CE = PB0 */
void CS_Select(void)   { GPIOA->BRR = GPIO_BRR_BR4; }   // CSN LOW
void CS_UnSelect(void) { GPIOA->BSRR = GPIO_BSRR_BS4; } // CSN HIGH
void CE_Enable(void)   { GPIOB->BSRR = GPIO_BSRR_BS0; } // CE HIGH
void CE_Disable(void)  { GPIOB->BRR  = GPIO_BRR_BR0; }  // CE LOW

/* ---------- nRF24 SPI helpers ---------- */
void nrf24_WriteReg(uint8_t Reg, uint8_t Data)
{
    uint8_t cmd = W_REG(Reg);
    CS_Select();
    spi1_transmit(&cmd, 1);
    spi1_transmit(&Data, 1);
    CS_UnSelect();
}

void nrf24_WriteRegMulti(uint8_t Reg, uint8_t *data, int size)
{
    uint8_t cmd = W_REG(Reg);
    CS_Select();
    spi1_transmit(&cmd, 1);
    spi1_transmit(data, size);
    CS_UnSelect();
}

uint8_t nrf24_ReadReg(uint8_t Reg)
{
    uint8_t cmd = R_REG(Reg);
    uint8_t val = 0;
    CS_Select();
    spi1_transmit(&cmd, 1);
    spi1_receive(&val, 1);
    CS_UnSelect();
    return val;
}

void nrf24_ReadReg_Multi(uint8_t Reg, uint8_t *data, int size)
{
    uint8_t cmd = R_REG(Reg);
    CS_Select();
    spi1_transmit(&cmd, 1);
    spi1_receive(data, size);
    CS_UnSelect();
}

void nrfsendCmd(uint8_t cmd)
{
    CS_Select();
    spi1_transmit(&cmd, 1);
    CS_UnSelect();
}

/* ---------- GPIO init for CSN/CE and LED ---------- */
void ncs_cs_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;

    /* PA4: CSN output push-pull (50MHz) */
    GPIOA->CRL &= ~(GPIO_CRL_MODE4_Msk | GPIO_CRL_CNF4_Msk);
    GPIOA->CRL |= (GPIO_CRL_MODE4_1 | GPIO_CRL_MODE4_0);

    /* PB0: CE output push-pull (50MHz) */
    GPIOB->CRL &= ~(GPIO_CRL_MODE0_Msk | GPIO_CRL_CNF0_Msk);
    GPIOB->CRL |= (GPIO_CRL_MODE0_1 | GPIO_CRL_MODE0_0);

    /* Default safe states */
    GPIOA->BSRR = GPIO_BSRR_BS4; // CSN HIGH (idle)
    GPIOB->BRR  = GPIO_BRR_BR0;  // CE LOW (disabled)
}

void led_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    GPIOC->CRH &= ~(GPIO_CRH_MODE13_Msk | GPIO_CRH_CNF13_Msk);
    GPIOC->CRH |= GPIO_CRH_MODE13_1; // output 2MHz
}
void led_on()  { GPIOC->BRR = GPIO_BRR_BR13; }   // PC13 low -> LED on
void led_off() { GPIOC->BSRR = GPIO_BSRR_BS13; } // PC13 high -> LED off
void toggle_led(){ GPIOC->ODR ^= GPIO_ODR_ODR13; }

/* ---------- reset / default setup ---------- */
void nrf24_reset(uint8_t REG)
{
    if (REG == STATUS)
    {
        /* Write 1 to IRQ bits to clear them */
        nrf24_WriteReg(STATUS, 0x70);
    }
    else if (REG == FIFO_STATUS)
    {
        nrf24_WriteReg(FIFO_STATUS, 0x11);
    }
    else
    {
        nrf24_WriteReg(CONFIG, 0x08);   // default: CRC enabled, PWR_DOWN (no PWR_UP here)
        nrf24_WriteReg(EN_AA, 0x3F);
        nrf24_WriteReg(EN_RXADDR, 0x03);
        nrf24_WriteReg(SETUP_AW, 0x03);
        nrf24_WriteReg(SETUP_RETR, 0x03);
        nrf24_WriteReg(RF_CH, 0x02);
        nrf24_WriteReg(RF_SETUP, 0x0E);
        nrf24_WriteReg(STATUS, 0x70);   // clear IRQs
        nrf24_WriteReg(OBSERVE_TX, 0x00);
        nrf24_WriteReg(CD, 0x00);
        uint8_t rx0[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
        nrf24_WriteRegMulti(RX_ADDR_P0, rx0, 5);
        uint8_t rx1[5] = {0xC2,0xC2,0xC2,0xC2,0xC2};
        nrf24_WriteRegMulti(RX_ADDR_P1, rx1, 5);
        nrf24_WriteReg(RX_ADDR_P2, 0xC3);
        nrf24_WriteReg(RX_ADDR_P3, 0xC4);
        nrf24_WriteReg(RX_ADDR_P4, 0xC5);
        nrf24_WriteReg(RX_ADDR_P5, 0xC6);
        uint8_t tx0[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
        nrf24_WriteRegMulti(TX_ADDR, tx0, 5);

        /* set RX payload width for pipe0 to 2 bytes (PRX must match) */
        nrf24_WriteReg(RX_PW_P0, 2);
        nrf24_WriteReg(RX_PW_P1, 0);
        nrf24_WriteReg(RX_PW_P2, 0);
        nrf24_WriteReg(RX_PW_P3, 0);
        nrf24_WriteReg(RX_PW_P4, 0);
        nrf24_WriteReg(RX_PW_P5, 0);

        nrf24_WriteReg(FIFO_STATUS, 0x11);
        nrf24_WriteReg(DYNPD, 0);
        nrf24_WriteReg(FEATURE, 0);
    }
}

/* ---------- init ---------- */
void NRF24_Init(void)
{
    spi_init();
    ncs_cs_init();

    CE_Disable();
    nrf24_reset(0);

    /* minimal default config: CRC enabled, PWR_DOWN (we will power up in TxMode) */
    nrf24_WriteReg(CONFIG, 0x00);  // PWR_DOWN, PRIM_RX = 0
    nrf24_WriteReg(EN_AA, 0x00);
    nrf24_WriteReg(EN_RXADDR, 0x00);
    nrf24_WriteReg(SETUP_AW, 0x03);
    nrf24_WriteReg(SETUP_RETR, 0x00);
    nrf24_WriteReg(RF_CH, 0x00);
    nrf24_WriteReg(RF_SETUP, 0x0E);

    /* Keep CE LOW here. CE will be set when entering TX mode after PWR_UP delay */
}

/* ---------- set TX mode (power up then CE enable after delay) ---------- */
void NRF24_TxMode(uint8_t *Address, uint8_t channel)
{
    CE_Disable();
    nrf24_WriteReg(RF_CH, channel);
    nrf24_WriteRegMulti(TX_ADDR, Address, 5);

    uint8_t config = nrf24_ReadReg(CONFIG);
    config |= (1<<1);   // PWR_UP = 1
    config &= ~(1<<0);  // PRIM_RX = 0 (TX)
    nrf24_WriteReg(CONFIG, config);

    /* Wait tpd2stby ~1.5 ms */
    delay(2);

    /* CE can be toggled for transmit pulses; keep CE enabled for normal operation */
    CE_Enable();
}

/* ---------- Transmit payload of length len (1..32) ----------
   Returns: 1 = success (TX_DS), 0 = fail (MAX_RT or no ack)
   Note: PRX must have RX_PW_P0 set accordingly (we set to 2 in reset)
*/
uint8_t NRF24_Transmit(uint8_t *data, uint8_t len)
{
    if (len == 0 || len > 32) return 0;

    uint8_t cmd = W_TX_PAYLOAD;
    CS_Select();
    spi1_transmit(&cmd, 1);
    spi1_transmit(data, len);
    CS_UnSelect();

    /* pulse CE to start transmission if needed (short pulse >=10us for PTX)
       Some setups send with CE already high; to be safe, do a short pulse:
    */
    CE_Enable();
    delay(1); // 1 ms - safe (datasheet needs >=10us, 1ms is fine)
    CE_Disable();

    /* small wait then check STATUS */
    delay(2);

    uint8_t status = nrf24_ReadReg(STATUS);

    if (status & (1<<5)) // TX_DS
    {
        nrf24_WriteReg(STATUS, (1<<5)); // clear TX_DS
        return 1;
    }
    else if (status & (1<<4)) // MAX_RT
    {
        nrf24_WriteReg(STATUS, (1<<4)); // clear MAX_RT
        nrfsendCmd(FLUSH_TX);
        return 0;
    }
    /* else unknown / still sending */
    return 0;
}
//==================================ADC====================================

void ADC_Init(void)
{
    /* Enable clock GPIOA & ADC1 */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;   // GPIOA
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;   // ADC1

    /* PA0, PA1, PA2, PA3: Analog input */
    GPIOA->CRL &= ~((0xF) << (0 * 4));   // PA0
    GPIOA->CRL &= ~((0xF) << (1 * 4));   // PA1
    GPIOA->CRL &= ~((0xF) << (2 * 4));   // PA2
    GPIOA->CRL &= ~((0xF) << (3 * 4));   // PA3

    /* ADC configuration */
    ADC1->CR1 = 0;
    ADC1->CR2 = ADC_CR2_ADON;

    /* Sample time cho channel 0,1,2,3 */
    ADC1->SMPR2 |= (ADC_SMPR2_SMP0_1 | ADC_SMPR2_SMP0_2); // CH0
    ADC1->SMPR2 |= (ADC_SMPR2_SMP1_1 | ADC_SMPR2_SMP1_2); // CH1
    ADC1->SMPR2 |= (ADC_SMPR2_SMP2_1 | ADC_SMPR2_SMP2_2); // CH2
    ADC1->SMPR2 |= (ADC_SMPR2_SMP3_1 | ADC_SMPR2_SMP3_2); // CH3

    /* Calibration */
    ADC1->CR2 |= ADC_CR2_RSTCAL;
    while (ADC1->CR2 & ADC_CR2_RSTCAL);

    ADC1->CR2 |= ADC_CR2_CAL;
    while (ADC1->CR2 & ADC_CR2_CAL);
}



uint16_t ADC_Read(uint8_t channel)
{
    ADC1->SQR3 = channel;      // 2 = PA2, 3 = PA3
    ADC1->CR2 |= ADC_CR2_ADON;

    while (!(ADC1->SR & ADC_SR_EOC));
    return ADC1->DR;
}



uint16_t Convert_throttle(uint16_t value_ADC)
{
    const uint16_t center   = 2047;   // điểm bắt đầu
    const uint16_t max_adc  = 4095;   // ADC 12-bit
    const uint16_t deadzone = 100;    // vùng chết quanh center
    int16_t output;

    // Giới hạn giá trị ADC
    if (value_ADC < center)
        value_ADC = center;
    if (value_ADC > max_adc)
        value_ADC = max_adc;

    // Deadzone tại center
    if (value_ADC <= (center + deadzone))
    {
        output = 1000;
    }
    else
    {
        output = 1000
               + ((value_ADC - (center + deadzone)) * 1000)
                 / (max_adc - (center + deadzone));

        if (output > 2000)
            output = 2000;
    }

    // Làm tròn xuống bội số 10
    output = (output / 10) * 10;

    return output;
}

uint8_t Convert_Yaw(uint16_t value_ADC)
{
    const uint16_t adc_min = 0;
    const uint16_t adc_max = 4095;

    const uint16_t out_min = 20;
   // const uint16_t out_mid = 40;
    const uint16_t out_max = 60;

    uint8_t output;

    /* Giới hạn ADC */
    if (value_ADC > adc_max)
        value_ADC = adc_max;

    /* Tuyến tính toàn dải 0 → 4095 */
    output = out_min
           + ((uint32_t)value_ADC * (out_max - out_min))
             / (adc_max - adc_min);

    /* Bảo vệ giới hạn */
    if (output < out_min) output = out_min;
    if (output > out_max) output = out_max;

    return output;
}

uint8_t Convert_Roll_Pitch(uint16_t value_ADC)
{
    const uint16_t adc_min = 0;
    const uint16_t adc_max = 4095;

    const uint16_t out_min = 20;
    const uint16_t out_max = 60;

    uint8_t output;

    /* Giới hạn ADC */
    if (value_ADC > adc_max)
        value_ADC = adc_max;

    /* Ánh xạ tuyến tính */
    output = out_min
           + ((uint32_t)value_ADC * (out_max - out_min))
             / (adc_max - adc_min);

    return output;
}



/* ---------- main ---------- */
int main(void)
{
    uint8_t TxAddress[5] = {0xEE,0xDD,0xCC,0xBB,0xAA};
    uint8_t TxData[4];



    SysTick_Init();
    NRF24_Init();         // spi_init() called inside
    NRF24_TxMode(TxAddress, 10); // power up and enable CE
    ADC_Init();
    led_init();
    led_off();

    while (1)
    {
     	TxData[0] = Convert_throttle(ADC_Read(2)) / 10;     //  Throttle
     	TxData[1] = Convert_Yaw(ADC_Read(3));               //  Yaw
     	TxData[2] = Convert_Roll_Pitch(ADC_Read(0));		//  Roll
     	TxData[3] = Convert_Roll_Pitch(ADC_Read(1));		//  Pitch

     	NRF24_Transmit(TxData, 4);


    }
    /* should never reach */
    return 0;
}
