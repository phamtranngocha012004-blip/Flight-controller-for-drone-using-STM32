
#include "i2c/i2c.h"

void i2c_init(void)
{
    /* Enable clocks for GPIOB, Alternate Function IO, and I2C1 */
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    /* PB6=SCL, PB7=SDA configuration: Alternate Function Open-Drain, 50MHz */
    GPIOB->CRL &= ~((0xF << 24) | (0xF << 28)); // Clear bits for PB6 and PB7

    /* PB6: CNF=11 (AF OD), MODE=11 (Output 50MHz) */
    GPIOB->CRL |= (0b11 << 24) | (0b11 << 26);

    /* PB7: CNF=11 (AF OD), MODE=11 (Output 50MHz) */
    GPIOB->CRL |= (0b11 << 28) | (0b11 << 30);

    /* Reset I2C peripheral */
    I2C1->CR1 = I2C_CR1_SWRST;

    I2C1->CR1 = 0;             // Exit reset state

    /* Configure I2C Speed */
    I2C1->CR2 = 8;             // APB1 clock frequency = 8MHz
    I2C1->CCR = 40;            // 100kHz standard mode (CCR = PCLK / (2 * 100000))
    I2C1->TRISE = 9;           // Maximum rise time (8MHz * 1us + 1)

    /* Enable Acknowledgement and Peripheral */
    I2C1->CR1 |= I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_PE;
}

I2C_Status i2c_WriteByte(uint8_t dev, uint8_t reg, uint8_t data)
{
    uint32_t timeout;

    /* Wait while the bus is busy */
    timeout = I2C_TIMEOUT;
    while (I2C1->SR2 & I2C_SR2_BUSY)
    {
        //if (--timeout == 0) return I2C_ERR_BUSY;
    }

    /* Generate START condition */
    I2C1->CR1 |= I2C_CR1_START;
    timeout = I2C_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_SB)) // Wait for Start Bit (SB) flag
    {
        if (--timeout == 0) return I2C_ERR_TIMEOUT;
    }

    /* Send device address with Write bit (0) */
    I2C1->DR = (dev << 1);
    timeout = I2C_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) // Wait for Address matched (ADDR) flag
    {
        if (I2C1->SR1 & I2C_SR1_AF) return I2C_ERR_NACK; // Check for Acknowledge Failure
        if (--timeout == 0) return I2C_ERR_TIMEOUT;
    }
    (void)I2C1->SR2; // Clear ADDR flag by reading SR2

    /* Send internal register address */
    timeout = I2C_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_TXE)) // Wait for TX buffer empty
    {
        if (--timeout == 0) return I2C_ERR_TIMEOUT;
    }
    I2C1->DR = reg;

    /* Wait for Byte Transfer Finished (BTF) */
    timeout = I2C_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_BTF))
    {
        if (--timeout == 0) return I2C_ERR_TIMEOUT;
    }

    /* Send the data byte */
    I2C1->DR = data;

    /* Wait for completion of data transmission */
    timeout = I2C_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_BTF))
    {
        if (--timeout == 0) return I2C_ERR_TIMEOUT;
    }

    /* Generate STOP condition */
    I2C1->CR1 |= I2C_CR1_STOP;

    return I2C_OK;
}


I2C_Status i2c_ReadMulti(uint8_t dev, uint8_t reg, uint8_t len, uint8_t *buf)
{
    uint32_t timeout;

    if (len == 0)
    {
        return I2C_ERR_TIMEOUT;
    }

    /* Wait while the bus is busy */
    timeout = I2C_TIMEOUT;
    while (I2C1->SR2 & I2C_SR2_BUSY)
    {
        if (--timeout == 0)
        {
            return I2C_ERR_BUSY;
        }
    }

    /* Generate START condition */
    I2C1->CR1 |= I2C_CR1_START;
    timeout = I2C_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_SB))
    {
        if (--timeout == 0)
        {
            return I2C_ERR_TIMEOUT;
        }
    }

    /* Send device address + Write to set the register pointer */
    I2C1->DR = (dev << 1);
    timeout = I2C_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_ADDR))
    {
        if (I2C1->SR1 & I2C_SR1_AF)
        {
            return I2C_ERR_NACK;
        }
        if (--timeout == 0)
        {
            return I2C_ERR_TIMEOUT;
        }
    }
    (void)I2C1->SR2; // Clear ADDR flag

    /* Send the register address to read from */
    I2C1->DR = reg;
    timeout = I2C_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_BTF))
    {
        if (--timeout == 0)
        {
            return I2C_ERR_TIMEOUT;
        }
    }

    /* Generate Repeated START condition */
    I2C1->CR1 |= I2C_CR1_START;
    timeout = I2C_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_SB))
    {
        if (--timeout == 0)
        {
            return I2C_ERR_TIMEOUT;
        }
    }

    /* Send device address + Read bit (1) */
    I2C1->DR = (dev << 1) | 1;
    timeout = I2C_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_ADDR))
    {
        if (--timeout == 0)
        {
            return I2C_ERR_TIMEOUT;
        }
    }

    /* Case: Reading only 1 byte */
    if (len == 1)
    {
        I2C1->CR1 &= ~I2C_CR1_ACK;  // Disable ACK
        (void)I2C1->SR2;            // Clear ADDR flag
        I2C1->CR1 |= I2C_CR1_STOP;  // Generate STOP condition

        timeout = I2C_TIMEOUT;
        while (!(I2C1->SR1 & I2C_SR1_RXNE)) // Wait for RX buffer not empty
        {
            if (--timeout == 0)
            {
                return I2C_ERR_TIMEOUT;
            }
        }
        *buf = I2C1->DR;            // Read data
        return I2C_OK;
    }

    /* Case: Reading multiple bytes */
    I2C1->CR1 |= I2C_CR1_ACK;       // Ensure ACK is enabled
    (void)I2C1->SR2;                // Clear ADDR flag

    while (len > 2)
    {
        timeout = I2C_TIMEOUT;
        while (!(I2C1->SR1 & I2C_SR1_RXNE))
        {
            if (--timeout == 0)
            {
                return I2C_ERR_TIMEOUT;
            }
        }
        *buf++ = I2C1->DR;          // Read data and move pointer
        len--;
    }

    /* Handle last 2 bytes specifically according to STM32 I2C sequence */
    I2C1->CR1 &= ~I2C_CR1_ACK;      // NACK the second to last byte
    I2C1->CR1 |= I2C_CR1_STOP;      // Generate STOP after last byte

    /* Receive second to last byte */
    timeout = I2C_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_RXNE))
    {
        if (--timeout == 0)
        {
            return I2C_ERR_TIMEOUT;
        }
    }
    *buf++ = I2C1->DR;

    /* Receive last byte */
    timeout = I2C_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_RXNE))
    {
        if (--timeout == 0)
        {
            return I2C_ERR_TIMEOUT;
        }
    }
    *buf = I2C1->DR;

    return I2C_OK;
}

