
#include <SPI/spi.h>

void spi_init(void)
{
    /* Enable clocks for GPIOA and SPI1 */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN |                  // Enable GPIOA clock
                    RCC_APB2ENR_SPI1EN;                  // Enable SPI1 clock

    /* GPIO configuration:
       - PA5 = SCK  : Alternate function push-pull, 2 MHz
       - PA6 = MISO : Input floating
       - PA7 = MOSI : Alternate function push-pull, 2 MHz
       For AF push-pull: CNF = 10
    */
    GPIOA->CRL &= ~(GPIO_CRL_MODE5_Msk | GPIO_CRL_CNF5_Msk |
                    GPIO_CRL_MODE6_Msk | GPIO_CRL_CNF6_Msk |
                    GPIO_CRL_MODE7_Msk | GPIO_CRL_CNF7_Msk); // Clear configuration for PA5–PA7

    GPIOA->CRL |= (GPIO_CRL_MODE5_1) |                    // PA5 output mode, 2 MHz
                  (GPIO_CRL_CNF5_1)  |                    // PA5 alternate function push-pull
                  (GPIO_CRL_CNF6_0)  |                    // PA6 input floating
                  (GPIO_CRL_MODE7_1) |                    // PA7 output mode, 2 MHz
                  (GPIO_CRL_CNF7_1);                      // PA7 alternate function push-pull

    /* SPI1 configuration */
    SPI1->CR1 = 0;                                       // Reset SPI1 control register 1
    SPI1->CR1 |= SPI_CR1_MSTR;                            // Configure SPI as master
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;               // Enable software NSS management, set NSS high
    SPI1->CR1 |= SPI_CR1_BR_0;                            // Set baud rate = fPCLK / 4
    SPI1->CR1 |= SPI_CR1_SPE;                             // Enable SPI1 peripheral
}

/* Transmit data buffer using polling */
void spi1_transmit(uint8_t *data, uint32_t size)
{
    uint32_t i = 0;                                      // Buffer index
    volatile uint8_t tmp;                                // Temporary variable to clear OVR flag

    while (i < size)
    {
        while (!(SPI1->SR & SPI_SR_TXE));                // Wait until transmit buffer is empty
        SPI1->DR = data[i++];                            // Write one byte to the data register
    }

    while (!(SPI1->SR & SPI_SR_TXE));                    // Wait until last byte is loaded
    while (SPI1->SR & SPI_SR_BSY);                       // Wait until SPI is no longer busy

    /* Clear possible overrun (OVR) flag by reading DR then SR */
    tmp = SPI1->DR;                                      // Read data register
    tmp = SPI1->SR;                                      // Read status register to clear OVR
    (void)tmp;                                           // Prevent unused variable warning
}

/* Receive a number of bytes into data buffer (dummy write 0xFF to generate clock) */
void spi1_receive(uint8_t *data, uint32_t size)
{
    while (size)
    {
        SPI1->DR = 0xFF;                                 // Send dummy byte to generate SPI clock
        while (!(SPI1->SR & SPI_SR_RXNE));               // Wait until receive buffer is not empty
        *data++ = (uint8_t)SPI1->DR;                     // Read received byte
        size--;                                          // Decrease remaining byte count
    }
}

