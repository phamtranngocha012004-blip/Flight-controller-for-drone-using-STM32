
#ifndef INC_I2C_I2C_H_
#define INC_I2C_I2C_H_



/**********************************************************************************************
 * INCLUDE
 *********************************************************************************************/
#include "stm32f103xb.h"

/**********************************************************************************************
 * VARIABLE
 *********************************************************************************************/

typedef enum {
    I2C_OK = 0,
    I2C_ERR_BUSY,
    I2C_ERR_TIMEOUT,
    I2C_ERR_NACK
} I2C_Status;


#define I2C_TIMEOUT 10000             // Timeout I2C
/**********************************************************************************************
 * API
 *********************************************************************************************/

/*
 * brief.....Initialize I2C1 peripheral (PB6-SCL, PB7-SDA)
 * param.....None
 * Retval....None
 */
void i2c_init(void);

/*
 * brief.....Write a single byte to an I2C slave device register
 * param.....dev: Device address (7-bit), reg: Register address, data: Value to write
 * Retval....I2C_Status (OK, BUSY, TIMEOUT, NACK)
 */
I2C_Status i2c_WriteByte(uint8_t dev, uint8_t reg, uint8_t data);

/*
 * brief.....Read multiple bytes from an I2C slave device starting from a register
 * param.....dev: Device address (7-bit), reg: Start register, len: Number of bytes, buf: Pointer to storage
 * Retval....I2C_Status (OK, BUSY, TIMEOUT, NACK)
 */
I2C_Status i2c_ReadMulti(uint8_t dev, uint8_t reg, uint8_t len, uint8_t *buf);


#endif /* INC_I2C_I2C_H_ */
