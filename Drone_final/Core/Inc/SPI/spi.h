/*
 * spi.h
 *
 *  Created on: Jan 16, 2026
 *      Author: phamt
 */

#ifndef _SPI_H
#define _SPI_H



/**********************************************************************************************
 * INCLUDE
 *********************************************************************************************/
#include "stm32f103xb.h"

/**********************************************************************************************
 * API
 *********************************************************************************************/


/*
 * brief.....Set up for SPI
 * param.....None
 * Retval....None
 */
void spi_init(void);


/*
 * brief.....Transmit data buffer
 * param.....data: Data buffer
 * param.....size: size of data
 * retval....None
 */
void spi1_transmit(uint8_t *data, uint32_t size);

/*
 * brief.....Receive a number of bytes into data buffer
 * param.....data: data received
 * param.....size: size of data received
 * retval....None
 */
void spi1_receive(uint8_t *data, uint32_t size);



#endif /* SRC_SPI_SPI_H_ */
