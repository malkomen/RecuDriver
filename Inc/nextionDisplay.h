/*
 * nextionDisplayTask.h
 *
 *  Created on: 24.01.2022
 *      Author: Slawek
 */

#ifndef __NEXTIONDISPLAYTASK_H
#define __NEXTIONDISPLAYTASK_H

#include "main.h"

#define NEXTION_DISPLAY_BUFFERSIZE_UART_2N 10///< Buffer size of 2^7=128byte
#define NEXTION_DISPLAY_BUFFERSIZE_UART (1 << NEXTION_DISPLAY_BUFFERSIZE_UART_2N)

#define NEXTION_RX_BUF_SIZE 	512
#define TIMEOUT_UART_CHAR  		10 //ms






void StartNextionDisplayTask(void const * argument);




#endif /* __NEXTIONDISPLAYTASK_H */
