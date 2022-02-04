/*
 * nextionDisplayTask.h
 *
 *  Created on: 24.01.2022
 *      Author: Slawek
 */

#ifndef __NEXTIONDISPLAYTASK_H
#define __NEXTIONDISPLAYTASK_H

#include "main.h"

#define NEXTION_DISPLAY_BUFFERSIZE_UART_2N 7///< Buffer size of 2^7=128byte
#define NEXTION_DISPLAY_BUFFERSIZE_UART (1 << NEXTION_DISPLAY_BUFFERSIZE_UART_2N)

#define NEXTION_RX_BUF_SIZE 	32
#define TIMEOUT_UART_CHAR  		10 //ms

typedef enum {odczytCzujnikow = 1, costam = 2}nextionCyclicalTrace_t;


void StartNextionDisplayTask(void const * argument);




#endif /* __NEXTIONDISPLAYTASK_H */
