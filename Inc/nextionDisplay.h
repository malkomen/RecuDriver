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

#define PLANING_MODE_QTY		4 //liczba trow planowania

typedef enum {odczytCzujnikow = 1, costam = 2}nextionCyclicalTrace_t;

typedef enum
{
	modeWork_zDala 			= 1,
	modeWork_normalny 		= 2,
	modeWork_intensywny 	= 3,
	modeWork_Turbo 			= 4,
	modeWork_okap 			= 5,
	modeWork_Kominek 		= 6,
	modeWork_Priorytetowy 	= 7,
	modeWork_Urlop 			= 8
}modeWork_t;

typedef enum
{
	planing_wDomu 			= 1,
	planing_wBiurze			= 2,
	planing_tydzienWPracy	= 3,
	planing_uzutkownia		= 4
}planing_t;



typedef struct
{
	modeWork_t pon[24];
	modeWork_t wt[24];
	modeWork_t sr[24];
	modeWork_t czw[24];
	modeWork_t pt[24];
	modeWork_t sob[24];
	modeWork_t niedz[24];
}timeTable_t;

/*typedef struct
{
	timeTable_t plan[4];
}timeTablePlaninig_t;*/




void StartNextionDisplayTask(void const * argument);




#endif /* __NEXTIONDISPLAYTASK_H */
