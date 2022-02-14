/*
 * automationTask.h
 *
 *  Created on: 14.02.2022
 *      Author: Slawek
 */

#ifndef __AUTOMATIONTASK_H
#define __AUTOMATIONTASK_H

#include "main.h"


void StartAutomationTask(void const * argument);

#define PLAN_MODE_QTY		4 //liczba trybow planowania

typedef enum {odczytCzujnikow = 1, costam = 2}nextionCyclicalTrace_t;

typedef enum
{
	modeWork_OFF			= 0,
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
	planner_wDomu 			= 0,
	planner_wBiurze			= 1,
	planner_tydzienWPracy	= 2,
	planner_uzytkownia		= 3
}planner_t;



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

typedef struct
{
	timeTable_t plan[PLAN_MODE_QTY];
	planner_t activePlan;
}timeTablePlanner_t;


timeTablePlanner_t* GetTimeTablePlanner();

HAL_StatusTypeDef SaveSetting();


#endif /* __AUTOMATIONTASK_H */
