/*
 * car_wash.c
 *
 *  Created on: 14 черв. 2018 р.
 *      Author: Romko
 */

#include "car_wash.h"
#include "main.h"


#define FOAM_ST_DURATION	30
#define BRUSH_ST_DURATION   30
#define WASHIG_ST_DURATION  60
#define DRYING_ST_DURATION  30


static inline U8 check_car_in_slot_1()
{
	return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
}

static inline U8 check_car_in_slot_2()
{
	return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
}

static inline U8 check_car_in_slot_3()
{
	return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
}

static inline U8 check_car_in_slot_4()
{
	return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);
}

typedef void (*st_handle)(void*);


typedef enum
{
	ST_WAITING_FOR_START = 0,
	ST_FOAM,
	ST_BRUSH,
	ST_WASHING,
	ST_DRYING,
	ST_END_WASH,
	ST_MAX,
} SLOT_STATE;


typedef struct CAR_SLOT_S
{
	SLOT_STATE State;
	U64 StateCurrentTime;
	U64 StateTimeout;
	U32 CarSlot;
	U8 (*is_car_present)(void);
	st_handle st_process[ST_MAX];
} WASHING_MACHINE_ST;

static WASHING_MACHINE_ST WM[MAX_CAR_SLOT];
static int app_start_button_pressed;

static void AppInit(void);

static void waiting_state_handler(WASHING_MACHINE_ST* wm_struct)
{
	if(0 == wm_struct->StateCurrentTime && wm_struct->is_car_present())
	{
		printf("Car #%d Waiting for START button press\r\n", wm_struct->CarSlot + 1);
		wm_struct ->StateCurrentTime++;
	}

	if(app_start_button_pressed && wm_struct->is_car_present())
	{
		wm_struct->State = ST_FOAM;
		wm_struct ->StateCurrentTime = 0;
	}
}


static void foam_state_handler(WASHING_MACHINE_ST* wm_struct)
{
	if(0 == wm_struct->StateCurrentTime)
	{
		printf("Car #%d Start foam\r\n", wm_struct->CarSlot + 1);
	}

	wm_struct ->StateCurrentTime++;

	if(wm_struct ->StateCurrentTime > FOAM_ST_DURATION)
	{
		wm_struct->State = ST_BRUSH;
		wm_struct ->StateCurrentTime = 0;
	}
}

static void brush_state_handler(WASHING_MACHINE_ST* wm_struct)
{
	if(0 == wm_struct->StateCurrentTime)
	{
		printf("Car #%d Start brush\r\n", wm_struct->CarSlot + 1);
	}

	wm_struct ->StateCurrentTime++;

	if(wm_struct ->StateCurrentTime > BRUSH_ST_DURATION)
	{
		wm_struct->State = ST_WASHING;
		wm_struct ->StateCurrentTime = 0;
	}
}

static void washing_state_handler(WASHING_MACHINE_ST* wm_struct)
{
	if(0 == wm_struct->StateCurrentTime)
	{
		printf("Car #%d Start washing\r\n", wm_struct->CarSlot + 1);
	}

	wm_struct ->StateCurrentTime++;

	if(wm_struct ->StateCurrentTime > WASHIG_ST_DURATION)
	{
		wm_struct->State = ST_DRYING;
		wm_struct ->StateCurrentTime = 0;
	}
}

static void drying_state_handler(WASHING_MACHINE_ST* wm_struct)
{
	if(0 == wm_struct->StateCurrentTime)
	{
		printf("Car #%d Start drying\r\n", wm_struct->CarSlot + 1);
	}

	wm_struct ->StateCurrentTime++;

	if(wm_struct ->StateCurrentTime > DRYING_ST_DURATION)
	{
		wm_struct->State = ST_END_WASH;
		wm_struct ->StateCurrentTime = 0;
	}

}

static void end_state_handler(WASHING_MACHINE_ST* wm_struct)
{
	printf("Car #%d is ready\r\n", wm_struct->CarSlot + 1);
	wm_struct->State = ST_WAITING_FOR_START;
	wm_struct ->StateCurrentTime = 0;
}





void ApplicationScan(void)
{
	AppInit();
	int i;
	while(1)
	{
		if(IsTimeReached())
		{
			ButtonDissableInt();
			app_start_button_pressed = ButtonGetAndConsumeEvent();
			ButtonEnableInt();
			/* Some kind of filtering for button, needs to be pressed xx ms to initiate*/
			for(i = 0; i< MAX_CAR_SLOT; i++)
			{
				if(NULL != WM[i].st_process[WM[i].State])
				{
					WM[i].st_process[WM[i].State](&WM[i]);
				}
			}
		}
	}
}



void AppInit(void)
{
	int i;
	for(i = 0; i< MAX_CAR_SLOT; i++)
	{
		WM[i].st_process[ST_WAITING_FOR_START] = (void*)waiting_state_handler;
		WM[i].st_process[ST_FOAM]              = (void*)foam_state_handler;
		WM[i].st_process[ST_BRUSH]             = (void*)brush_state_handler;
		WM[i].st_process[ST_WASHING]           = (void*)washing_state_handler;
		WM[i].st_process[ST_DRYING]            = (void*)drying_state_handler;
		WM[i].st_process[ST_END_WASH]          = (void*)end_state_handler;
		WM[i].CarSlot = i;
	}
	WM[0].is_car_present = check_car_in_slot_1;
	WM[1].is_car_present = check_car_in_slot_2;
	WM[2].is_car_present = check_car_in_slot_3;
	WM[3].is_car_present = check_car_in_slot_4;
}



