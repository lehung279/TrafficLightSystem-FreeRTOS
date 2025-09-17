/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
FreeRTOS is a market leading RTOS from Real Time Engineers Ltd. that supports
31 architectures and receives 77500 downloads a year. It is professionally
developed, strictly quality controlled, robust, supported, and free to use in
commercial products without any requirement to expose your proprietary source
code.

This simple FreeRTOS demo does not make use of any IO ports, so will execute on
any Cortex-M3 of Cortex-M4 hardware.  Look for TODO markers in the code for
locations that may require tailoring to, for example, include a manufacturer
specific header file.

This is a starter project, so only a subset of the RTOS features are
demonstrated.  Ample source comments are provided, along with web links to
relevant pages on the http://www.FreeRTOS.org site.

Here is a description of the project's functionality:

The main() Function:
main() creates the tasks and software timers described in this section, before
starting the scheduler.

The Queue Send Task:
The queue send task is implemented by the prvQueueSendTask() function.
The task uses the FreeRTOS vTaskDelayUntil() and xQueueSend() API functions to
periodically send the number 100 on a queue.  The period is set to 200ms.  See
the comments in the function for more details.
http://www.freertos.org/vtaskdelayuntil.html
http://www.freertos.org/a00117.html

The Queue Receive Task:
The queue receive task is implemented by the prvQueueReceiveTask() function.
The task uses the FreeRTOS xQueueReceive() API function to receive values from
a queue.  The values received are those sent by the queue send task.  The queue
receive task increments the ulCountOfItemsReceivedOnQueue variable each time it
receives the value 100.  Therefore, as values are sent to the queue every 200ms,
the value of ulCountOfItemsReceivedOnQueue will increase by 5 every second.
http://www.freertos.org/a00118.html

An example software timer:
A software timer is created with an auto reloading period of 1000ms.  The
timer's callback function increments the ulCountOfTimerCallbackExecutions
variable each time it is called.  Therefore the value of
ulCountOfTimerCallbackExecutions will count seconds.
http://www.freertos.org/RTOS-software-timer.html

The FreeRTOS RTOS tick hook (or callback) function:
The tick hook function executes in the context of the FreeRTOS tick interrupt.
The function 'gives' a semaphore every 500th time it executes.  The semaphore
is used to synchronise with the event semaphore task, which is described next.

The event semaphore task:
The event semaphore task uses the FreeRTOS xSemaphoreTake() API function to
wait for the semaphore that is given by the RTOS tick hook function.  The task
increments the ulCountOfReceivedSemaphores variable each time the semaphore is
received.  As the semaphore is given every 500ms (assuming a tick frequency of
1KHz), the value of ulCountOfReceivedSemaphores will increase by 2 each second.

The idle hook (or callback) function:
The idle hook function queries the amount of free FreeRTOS heap space available.
See vApplicationIdleHook().

The malloc failed and stack overflow hook (or callback) functions:
These two hook functions are provided as examples, but do not contain any
functionality.
*/

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4_discovery.h"
/* Kernel includes. */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"



/*-----------------------------------------------------------*/
#define mainQUEUE_LENGTH 5

#define red  		0
#define amber  		1
#define green  		2
#define no_change 	3

#define lane_capacity 19

#define red_light	GPIO_Pin_0
#define amber_light	GPIO_Pin_1
#define green_light	GPIO_Pin_2


#define green_light_time	10000
#define	amber_light_time	2000
#define	red_light_time		7000

#define Shift_Register_Clock 	GPIO_Pin_7
#define Shift_Register_Data 	GPIO_Pin_6
#define Shift_Register_Reset 	GPIO_Pin_8
/*
 * TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware( void );

/*
 * The queue send and receive tasks as described in the comments at the top of
 * this file.
 */
static void Traffic_light( void *pvParameters );
static void Create_Traffic( void *pvParameters );
static void Display_Traffic( void *pvParameters );
static void Adjust_Traffic(void *pvParameters);
static void clear_lane();
static void add_car(uint32_t car_stat);
static void Init_Traffic_Light(void);
static void Init_Cars(void);
static void Init_TrafficKnob();
static void software_timer_up(TimerHandle_t xTimer);
static void update_traffic_light (uint8_t light);
static uint32_t accumulate_traffic(uint32_t Traff);
static uint16_t read_traffic_controll_knob();

// Device Drivers:
/* GPIO Driver Functions ********************************/
static void GPIO_Init_Shift_Register (void);
static void GPIO_Init_Traffic_Light (void);
static void GPIO_Update_LEDs(uint8_t LEDs);
static void GPIO_Reset_Shift_Register(void);
static void GPIO_Shift_Register_bit_On(void);
static void GPIO_Shift_Register_bit_Off(void);
/* ADC Driver Functions ********************************/
static void Init_ADC();
static uint16_t ADC_Start_Conversion();

xQueueHandle xTrafficFlowQueue_handle = 0;
xQueueHandle xTrafficNewLightQueue_handle = 0;
xQueueHandle xTrafficExpiredLightQueue_handle = 0;
xQueueHandle xTrafficAdjustRateQueue_handle = 0;
xQueueHandle xTrafficLightAdjustTimingQueue_handle = 0;

TimerHandle_t xRedLightTimmer_handle = 0;
TimerHandle_t xAmberLightTimmer_handle = 0;
TimerHandle_t xGreenLightTimmer_handle = 0;

/*-----------------------------------------------------------*/

int main(void)
{
	// Initializes the traffic light GPIO pins
	Init_Traffic_Light();
	// Initializes the GPIO pins that are connected to Serial to parallel shift registers
	// that drive LEDs representing cars.
	Init_Cars();
	// Initializes the ADC peripheral that the Potentiometer that adjusts the traffic load is connected to.
	Init_TrafficKnob();

	// No car on the road (all LEDs off)
	clear_lane();
	// Traffic lights off
	update_traffic_light(no_change);
	/* Configure the system ready to run the demo.  The clock configuration
	can be done here if it was not done before main() was called. */
	prvSetupHardware();

	/* Create the queue used by the queue send and queue receive tasks.
	http://www.freertos.org/a00116.html */
	xTrafficFlowQueue_handle = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));
	xTrafficNewLightQueue_handle = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint8_t));
	xTrafficExpiredLightQueue_handle = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint8_t));
	xTrafficAdjustRateQueue_handle = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint16_t));
	xTrafficLightAdjustTimingQueue_handle = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint16_t));


	/* Add to the registry, for the benefit of kernel aware debugging. */
	vQueueAddToRegistry( xTrafficFlowQueue_handle, "TrafficFlowQueue" );
	vQueueAddToRegistry( xTrafficNewLightQueue_handle, "TrafficLightStatusQueue1" );
	vQueueAddToRegistry( xTrafficExpiredLightQueue_handle, "TrafficLightStatusQueue2" );
	vQueueAddToRegistry( xTrafficAdjustRateQueue_handle, "TrafficAdjustQueue1" );
	vQueueAddToRegistry( xTrafficLightAdjustTimingQueue_handle, "TrafficAdjustQueue2" );


	xTaskCreate( Traffic_light, "TrafficLight", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate( Create_Traffic, "CreateTraffic", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate( Display_Traffic, "DisplayTraffic", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate( Adjust_Traffic, "AdjustTraffic", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	return 0;
}

/*************************************************************
		Tasks
*************************************************************/

static void Traffic_light( void *pvParameters )
{
	TickType_t	red_light_on = pdMS_TO_TICKS(red_light_time);
	TickType_t	amber_light_on = pdMS_TO_TICKS(amber_light_time);
	TickType_t  green_light_on = pdMS_TO_TICKS(green_light_time);

	uint16_t 	Traffic_rate = 0;
	uint8_t 	rx_data;
	uint8_t 	current_light = green;

	xRedLightTimmer_handle = xTimerCreate("RedLightTimer", red_light_on, pdFALSE, 0, software_timer_up); // Software timer ID is initialized to zero.
	xAmberLightTimmer_handle = xTimerCreate("AmberLightTimer", amber_light_on, pdFALSE, 0, software_timer_up); // Software timer ID is initialized to zero.
	xGreenLightTimmer_handle = xTimerCreate("GreenLightTimer", green_light_on, pdFALSE, 0, software_timer_up); // Software timer ID is initialized to zero.

	while(1)
	{
		if(!xQueueReceive(xTrafficLightAdjustTimingQueue_handle,&Traffic_rate,200))
		{
			printf("xTrafficLightAdjustTimingQueue empty!\n");
		}

		red_light_on = pdMS_TO_TICKS(red_light_time - 1000*((Traffic_rate)/1000));
		green_light_on = pdMS_TO_TICKS(green_light_time + 2000*((Traffic_rate)/1000));
		if(current_light != no_change)
		{
			// Send the new traffic light status to Display_Traffic task.
			if(!xQueueSend(xTrafficNewLightQueue_handle, &current_light, 1000))
			{
				printf("xTrafficNewLightQueue full!\n");
			}
		}

		if (current_light == green)
		{
			xTimerChangePeriod(xGreenLightTimmer_handle, green_light_on, 0);
			xTimerStart(xGreenLightTimmer_handle, 0);
			current_light = no_change;
			update_traffic_light (green);
		}
		else if (current_light == amber)
		{
			xTimerStart(xAmberLightTimmer_handle, 0);
			current_light = no_change;
			update_traffic_light (amber);
		}
		else if (current_light == red)
		{
			xTimerChangePeriod(xRedLightTimmer_handle, red_light_on, 0);
			xTimerStart(xRedLightTimmer_handle, 0);
			current_light = no_change;
			update_traffic_light (red);
		}
		if(xQueueReceive(xTrafficExpiredLightQueue_handle, &rx_data, 1000))
		{
			if (rx_data == green)
				current_light = amber;
			else if (rx_data == amber)
						current_light = red;
			else if (rx_data == red)
						current_light = green;
		}
		vTaskDelay(500);
	}
}

/*-----------------------------------------------------------*/

static void Create_Traffic( void *pvParameters )
{
	static uint8_t 	tRate_new = 0;
	static int8_t 	tRate_current = 0;
	uint8_t 		car = 0;
	static uint32_t Traffic = 0;
	uint8_t 		rx_light = no_change;
	uint8_t 		rand_number = 0;
	uint16_t 		rx_TrafficLoad = 0;

	while(1)
	{
		rand_number = rand()%2;

		xQueueReceive(xTrafficNewLightQueue_handle, &rx_light, 100);
		if (xQueueReceive(xTrafficAdjustRateQueue_handle, &rx_TrafficLoad, 100))
		{
			tRate_new = 4 - (rx_TrafficLoad / 1000);
		}

		if (tRate_current == -1)
		{
			tRate_current = tRate_new;
		}
		if (tRate_current == 0)
			car = 1;
		else
			car = 0;
		car = car * rand_number;
		if (rx_light == green)
		{
			if (car)
			{
				Traffic = (Traffic << 1) + 1;
			}
			else
				Traffic = Traffic << 1;
		}
		if (rx_light == amber || rx_light == red)
		{
			Traffic = accumulate_traffic(Traffic);
			if (car)
			{
				Traffic = Traffic | 1;
			}
		}
		tRate_current--;

  		if( xQueueSend(xTrafficFlowQueue_handle,&Traffic,1000))
		{
			printf("Sent %d car\n", car);
			vTaskDelay(400);
		}
		else
		{
			printf("Tsend fd!\n");
		}
	}
}

/*-----------------------------------------------------------*/

static void Display_Traffic( void *pvParameters )
{
	uint32_t rx_data;
	uint32_t  car_stat;

	while(1)
	{

		if(xQueueReceive(xTrafficFlowQueue_handle, &rx_data, 1000))
		{
			clear_lane();
			for (int i=lane_capacity ; i> 0; i--)
			{
				car_stat = rx_data & (1<<(i-1));
				add_car(car_stat);
			}
		}
		else
		{
			printf("QEmpty!\n");
		}
		vTaskDelay(400);
	}
}


/************************************************************/

static void Adjust_Traffic(void *pvParameters)
{
	uint16_t Traffic_Knob;
	while(1)
	{
		Traffic_Knob = read_traffic_controll_knob();


		if(!xQueueSend(xTrafficLightAdjustTimingQueue_handle,&Traffic_Knob,1000))
		{
			printf("xTrafficLightAdjustTimingQueue full!\n");
		}
  		if( xQueueSend(xTrafficAdjustRateQueue_handle,&Traffic_Knob,1000))
		{
  			printf("Traffic Knob %d \n", Traffic_Knob);
			vTaskDelay(1000);
		}
		else
		{
			printf("Tsend fd!\n");
		}
	}
}


/*** Software Timer *********************************************************/

static void software_timer_up(TimerHandle_t xTimer)
{
	uint8_t expired_light=0;

	if (xTimer == xRedLightTimmer_handle)
	{
		expired_light = red;
	}
	else if (xTimer == xAmberLightTimmer_handle)
	{
		expired_light = amber;
	}
	else if (xTimer == xGreenLightTimmer_handle)
	{
		expired_light = green;
	}
	if( xQueueSend(xTrafficExpiredLightQueue_handle,&expired_light,0))
		printf("%d light times up!\n", expired_light);
}

/*   Helper Functions **************************************/
/**********************************************************/

static uint32_t accumulate_traffic(uint32_t Traffic)
{
	uint32_t shift_bit = 0x80, tmp = 0;

	uint32_t Traffic_befIntr = Traffic & 0xFF; 		// cars before the intersection
	uint32_t Traffic_aftIntr = Traffic & 0xFFFFFF00; 	// cars after the intersection

	Traffic_aftIntr = Traffic_aftIntr << 1;
	while (shift_bit)
	{
		if((Traffic_befIntr & shift_bit) == 0)
		{
			Traffic_befIntr = ((Traffic_befIntr << 1) & (~tmp)) | tmp;
			Traffic_befIntr = Traffic_befIntr & 0xFF;
			Traffic = Traffic_befIntr | Traffic_aftIntr;
			return Traffic;
		}
		tmp = tmp | shift_bit;
		shift_bit = shift_bit >> 1;
	}
	Traffic = Traffic_befIntr | Traffic_aftIntr;
	return Traffic;
}



/************************************************************
Device Driver Access Functions
*************************************************************/

// GPIO --------------------------------------------
static void Init_Cars(void)
{
	GPIO_Init_Shift_Register();
}

static void Init_Traffic_Light()
{
	GPIO_Init_Traffic_Light();
}


static void clear_lane()
{
	GPIO_Reset_Shift_Register();
}

static void update_traffic_light (uint8_t light)
{
	GPIO_Update_LEDs(light);
}

static void add_car(uint32_t car_stat)
{
	if(car_stat)
	{
		GPIO_Shift_Register_bit_On();

	}
	else
	{
		GPIO_Shift_Register_bit_Off();
	}
}


// ADC ---------------------------------------------
static void Init_TrafficKnob()
{
	Init_ADC();
}


static uint16_t read_traffic_controll_knob()
{
	return (ADC_Start_Conversion());
}




/*** Device Driver Functions ************************
*****************************************************/
// GPIO ----------------------------------
static void GPIO_Init_Shift_Register (void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* Enable the GPIO_LED Clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	/* Configure the GPIO_LED pin */
	GPIO_InitStructure.GPIO_Pin = Shift_Register_Reset | Shift_Register_Data | Shift_Register_Clock;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

static void GPIO_Init_Traffic_Light (void)
{

	  GPIO_InitTypeDef  GPIO_InitStructure;

	  /* Enable the GPIO_LED Clock */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	  /* Configure the GPIO_LED pin */
	  GPIO_InitStructure.GPIO_Pin = red_light | amber_light | green_light ;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);
}


static void GPIO_Update_LEDs(uint8_t LEDs)
{
	if (LEDs == red)
	{
		GPIO_SetBits(GPIOC, red_light);
		GPIO_ResetBits(GPIOC, amber_light);
		GPIO_ResetBits(GPIOC, green_light);
	}
	else if (LEDs == amber)
	{
		GPIO_ResetBits(GPIOC, red_light);
		GPIO_SetBits(GPIOC, amber_light);
		GPIO_ResetBits(GPIOC, green_light);
	}
	else if (LEDs == green)
	{
		GPIO_ResetBits(GPIOC, red_light);
		GPIO_ResetBits(GPIOC, amber_light);
		GPIO_SetBits(GPIOC, green_light);
	}
	else
	{
		GPIO_ResetBits(GPIOC, red_light);
		GPIO_ResetBits(GPIOC, amber_light);
		GPIO_ResetBits(GPIOC, green_light);
	}

}

static void GPIO_Reset_Shift_Register(void)
{
	GPIO_ResetBits(GPIOC, Shift_Register_Reset);
	for (int i = 0; i<10; i++);
	GPIO_SetBits(GPIOC, Shift_Register_Reset);
}

static void GPIO_Shift_Register_bit_On(void)
{
	// Reset the data pin.
	GPIO_SetBits(GPIOC, Shift_Register_Data);
	//Generate a pulse on the clock pin of the shift register.
	GPIO_ResetBits(GPIOC, Shift_Register_Clock);
	for (int i = 0; i<5; i++);
	GPIO_SetBits(GPIOC, Shift_Register_Clock);
	GPIO_ResetBits(GPIOC, Shift_Register_Data);
	//printf("A car was added to the intersection!\n");
}

static void GPIO_Shift_Register_bit_Off(void)
{
	// Reset the data pin.
	GPIO_ResetBits(GPIOC, Shift_Register_Data);
	//Generate a pulse on the clock pin of the shift register.
	GPIO_ResetBits(GPIOC, Shift_Register_Clock);
	for (int i = 0; i<5; i++);
	GPIO_SetBits(GPIOC, Shift_Register_Clock);
	//printf("No car at this time!\n");
}

/*-- ADC functions ---------------------------------------------------------*/

static void Init_ADC()
{
	// Enable clock for ADC1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    // Init GPIOB for ADC input
    GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	// Init ADC1
    ADC_InitTypeDef ADC_InitStruct;
	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_ExternalTrigConv = DISABLE;
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_NbrOfConversion = 1;
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
	ADC_Init(ADC1, &ADC_InitStruct);
	ADC_Cmd(ADC1, ENABLE);
	// Select input channel for ADC1
	// ADC1 channel 9 is on PB1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_84Cycles);
}

static uint16_t ADC_Start_Conversion()
{
	uint16_t converted_data;
	// Start ADC conversion
	ADC_SoftwareStartConv(ADC1);
	// Wait until conversion is finish
	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
	converted_data = ADC_GetConversionValue(ADC1);
	return converted_data;
}


/*--- FreeRTOS Hok Functions --------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software 
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  pxCurrentTCB can be
	inspected in the debugger if the task name passed into this function is
	corrupt. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.

	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );

	/* TODO: Setup the clocks, etc. here, if they were not configured before
	main() was called. */
}

