/***************************************************************************//**
 * @file
 * @brief
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement.
 * The software is governed by the sections of the MSLA applicable to Micrium
 * Software.
 *
 ******************************************************************************/

/*
*********************************************************************************************************
*
*                                             EXAMPLE MAIN
*
* File : ex_main.c
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*********************************************************************************************************
*/

#include  <bsp_os.h>
#include  "bsp.h"

#include  <cpu/include/cpu.h>
#include  <common/include/common.h>
#include  <kernel/include/os.h>
#include  <kernel/include/os_trace.h>

#include  <common/include/lib_def.h>
#include  <common/include/rtos_utils.h>
#include  <common/include/toolchains.h>

#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_core.h"

#include "capsense.h"
#include "em_acmp.h"

#include "app.h"
/*
*********************************************************************************************************
*********************************************************************************************************
*                                             LOCAL DEFINES
*********************************************************************************************************
*********************************************************************************************************
*/

#define  EX_MAIN_START_TASK_PRIO              	20u
#define  EX_MAIN_LED_OUTPUT_TASK_PRIO           21u
#define  EX_MAIN_SLIDER_INPUT_TASK_PRIO         22u
#define  EX_MAIN_BUTTON_INPUT_TASK_PRIO         23u
#define  EX_MAIN_IDLE_TASK_PRIO              	24u

#define  EX_MAIN_START_TASK_STK_SIZE         	512u
#define  EX_MAIN_BUTTON_INPUT_TASK_STK_SIZE		512u
#define  EX_MAIN_SLIDER_INPUT_TASK_STK_SIZE		512u
#define  EX_MAIN_LED_OUTPUT_TASK_STK_SIZE		512u
#define  EX_MAIN_IDLE_TASK_STK_SIZE			 	512u

/*
*********************************************************************************************************
*********************************************************************************************************
*                                        LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*********************************************************************************************************
*/

/* Start Task Stack.                                    */
static  CPU_STK  Ex_MainStartTaskStk[EX_MAIN_START_TASK_STK_SIZE];
/* Start Task TCB.                                      */
static  OS_TCB   Ex_MainStartTaskTCB;

/* Idle Task Stack.                                    */
static  CPU_STK  Ex_MainIdleTaskStk[EX_MAIN_IDLE_TASK_STK_SIZE];
/* Idle Task TCB.                                      */
static  OS_TCB   Ex_MainIdleTaskTCB;

/* ButtonInput Task Stack.                                    */
static  CPU_STK  Ex_MainButtonInputTaskStk[EX_MAIN_BUTTON_INPUT_TASK_STK_SIZE];
/* ButtonInput Task TCB.                                      */
static  OS_TCB   Ex_MainButtonInputTaskTCB;

/* SliderInput Task Stack.                                    */
static  CPU_STK  Ex_MainSliderInputTaskStk[EX_MAIN_SLIDER_INPUT_TASK_STK_SIZE];
/* SliderInput Task TCB.                                      */
static  OS_TCB   Ex_MainSliderInputTaskTCB;

/* LedOutput Task Stack.                                    */
static  CPU_STK  Ex_MainLedOutputTaskStk[EX_MAIN_LED_OUTPUT_TASK_STK_SIZE];
/* LedOutput Task TCB.                                      */
static  OS_TCB   Ex_MainLedOutputTaskTCB;


/* Create Message Queue */
static OS_Q			queue;
static OS_MSG_SIZE  msg_size = sizeof(int);
/* Create OS Task Flags */
static OS_FLAG_GRP	flg;
/* Create semaphore */
static OS_SEM		sem;
/* Create OS Timer */
static OS_TMR		tmr;
/* Create enum for the flags */
enum btn_flag_enum {
	btn_none_pressed,
	btn_0_pressed,
	btn_0_unpressed,
	btn_1_pressed,
	btn_1_unpressed,
	btn_both_pressed,
};
/* Create enum for messages to be passed by peripheral tasks */
enum led_codes {
	btn0,
	btn1,
	btn_none,
	sld0,
	sld1,
	sld_none,
};
// Debug variable purely for debug purposes
int debug_flag;

/*
*********************************************************************************************************
*********************************************************************************************************
*                                       LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*********************************************************************************************************
*/
// task main functions
static  void  Ex_MainStartTask (void  *p_arg);
static  void  Ex_MainIdleTask (void  *p_arg);
static  void  Ex_MainButtonInputTask (void  *p_arg);
static  void  Ex_MainSliderInputTask (void  *p_arg);
static  void  Ex_MainLedOutputTask (void  *p_arg);
// callback function for OSTimer
static void MyCallback(OS_TMR p_tmr, void *p_arg);
// led driver function
static void led_drive(bool PB0_status, bool PB1_status, uint8_t slider_pos);



/*
*********************************************************************************************************
*********************************************************************************************************
*                                       GLOBAL VARIABLES
*********************************************************************************************************
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*********************************************************************************************************
*                                          GLOBAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C applications. It is assumed that your code will
*               call main() once you have performed all necessary initialization.
*
* Argument(s) : None.
*
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
/***************************************************************************//**
 * @brief
 *   Creates the Startup Task and then begins the OS.
 *
 * @details
 * 	 Creates the Startup Task which in turn creates all other tasks.
 * 	 Then the OS is started, which runs indefinitely and does not return.
 *
 *
 ******************************************************************************/
int  main (void)
 {
    RTOS_ERR  err;


    BSP_SystemInit();                                           /* Initialize System.                                   */
    CPU_Init();                                                 /* Initialize CPU.                                      */

    OS_TRACE_INIT();
    OSInit(&err);                                               /* Initialize the Kernel.                               */
                                                                /*   Check error code.                                  */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);


    OSTaskCreate(&Ex_MainStartTaskTCB,                          /* Create the Start Task.                               */
                 "Ex Main Start Task",
                  Ex_MainStartTask,
                  DEF_NULL,
                  EX_MAIN_START_TASK_PRIO,
                 &Ex_MainStartTaskStk[0],
                 (EX_MAIN_START_TASK_STK_SIZE / 10u),
                  EX_MAIN_START_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);
    /*   Check error code.                                  */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

    /* Start the kernel.                                    */
    OSStart(&err);
    /*   Check error code.                                  */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);


    return (1);
}


/***************************************************************************//**
 * @brief
 *   Creates all other user-defined tasks.
 *
 * @details
 * 	 Creates the button input task, idle task, LED control task, and slider input task.
 *
 *
 ******************************************************************************/

static  void  Ex_MainStartTask (void  *p_arg)
{
    RTOS_ERR  err;


    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif

    Common_Init(&err);                                          /* Call common module initialization example.           */
    APP_RTOS_ASSERT_CRITICAL(err.Code == RTOS_ERR_NONE, ;);

    BSP_OS_Init();                                              /* Initialize the BSP. It is expected that the BSP ...  */
                                                                /* ... will register all the hardware controller to ... */
                                                                /* ... the platform manager at this moment.             */
    // initialize common modules for all tasks
    cmu_open();
    /* Create the Idle Task.                               */
    OSTaskCreate(&Ex_MainIdleTaskTCB,
                 "Ex Main Idle Task",
                  Ex_MainIdleTask,
                  DEF_NULL,
                  EX_MAIN_IDLE_TASK_PRIO,
                 &Ex_MainIdleTaskStk[0],
                 (EX_MAIN_IDLE_TASK_STK_SIZE / 10u),
                  EX_MAIN_IDLE_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);
    /* Create the Button Input Task.                               */
    OSTaskCreate(&Ex_MainButtonInputTaskTCB,
                 "Ex Main Button Input Task",
                  Ex_MainButtonInputTask,
                  DEF_NULL,
                  EX_MAIN_BUTTON_INPUT_TASK_PRIO,
                 &Ex_MainButtonInputTaskStk[0],
                 (EX_MAIN_BUTTON_INPUT_TASK_STK_SIZE / 10u),
                  EX_MAIN_BUTTON_INPUT_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);
    /* Create the Slider Input Task.                               */
    OSTaskCreate(&Ex_MainSliderInputTaskTCB,
                 "Ex Main SliderInput Task",
                  Ex_MainSliderInputTask,
                  DEF_NULL,
                  EX_MAIN_SLIDER_INPUT_TASK_PRIO,
                 &Ex_MainSliderInputTaskStk[0],
                 (EX_MAIN_SLIDER_INPUT_TASK_STK_SIZE / 10u),
                  EX_MAIN_SLIDER_INPUT_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);
    /* Create the Led Output Task.                               */
    OSTaskCreate(&Ex_MainLedOutputTaskTCB,
                 "Ex Main Led Output Task",
                  Ex_MainLedOutputTask,
                  DEF_NULL,
                  EX_MAIN_LED_OUTPUT_TASK_PRIO,
                 &Ex_MainLedOutputTaskStk[0],
                 (EX_MAIN_LED_OUTPUT_TASK_STK_SIZE / 10u),
                  EX_MAIN_LED_OUTPUT_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);
    // Create the message queue
    OSQCreate((OS_Q	*)
    		&queue,
			(CPU_CHAR *)"Message Queue",
			(OS_MSG_QTY)4,
			(RTOS_ERR *)&err);
    // Create the event flag
    OSFlagCreate((OS_FLAG_GRP *)
    		&flg,
			(CPU_CHAR *) "button flag",
			(OS_FLAGS) 0,
			(RTOS_ERR *)&err);
    // Create the semaphore
    OSSemCreate ((OS_SEM *) &sem,
            (CPU_CHAR *) "Slider Semaphore",
            (OS_SEM_CTR) 0,
            (RTOS_ERR *) &err);
    // Create the OSTimer
    OSTmrCreate ((OS_TMR *) &tmr,
			(CPU_CHAR *) "OS Timer",
			(OS_TICK) 10,
			(OS_TICK) 10,
			(OS_OPT) OS_OPT_TMR_PERIODIC,
			(OS_TMR_CALLBACK_PTR) &MyCallback,
			(void *) 0,
			(RTOS_ERR *) &err);
    // Start the timer
    OSTmrStart ((OS_TMR *) &tmr, (RTOS_ERR *) &err);

    while (DEF_ON) {
                                                                /* Delay Start Task execution for                       */
        OSTimeDly( 1000,                                        /*   1000 OS Ticks                                      */
                   OS_OPT_TIME_DLY,                             /*   from now.                                          */
                  &err);
                                                                /*   Check error code.                                  */
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}

/***************************************************************************//**
 * @brief
 *   Puts the CPU to EM1 energy mode every time the task is executed.
 *
 * @details
 * 	 For the duration of timer ticks specified in the OSTimeDly function, will
 * 	 ready and run the task this frequently, putting the processor into EM1.
 *
 *
 ******************************************************************************/

static  void  Ex_MainIdleTask (void  *p_arg)
{
    RTOS_ERR  err;


    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif
                                                                /* ... will register all the hardware controller to ... */
                                                                /* ... the platform manager at this moment.             */

    while (DEF_ON) {
    	EMU_EnterEM1();
        /* Delay Start Task execution for                       */
		OSTimeDly( 100,                                        /*   1000 OS Ticks                                      */
					OS_OPT_TIME_DLY,                             /*   from now.                                          */
					&err);
                                                                /*   Check error code.                                  */
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}

/***************************************************************************//**
 * @brief
 *   Checks the values of the pushbuttons.
 *
 * @details
 * 	 For duration specified by OS timer ticks in delay function, will ready and eventually run this
 * 	 task to poll the values of the pushbuttons.
 *
 ******************************************************************************/

static  void  Ex_MainButtonInputTask (void  *p_arg)
{
    RTOS_ERR  err;
    OS_FLAGS flag;
    int msg;

    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif
                                                                /* ... will register all the hardware controller to ... */
                                                                /* ... the platform manager at this moment.             */
    buttons_setup();
    while (DEF_ON) {
    	// check the event flag
    	flag = OSFlagPend ((OS_FLAG_GRP  *) &flg,
			(OS_FLAGS)      0x0F,
			(OS_TICK)       0,
			(OS_OPT)        OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_BLOCKING,
			(CPU_TS *)      NULL,
			(RTOS_ERR *)    &err);
    	debug_flag = flag;
    	// determine the status of the buttons based on the flag
    	switch(flag) {
    		case 0b00000000:
    			msg = btn_none;
				break;
    		case 0b00000001:
    			msg = btn0;
    			break;
    		case 0b00000010:
    			msg = btn_none;
    			break;
    		case 0b00000100:
    			msg = btn1;
    			break;
    		case 0b00000101:
    			msg = btn_none;
    			break;
    		case 0b00000110:
    			msg = btn1;
    			break;
    		case 0b00000111:
    			msg = btn1;
    			break;
    		case 0b00001000:
    			msg = btn_none;
    			break;
    		case 0b00001001:
    			msg = btn0;
    			break;
    		case 0b00001010:
    			msg = btn_none;
    			break;
    		case 0b00001011:
    			msg = btn_none;
    			break;
    		case 0b00001100:
    			msg = btn_none;
    			break;
    		case 0b00001101:
    			msg = btn0;
    			break;
    		case 0b00001110:
    			msg = btn_none;
    			break;
    		case 0b00001111:
    			msg = btn_none;
    			break;
    		default:
    			break;
    	}
    	// send message to the queue
    	OSQPost ((OS_Q *) &queue, &msg, (OS_MSG_SIZE) sizeof(msg), (OS_OPT) OS_OPT_POST_FIFO, (RTOS_ERR *) &err);
        /* Delay Start Task execution for                       */
		OSTimeDly( 10, OS_OPT_TIME_DLY, &err);
		/*   Check error code.                                  */
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}

/***************************************************************************//**
 * @brief
 *   Obtains the value of the slider.
 *
 * @details
 * 	 For the duration of specified by OS ticks, will begin this task. However,
 * 	 this task is also visited for each time the capacitor pads are measured using
 * 	 the OS time tick.
 *
 *
 ******************************************************************************/

static  void  Ex_MainSliderInputTask (void  *p_arg)
{
    RTOS_ERR  err;
    int msg;
    uint8_t slider_pos;


    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif
                                                                /* ... will register all the hardware controller to ... */
                                                                /* ... the platform manager at this moment.             */
    slider_setup();
    while (DEF_ON) {
    	// pend on the semaphore that is to be posted by the OSTimer counter handler
        OSSemPend (&sem, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);
        // retrieve the slider status
        slider_position(&slider_pos);
        // determine the message to be sent
        if (slider_pos == INACTIVE) {
        	msg = sld_none;
        }
        else if (slider_pos == LEFT) {
        	msg = sld0;
        }
        else {
        	msg = sld1;
        }
        // send message to the queue
        OSQPost ((OS_Q *) &queue, &msg, (OS_MSG_SIZE) sizeof(msg), (OS_OPT) OS_OPT_POST_FIFO, (RTOS_ERR *) &err);
        /* Delay for 100 ms                      */
		OSTimeDly( 10, OS_OPT_TIME_DLY, &err);
    	/*   Check error code.                                  */
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}


/***************************************************************************//**
 * @brief
 *   Drives the LEDs based on the pushbutton and slider values.
 *
 * @details
 * 	 This task is run as often as is specified by the OS timer tick value in the OSTimeDly
 * 	 function.
 *
 *
 ******************************************************************************/

static  void  Ex_MainLedOutputTask (void  *p_arg)
{
    RTOS_ERR  err;
    int *msg;
    bool PB0_status, PB1_status;
    uint8_t slider_pos;


    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif
                                                                /* ... the platform manager at this moment.             */
    gpio_open();
    while (DEF_ON) {

    	// Receive the message from the head of the queue
    	msg = OSQPend ((OS_Q * ) &queue,
			(OS_TICK)      0,
			(OS_OPT)       OS_OPT_PEND_BLOCKING,
			(OS_MSG_SIZE *)&msg_size,
			(CPU_TS *)     NULL,
			(RTOS_ERR *)   &err);

    	// update peripheral statuses locally depending on the message received
    	if (*msg == btn_none) {
    		PB0_status = 0;
    		PB1_status = 0;
    	}
    	else if (*msg == btn0) {
    		PB0_status = 1;
    		PB1_status = 0;
    	}
    	else if (*msg == btn1) {
    		PB0_status = 0;
    		PB1_status = 1;
    	}
    	else if (*msg == sld_none) {
    		slider_pos = INACTIVE;
    	}
    	else if (*msg == sld0) {
    		slider_pos = LEFT;
    	}
    	else if (*msg == sld1) {
    		slider_pos = RIGHT;
    	}
    	// drive the LEDs based on the local statuses
    	led_drive(PB0_status, PB1_status, slider_pos);
        /* Delay Start Task execution for  */
		OSTimeDly( 10,
					OS_OPT_TIME_DLY,
					&err);
		/*   Check error code.                                  */
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}


/***************************************************************************//**
 * @brief
 *   Callback function for kernel timer.
 *
 * @details
 * 	 Posts on a semaphore shared by the slider input task. This function is not
 * 	 called until the OSTimer reaches a 10 ms timeout. This function is called
 * 	 periodically every 10 ms to signal the slider input task to execute.
 *
 * @note
 *   This function is called every time the OSTimer counts 10 ms.
 *
 ******************************************************************************/
static void MyCallback (OS_TMR p_tmr, void *p_arg) {
	RTOS_ERR  err;
	OSSemPost (&sem, OS_OPT_POST_ALL, &err);
}

/***************************************************************************//**
 * @brief
 *   Drives the LEDs based on the pushbutton and slider values.
 *
 * @details
 * 	 Utilizes global variables slider_pos, PB0_status, and PB1_status.
 * 	 Pushbutton operate on an XOR basis, and slider operates similarly.
 * 	 Pushbuttons and slider operate on OR basis.
 *
 * @note
 *   This function is called every time the LEDs should be set.
 *
 ******************************************************************************/
static void led_drive(bool PB0_status, bool PB1_status, uint8_t slider_pos) {
	// First determine the states of the pushbuttons combined as XOR
	bool button_activity = false;	// init button_activity to false
	if (PB0_status ^ PB1_status)
		button_activity = true;		// if the buttons are not both pressed, then set to true

	// When buttons are XOR false and the slider is either pressed on left and right or unpressed, turn off LEDs
	if (!button_activity && (slider_pos == INACTIVE)) {
		GPIO_PinOutClear(LED0_port, LED0_pin);
		GPIO_PinOutClear(LED1_port, LED1_pin);
	}
	// When buttons are XOR true and slider is LEFT
	else if (button_activity && (slider_pos == LEFT)) {
		// If pushbutton (PB0) pressed, turn on only LED0
		if (PB0_status) {
			GPIO_PinOutSet(LED0_port, LED0_pin);
			GPIO_PinOutClear(LED1_port, LED1_pin);
		}
		// if PB1 pressed, turn on both LED0 and LED1
		else {
			GPIO_PinOutSet(LED0_port, LED0_pin);
			GPIO_PinOutSet(LED1_port, LED1_pin);
		}
	}
	// If buttons are XOR true and slider is RIGHT
	else if (button_activity && (slider_pos == RIGHT)) {
		// If PB0 pressed, turn on both LEDs
		if (PB0_status) {
			GPIO_PinOutSet(LED0_port, LED0_pin);
			GPIO_PinOutSet(LED1_port, LED1_pin);
		}
		// If PB1 is pressed, only turn on LED1
		else {
			GPIO_PinOutSet(LED1_port, LED1_pin);
			GPIO_PinOutClear(LED0_port, LED0_pin);
		}
	}
	// If slider is INACTIVE (because either both sides are pressed or it's not pressed)
	// and buttons are XOR true
	else if (button_activity && (slider_pos == INACTIVE)) {
		// If PB0 is pressed, turn on only LED0
		if (PB0_status) {
			GPIO_PinOutSet(LED0_port, LED0_pin);
			GPIO_PinOutClear(LED1_port, LED1_pin);
		}
		// If PB1 is pressed, turn on LED1
		else {
			GPIO_PinOutSet(LED1_port, LED1_pin);
			GPIO_PinOutClear(LED0_port, LED0_pin);
		}
	}
	// If slider is not inactive and buttons are XOR false
	else if (!button_activity && !(slider_pos == INACTIVE)) {
		// If slider is left, turn on LED0
		if (slider_pos == LEFT) {
			GPIO_PinOutSet(LED0_port, LED0_pin);
			GPIO_PinOutClear(LED1_port, LED1_pin);
		}
		// If slider is right, turn on LED1
		else {
			GPIO_PinOutSet(LED1_port, LED1_pin);
			GPIO_PinOutClear(LED0_port, LED0_pin);
		}
	}
}


/***************************************************************************//**
 * @brief
 *   Interrupt handler for GPIO Even pins module.
 *
 *
 * @details
 * 	 Utilizes global variable PB0_status to poll the value of pushbutton 0
 * 	 using poll_PB0() function.
 *
 * @note
 *   This function is called every time PB0 is pressed.
 *
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
	RTOS_ERR  err;
	__disable_irq();
	/* Get and clear all pending GPIO interrupts */
	uint32_t interruptMask = GPIO_IntGet();
	GPIO_IntClear(interruptMask);
	bool PB0_status;
	poll_PB0(&PB0_status);
	if (PB0_status) {
		OSFlagPost ((OS_FLAG_GRP *) &flg,
			(OS_FLAGS)      0x02,
			(OS_OPT)        OS_OPT_POST_FLAG_CLR,
			(RTOS_ERR *)	&err);
		OSFlagPost ((OS_FLAG_GRP *) &flg,
			(OS_FLAGS)      0x01,
			(OS_OPT)        OS_OPT_POST_FLAG_SET,
			(RTOS_ERR *)	&err);
	}
	else {
		OSFlagPost ((OS_FLAG_GRP *) &flg,
					(OS_FLAGS)      0x01,
					(OS_OPT)        OS_OPT_POST_FLAG_CLR,
					(RTOS_ERR *)	&err);
		OSFlagPost ((OS_FLAG_GRP *) &flg,
					(OS_FLAGS)      0x02,
					(OS_OPT)        OS_OPT_POST_FLAG_SET,
					(RTOS_ERR *)	&err);
	}
	__enable_irq();
}

/***************************************************************************//**
 * @brief
 *   Interrupt handler for GPIO Odd pins module.
 *
 *
 * @details
 * 	 Utilizes global variable PB1_status to poll the value of pushbutton 1
 * 	 using poll_PB1() function.
 *
 * @note
 *   This function is called every time PB1 is pressed.
 *
 ******************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
	RTOS_ERR  err;
	__disable_irq();
	/* Get and clear all pending GPIO interrupts */
	uint32_t interruptMask = GPIO_IntGet();
	GPIO_IntClear(interruptMask);
	bool PB1_status;
	poll_PB1(&PB1_status);
	if (PB1_status) {
		OSFlagPost ((OS_FLAG_GRP *) &flg,
			(OS_FLAGS)      0x08,
			(OS_OPT)        OS_OPT_POST_FLAG_CLR,
			(RTOS_ERR *)	&err);
		OSFlagPost ((OS_FLAG_GRP *) &flg,
			(OS_FLAGS)      0x04,
			(OS_OPT)        OS_OPT_POST_FLAG_SET,
			(RTOS_ERR *)	&err);
	}
	else {
		OSFlagPost ((OS_FLAG_GRP *) &flg,
			(OS_FLAGS)      0x04,
			(OS_OPT)        OS_OPT_POST_FLAG_CLR,
			(RTOS_ERR *)	&err);
		OSFlagPost ((OS_FLAG_GRP *) &flg,
			(OS_FLAGS)      0x08,
			(OS_OPT)        OS_OPT_POST_FLAG_SET,
			(RTOS_ERR *)	&err);
	}

	__enable_irq();
}
