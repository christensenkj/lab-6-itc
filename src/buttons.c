/**
 * @file buttons.c
 * @author Karston Christensen
 * @date Feb 3rd, 2021
 * @brief Contains all the button driver functions
 * @details
 * Contains functions to initialize the buttons
 *
 */


//***********************************************************************************
// Include files
//***********************************************************************************
#include "buttons.h"

/***************************************************************************//**
 * @brief
 *   Driver to initialize the buttons.
 *
 * @details
 * 	 This routine is a low level driver.  It is entirely encapsulated from other drivers.
 * 	 It initializes the buttons.
 *
 * @note
 *   This function is normally called once to initialize operation of the buttons.
 *
 ******************************************************************************/
void buttons_setup() {
	// Configure PB0 as input and configure interrupt
	GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeInputPull, 1);
	GPIO_IntConfig(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, true, true, true);

	// Configure PB1 as input and configure interrupt
	GPIO_PinModeSet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, gpioModeInputPull, 1);
	GPIO_IntConfig(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, true, true, true);

	// Clear interrupts on even ports and enable interrupts
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);

	// Clear interrupts on odd ports and enable interrupts
	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
}

/***************************************************************************//**
 * @brief
 *   Driver to poll pushbutton 0.
 *
 * @details
 * 	 This routine is a low level driver.  It is entirely encapsulated from other drivers.
 * 	 It obtains the value of pushbutton 0.
 *
 * @note
 *   This function is called each time a pushbutton value is required.
 *
 * @param[in] status
 *   Pointer to the global variable that stores pushbutton 0 value. Passed by reference.
 *
 ******************************************************************************/
void poll_PB0(bool *status) {
	// get the value of pushbutton 0
	unsigned int val_pb0 = GPIO_PinInGet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN);
	// update the global variable accordingly
	if (val_pb0) {
		*status = false;
	}
	else {
		*status = true;
	}
}

/***************************************************************************//**
 * @brief
 *   Driver to poll pushbutton 1.
 *
 * @details
 * 	 This routine is a low level driver.  It is entirely encapsulated from other drivers.
 * 	 It obtains the value of pushbutton 1.
 *
 * @note
 *   This function is called each time a pushbutton value is required.
 *
 * @param[in] status
 *   Pointer to the global variable that stores pushbutton 1 value. Passed by reference.
 *
 ******************************************************************************/
void poll_PB1(bool *status) {
	// get the value of PB1
	unsigned int val_pb1 = GPIO_PinInGet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN);
	// update the global variable accordingly
	if (val_pb1) {
		*status = false;
	}
	else {
		*status = true;
	}
}
