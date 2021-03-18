/**
 * @file app.c
 * @author Karston Christensen
 * @date Feb 3rd, 2021
 * @brief Contains all the application driver functions
 * @details
 * Contains functions to initialize the application code
 *
 */


//***********************************************************************************
// Include files
//***********************************************************************************
#include "app.h"


//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function
//***********************************************************************************


/***************************************************************************//**
 * @brief
 *   Driver to initialize the peripherals by calling peripheral init functions.
 *
 * @details
 * 	 This routine calls all peripheral init functions to initialize all peripherals
 * 	 needed in the main while loop in main.c
 *
 * @note
 *   This function is normally called once to initialize operation of peripherals.
 *
 ******************************************************************************/
void app_peripheral_setup(void){
	cmu_open();
	gpio_open();
	buttons_setup();
	slider_setup();
}

/***************************************************************************//**
 * @brief
 *   Driver to initialize the letimer0 module.
 *
 * @details
 * 	 This routine is a low level driver. It is entirely encapsulated from other drivers.
 * 	 It initializes the letimer0 module.
 *
 * @note
 *   This function is normally called once to initialize operation of letimer0.
 *
 ******************************************************************************/
void app_letimer0_open(void){

}
