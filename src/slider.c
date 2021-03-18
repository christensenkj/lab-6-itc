/**
 * @file slider.c
 * @author Karston Christensen
 * @date Feb 3rd, 2021
 * @brief Contains all the slider driver functions
 * @details
 * Contains functions to initialize the slider and read its position value.
 *
 */


//***********************************************************************************
// Include files
//***********************************************************************************

#include "slider.h"
#include "capsense.h"


/***************************************************************************//**
 * @brief
 *   Driver to initialize the CAPSENSE slider
 *
 * @details
 * 	 This routine is a low level driver.  It is entirely encapsulated from other drivers.
 * 	 It initializes the CAPSENSE
 * 	 module.
 *
 * @note
 *   This function is normally called once to initialize operation of the CAPSENSE
 *   module.
 *
 ******************************************************************************/
void slider_setup() {
	// Configure the slider
	// Call CAPSENSE library function
	CAPSENSE_Init();
}


/***************************************************************************//**
 * @brief
 *   Driver to obtain CAPSENSE slider value.
 *
 * @details
 * 	 This routine is a low level driver.
 *
 * @note
 *   This function is normally called every time the CAPSENSE slider value is desired.
 *
 * @param[in] position
 *   Pointer to the global variable that stores slider position value.
 *
 ******************************************************************************/
void slider_position(uint8_t *position) {
	CAPSENSE_Sense();
	// initialize slider button values
	uint8_t val0 = 0;
	uint8_t val1 = 0;
	uint8_t val2 = 0;
	uint8_t val3 = 0;
	// initialize logical placeholders
	uint8_t val_left, val_right;
	// set position to inactive state
	*position = INACTIVE;
	// Determine which pads of the slider are pressed
	if (CAPSENSE_getPressed(0)) {
		val0 = 1;
	}
	if (CAPSENSE_getPressed(1)) {
		val1 = 1;
	}
	if (CAPSENSE_getPressed(2)) {
		val2 = 1;
	}
	if (CAPSENSE_getPressed(3)) {
		val3 = 1;
	}
	// find which side of the slider is being pressed
	val_left = val0 || val1;
	val_right = val2 || val3;
	// Set the position based on the contact with the slider
	if (val_left && val_right) {
		*position = INACTIVE;	// If both left and right are touched, treat as inactive
	}
	else if (val_left) {
		*position = LEFT;
	}
	else if (val_right) {
		*position = RIGHT;
	}
	else {
		*position = INACTIVE;
	}
	return;
}

