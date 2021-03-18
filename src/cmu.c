/**
 * @file cmu.c
 * @author Karston Christensen
 * @date Feb 3rd, 2021
 * @brief Contains all the cmu driver functions
 * @details
 * Contains functions to initialize the cmu
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#include "cmu.h"

//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************

/***************************************************************************//**
 * @brief
 *   Driver to initialize the clock management unit.
 *
 * @details
 * 	 This routine is a low level driver. It is entirely encapsulated from other drivers.
 * 	 It initializes the cmu.
 *
 * @note
 *   This function is normally called once to initialize operation of the cmu.
 *
 ******************************************************************************/
void cmu_open(void){

		//uint32_t	temp_freq;

//		CMU_ClockSelectSet(cmuClock_HFPER, cmuSelect_HFRCO);
//		CMU_ClockEnable(cmuClock_HFPER, true);

//		// By default, LFRCO is enabled
//		CMU_OscillatorEnable(cmuOsc_LFRCO, false, false);	// using LFXO or ULFRCO
//
//		// Route LF clock to the LF clock tree
//
//		CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true);
//		CMU_OscillatorEnable(cmuOsc_LFXO, false, false);		// Disable LFXO
//		CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);	// routing clock to LFA
//
//
//		CMU_ClockEnable(cmuClock_LFA, true);
//		CMU_ClockEnable(cmuClock_CORELE, true);

		// Peripheral clocks enabled
		CMU_ClockEnable(cmuClock_GPIO, true);

}

