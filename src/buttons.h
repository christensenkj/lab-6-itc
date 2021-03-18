#ifndef SRC_HEADER_FILES_BUTTONS_H_
#define SRC_HEADER_FILES_BUTTONS_H_

/**
 * @file buttons.h
 * @author Karston Christensen
 * @date February 3, 2021
 * @brief Contains all the pushbutton driver functions prototypes and defns for critical information.
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#include "bspconfig.h"
#include "em_gpio.h"
#include "bsp.h"


//***********************************************************************************
// function prototypes
//***********************************************************************************
void buttons_setup(void);
void poll_PB0(bool *status);
void poll_PB1(bool *status);

#endif /* SRC_HEADER_FILES_BUTTONS_H_ */
