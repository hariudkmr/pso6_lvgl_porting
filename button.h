/*
 * button.h
 *
 *  Created on: 25-May-2023
 *      Author: udayakumar
 */

#ifndef BUTTON_H_
#define BUTTON_H_

#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"
#include "cycfg_capsense.h"


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static uint32_t initialize_capsense(void);
static void process_touch(void);
static void initialize_capsense_tuner(void);
static void capsense_isr(void);
static void capsense_callback();
void handle_error(void);
void initialize_button(void);


#endif /* BUTTON_H_ */
