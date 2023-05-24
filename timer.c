/*
 * timer.c
 *
 *  Created on: 23-May-2023
 *      Author: udayakumar
 */


#include "timer.h"
#include "lv_hal_tick.h"

 cy_stc_sysint_t Timer_IRQ_config = {
         .intrSrc = tcpwm_0_interrupts_5_IRQn,
         .intrPriority = 7u
 };

#define MY_TCPWM_PWM_NUM   5
#define MY_TCPWM_PWM_NUM_MASK  (1UL << MY_TCPWM_PWM_NUM)

 void timer_handler()
 {
	 uint32_t interrupts = Cy_TCPWM_GetInterruptStatusMasked(TCPWM0, MY_TCPWM_PWM_NUM);

	 //Cy_GPIO_Inv(P1_4_PORT, P1_4_NUM);
	 lv_tick_inc(1);

	 Cy_TCPWM_ClearInterrupt(TCPWM0, MY_TCPWM_PWM_NUM, interrupts);
 }

 void timer_1ms_init()
 {

	if (CY_TCPWM_SUCCESS != Cy_TCPWM_PWM_Init(TCPWM0, MY_TCPWM_PWM_NUM, &tcpwm_0_cnt_5_config))
	{
		/* Handle possible errors */
	}
	/* Then start the counter */
	Cy_SysInt_Init(&Timer_IRQ_config, timer_handler);
	/* Enable the interrupt */
	NVIC_EnableIRQ(Timer_IRQ_config.intrSrc);

	/* Enable the initialized counter */
	Cy_TCPWM_PWM_Enable(TCPWM0, MY_TCPWM_PWM_NUM);

	Cy_TCPWM_TriggerStart_Single (TCPWM0, MY_TCPWM_PWM_NUM);

 }
