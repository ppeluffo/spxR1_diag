/*
 * spx_tkCtl.c
 *
 *  Created on: 4 de oct. de 2017
 *      Author: pablo
 */

#include "spx.h"

//------------------------------------------------------------------------------------
static void pv_tkCtl_init_system(void);
static void pv_tkCtl_wink_led(void);

// Timpo que espera la tkControl entre round-a-robin
#define TKCTL_DELAY_S	5

//------------------------------------------------------------------------------------
void tkCtl(void * pvParameters)
{

( void ) pvParameters;

	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );

	pv_tkCtl_init_system();

	xprintf_P( PSTR("\r\nstarting tkControl..\r\n\0"));

	for( ;; )
	{

		// Para entrar en tickless.
		// Cada 5s hago un chequeo de todo. En particular esto determina el tiempo
		// entre que activo el switch de la terminal y que esta efectivamente responde.
		vTaskDelay( ( TickType_t)( TKCTL_DELAY_S * 1000 / portTICK_RATE_MS ) );

		pv_tkCtl_wink_led();
		WDT_Reset();
	}
}
//------------------------------------------------------------------------------------
static void pv_tkCtl_init_system(void)
{
	debug_i2c = false;
}
//------------------------------------------------------------------------------------
static void pv_tkCtl_wink_led(void)
{

	// Prendo los leds
	IO_set_LED_KA();
	IO_set_LED_COMMS();

	vTaskDelay( ( TickType_t)( 5 / portTICK_RATE_MS ) );
	//taskYIELD();

	// Apago
	IO_clr_LED_KA();
	IO_clr_LED_COMMS();

}
//------------------------------------------------------------------------------------
