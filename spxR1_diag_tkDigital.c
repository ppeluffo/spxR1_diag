/*
 * spxR1_diag_tkCounters.c
 *
 *  Created on: 23 ago. 2018
 *      Author: pablo
  *
 *          Entradas	 | Nivel o contador   |    Contador
 * ----------------------|--------------------|----------------
 * JP31,JP30   JP20,JP21 |	D3 (PA0),D0 (PB7) |	D2(PA2),D1(PB2)
 *   1	        1	            X	               X
 *   1	        3	        JP13,JP14	       JP16,JP15
 *   3	        1	        JP16,JP15	       JP13,JP14
 *   3	        3	            X	               X
 *
 * El micro en las líneas 2 ( PA2) es donde puede generar interrupciones
 * que lo despiertan del modo sleep, por lo tanto son solo esas líneas que
 * utilizamos para generar las interrupciones y funcionar como contadores.
 * En el firmware, el funcionamiento que implementamos es:
 * Leemos el nivel lógico de las 4 entradas (D0,D1,D2,D3) en cada ciclo
 * al leer un frame.
 * En D1(PB2) y D2(PA2) activamos las interrupciones por flanco que cuentan
 * pulsos y representan a los contadores CNT0 y CNT1.
 *
 * En el datalogger SPX tenemos 2 grupos de entradas digitales.
 * Las contabilizamos de acuerdo al orden de los conectores externos para que
 * las misma sea coherente desde el punto de vista del tecnico.
 * D0: JP15
 * D1: JP14
 * D2: JP13
 * D3: JP16.
 *
 * Los 2 grupos son: DO,D1 y D2,D3.
 *
 * Grupo 1: D0(JP15) y D1(JP14).
 * Pueden configurarse para que sean ambos medidas de nivel o para que D0 mida nivel y D1 contador.
 * - Configuracion A: D0 level, D1 pulsos
 *   D0: JP15->jp30(3,2) : PB7 (level)
 *   D1: JP14->jp21(1,2) : PB2 (pulsos interrupcion)
 *
 * - Configuracion B: D0 level, D1 level
 *   D0: JP15->jp21(3,2) : PB2 (level)
 *   D1: JP14->jp30(1,2) : PB7 (level)
 *
 * Grupo 2: D2(JP13) y D3(JP16).
 * Pueden configurarse para que sean ambos medidas de nivel o para que D2 mida nivel y D3 contador.
 * - Configuracion C: D2 level, D3 level
 *   D2: JP13->jp31(1,2) : PA0 (level)
 *   D3: JP16->jp20(3,2) : PA2 (level)
 *
 * - Configuracion D: D2 pulsos, D2 level
 *   D2: JP13->jp20(1,2) : PA2 (pulsos interrupcion)
 *   D3: JP16->jp31(3,2) : PA0 (level)
 *
 */

#include "spxR1_diag.h"

BaseType_t xHigherPriorityTaskWokenDigital = pdFALSE;

static st_digital_frame digital_frame;

static bool wakeup_for_C0, wakeup_for_C1;

static void pv_tkDigital_init(void);

//------------------------------------------------------------------------------------
void tkDigital(void * pvParameters)
{

( void ) pvParameters;
uint32_t ulNotificationValue;
const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 10000 );

	vTaskDelay( ( TickType_t)( 200 / portTICK_RATE_MS ) );

	pv_tkDigital_init();

	xprintf_P( PSTR("starting tkDigital..\r\n\0"));

	// loop
	for( ;; )
	{

		// Cuando la interrupcion detecta un flanco, solo envia una notificacion
		// Espero que me avisen. Si no me avisaron en 10s salgo y repito el ciclo.
		// Esto es lo que me permite entrar en tickless.
		ulNotificationValue = ulTaskNotifyTake( pdFALSE, xMaxBlockTime );

		if( ulNotificationValue != 0 ) {
			// Fui notificado: llego algun flanco que determino.

			if ( wakeup_for_C0 ) {
				// El contador C0 solo puede estar en D1
				digital_frame.counter[1]++;
				wakeup_for_C0 = false;
			}

			if ( wakeup_for_C1 ) {
				// El contador C1 solo puede estar en D2
				digital_frame.counter[2]++;
				wakeup_for_C1 = false;
			}

			xprintf_P( PSTR("DIGITAL: C0=%d,C1=%d\r\n\0"),digital_frame.counter[1],digital_frame.counter[2]);

			// Espero 100ms de debounced
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

			IO_clr_CLRD();		// Borro el latch llevandolo a 0.
			IO_set_CLRD();		// Lo dejo en reposo en 1

		} else   {
			// Expiro el timeout de la tarea. Por ahora no hago nada.
		}
	}
}
//------------------------------------------------------------------------------------
static void pv_tkDigital_init(void)
{
	// Configuracion inicial de la tarea

uint8_t counter;

	// Configuracion de las interrupciones que genera el contador
	// PA2, PB2.
	// Los pines ya estan configurados como entradas.
	//
	PORTA.PIN2CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_RISING_gc;	// Sensa rising edge
	PORTA.INT0MASK = PIN2_bm;
	PORTA.INTCTRL = PORT_INT0LVL0_bm;

	PORTB.PIN2CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_RISING_gc;	// Sensa rising edge
	PORTB.INT0MASK = PIN2_bm;
	PORTB.INTCTRL = PORT_INT0LVL0_bm;

	wakeup_for_C0 = false;
	wakeup_for_C1 = false;

	for ( counter = 0; counter < NRO_DIGITAL_CHANNELS; counter++) {
		digital_frame.counter[ counter ] = 0;
	}

	IO_clr_CLRD();	// Borro el latch llevandolo a 0.
	IO_set_CLRD();	// Lo dejo en reposo en 1

}
//------------------------------------------------------------------------------------
ISR(PORTA_INT0_vect)
{
	// Esta ISR se activa cuando el contador D2 (PA2) genera un flaco se subida.
	// Solo avisa a la tarea principal ( que esta dormida ) que se levante y cuente
	// el pulso y haga el debounced.
	// Dado que los ISR de los 2 contadores son los que despiertan a la tarea, debo
	// indicarle de donde proviene el llamado
	wakeup_for_C1 = true;
	vTaskNotifyGiveFromISR( xHandle_tkDigital , &xHigherPriorityTaskWokenDigital );
	PORTA.INTFLAGS = PORT_INT0IF_bm;
}
//------------------------------------------------------------------------------------
ISR(PORTB_INT0_vect)
{
	// Esta ISR se activa cuando el contador D1 (PB2) genera un flaco se subida.
	// Solo avisa a la tarea principal ( que esta dormida ) que se levante y cuente
	// el pulso y haga el debounced.
	// Dado que los ISR de los 2 contadores son los que despiertan a la tarea, debo
	// indicarle de donde proviene el llamado
	wakeup_for_C0 = true;
	vTaskNotifyGiveFromISR( xHandle_tkDigital , &xHigherPriorityTaskWokenDigital );
	PORTB.INTFLAGS = PORT_INT0IF_bm;
}
//------------------------------------------------------------------------------------
