/*
 * sp5KV5_tkGprs_main.c
 *
 *  Created on: 26 de abr. de 2017
 *      Author: pablo
 *
 *
 *  Comandos AT para conectarse.
 *
 *  1) Chequeamos que halla un SIM presente.
 *  AT+CPIN?
 *  +CPIN: READY
 *
 *  2) Registacion.
 *  El dispositivo debe estar registrado en la red antes de poder usarla.
 *  Para chequear si esta registrado usamos
 *  AT+CGREG?
 *  +CGREG: 0,1
 *
 *  Normalmente el SIM esta para que el dispositivo se registre automaticamente
 *  Esto se puede ver con el comando AT+COPS? el cual tiene la red preferida y el modo 0
 *  indica que esta para registrarse automaticamente.
 *  Este comando se usa para de-registrar y registrar en la red.
 *  Conviene dejarlo automatico de modo que si el TE se puede registrar lo va a hacer.
 *  Solo chequeamos que este registrado con CGREG.
 *
 *  3) Si esta registrado, podemos obtener informacion de la red ( opcional )
 *  AT+COPS?
 *
 *  4) Una vez registrado consultamos la calidad de senal
 *  AT+CSQ
 *  +CSQ 10,99
 *
 *  5) Si estoy registrado, me debo atachear a la red
 *  AT+CGATT=1
 *
 *  6) APN
 * 	Incluso si GPRS Attach es exitoso, no significa que se haya establecido la llamada de datos.
 * 	En GPRS, un Protocolo de paquetes de datos (PDP) define la sesión de datos.
 * 	El contexto PDP establece la ruta de datos entre el dispositivo y el GGSN (Nodo de soporte Gateway GPRS).
 * 	GGSN actúa como una puerta de enlace entre el dispositivo y el resto del mundo.
 * 	Por lo tanto, debe establecer un contexto PDP antes de que pueda enviar / recibir datos en Internet.
 * 	El GGSN se identifica a través del nombre del punto de acceso (APN).
 * 	Cada proveedor tendrá sus propias APN y generalmente están disponibles en Internet.
 * 	El dispositivo puede definir múltiples contextos PDP que se almacenan de manera única
 * 	en los ID de contexto.
 *
 * 	7) Ahora que los contextos PDP están definidos, utilizo el contexto PDP correcto que coincida
 *  con la tarjeta SIM.
 *  Ahora para configurar la sesión, el contexto PDP apropiado debe estar activado.
 *  AT + CGACT = 1,1
 *  El primer parámetro (1) es activar el contexto y el segundo parámetro es el ID de contexto.
 *  Una vez que el Contexto PDP se activa con éxito, el dispositivo puede enviar / recibir
 *  datos en Internet.
 *  Para verificar si hubo algún problema con la activación PDP, envíe el comando
 *  AT + CEER.
 *
 *
 */

#include "spx_tkGprs.h"

void tkGprsRx(void * pvParameters)
{
	// Esta tarea lee y procesa las respuestas del GPRS. Lee c/caracter recibido y lo va
	// metiendo en un buffer circular propio del GPRS que permite luego su impresion,
	// analisis, etc.

( void ) pvParameters;
char c;

	// Espero la notificacion para arrancar
	vTaskDelay( ( TickType_t)( 200 / portTICK_RATE_MS ) );

	pv_gprs_rxbuffer_flush();

	xprintf_P( PSTR("starting tkGprsRX..\r\n\0"));

	// loop
	for( ;; )
	{
		while ( frtos_read( fdGPRS, &c, 1 ) == 1 ) {
			pv_gprs_rxbuffer_poke(c);
		}
	}
}
//------------------------------------------------------------------------------------
