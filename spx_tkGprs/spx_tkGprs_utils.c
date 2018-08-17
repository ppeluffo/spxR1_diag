/*
 * spx_tkGprs_utils.c
 *
 *  Created on: 25 de oct. de 2017
 *      Author: pablo
 */

#include "spx_tkGprs.h"

//------------------------------------------------------------------------------------
void pub_gprs_modem_pwr_off(void)
{
	// Apago la fuente del modem

	IO_clr_GPRS_SW();	// Es un FET que lo dejo cortado
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	IO_clr_GPRS_PWR();	// Apago la fuente.
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );

}
//------------------------------------------------------------------------------------
void pub_gprs_modem_pwr_on(void)
{
	// Prendo la fuente del modem

	IO_clr_GPRS_SW();	// GPRS=0V, PWR_ON pullup 1.8V )
	IO_set_GPRS_PWR();											// Prendo la fuente ( alimento al modem ) HW
	vTaskDelay( (portTickType)( 2000 / portTICK_RATE_MS ) );	// Espero 2s que se estabilize la fuente.


}
//------------------------------------------------------------------------------------
void pub_gprs_modem_pwr_sw(void)
{
	// Genera un pulso en la linea PWR_SW. Como tiene un FET la senal se invierte.
	// En reposo debe la linea estar en 0 para que el fet flote y por un pull-up del modem
	// la entrada PWR_SW esta en 1.
	// El PWR_ON se pulsa a 0 saturando el fet.
	IO_set_GPRS_SW();	// GPRS_SW = 3V, PWR_ON = 0V.
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	IO_clr_GPRS_SW();	// GPRS_SW = 0V, PWR_ON = pullup, 1.8V

}
//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS DEL BUFFER DE RECEPCION DEL GPRS
//------------------------------------------------------------------------------------
void pv_gprs_rxbuffer_flush(void)
{
	memset( pv_gprsRxCbuffer.buffer, '\0', UART_GPRS_RXBUFFER_LEN);
	pv_gprsRxCbuffer.ptr = 0;
}
//------------------------------------------------------------------------------------
void pv_gprs_rxbuffer_poke(char c)
{

	// Si hay lugar meto el dato.
	if ( pv_gprsRxCbuffer.ptr < UART_GPRS_RXBUFFER_LEN )
		pv_gprsRxCbuffer.buffer[ pv_gprsRxCbuffer.ptr++ ] = c;
}
//------------------------------------------------------------------------------------
void pub_gprs_print_RX_response(void)
{
	// Imprime la respuesta del server.
	// Utiliza el buffer de RX.
	// Solo muestra el payload, es decir lo que esta entre <h1> y </h1>
	// Todas las respuestas el server las encierra entre ambos tags excepto los errores del apache.

	char *start_tag, *end_tag;

	start_tag = strstr(pv_gprsRxCbuffer.buffer,"<h1>");
	end_tag = strstr(pv_gprsRxCbuffer.buffer, "</h1>");

	if ( ( start_tag != NULL ) && ( end_tag != NULL) ) {
		*end_tag = '\0';	// Para que frene el xprintf_P
		start_tag += 4;
		//xprintf_P ( PSTR("GPRS: rsp>%s\r\n\0"), start_tag );
		xprintf_P( PSTR ("GPRS: rxbuff>\r\n\0"));
		xnprint( start_tag, sizeof(pv_gprsRxCbuffer.buffer) );
		xprintf_P( PSTR ("\r\n[%d]\r\n\0"), pv_gprsRxCbuffer.ptr );
	}
}
//------------------------------------------------------------------------------------
void pub_gprs_print_RX_Buffer(void)
{

	// Imprimo todo el buffer local de RX. Sale por \0.
	xprintf_P( PSTR ("GPRS: rxbuff>\r\n\0"));

	// Uso esta funcion para imprimir un buffer largo, mayor al que utiliza xprintf_P. !!!
	xnprint( pv_gprsRxCbuffer.buffer, sizeof(pv_gprsRxCbuffer.buffer) );
	xprintf_P( PSTR ("\r\n[%d]\r\n\0"), pv_gprsRxCbuffer.ptr );
}
//------------------------------------------------------------------------------------
void pub_gprs_flush_RX_buffer(void)
{

	frtos_ioctl( fdGPRS,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	pv_gprs_rxbuffer_flush();

}
//------------------------------------------------------------------------------------
void pub_gprs_flush_TX_buffer(void)
{
	frtos_ioctl( fdGPRS,ioctl_UART_CLEAR_TX_BUFFER, NULL);

}
//------------------------------------------------------------------------------------
