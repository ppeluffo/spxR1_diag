/*
 * xmega01_utils.c
 *
 *  Created on: 1 de nov. de 2016
 *      Author: pablo
 */

#include "spxR1_diag.h"

#define RTC32_ToscBusy()        !( VBAT.STATUS & VBAT_XOSCRDY_bm )
void RTC32_ToscEnable( bool use1khz );

//-----------------------------------------------------------
void initMCU(void)
{
	// Inicializa los pines del micro

	// LEDS:
	IO_config_LED_KA();
	IO_config_LED_COMMS();

	// ANALOG: SENSOR VCC CONTROL
	IO_config_SENS_12V_CTL();

	// TERMINAL CTL PIN
	IO_config_TERMCTL_PIN();

	// INPUTS DIGITAL
	IO_config_PA0();
	IO_config_PA2();
	IO_config_PB2();
	IO_config_PB7();
	IO_config_CLRD();

	// OUT8814
	OUT_config();

	// GPRS
	IO_config_GPRS_SW();
	IO_config_GPRS_PWR();
	IO_config_GPRS_RTS();
	IO_config_GPRS_CTS();
	IO_config_GPRS_DCD();
	IO_config_GPRS_RI();
	IO_config_GPRS_RX();
	IO_config_GPRS_TX();

	// BLUETOOTH POWER CTL
	IO_config_BT_PWR_CTL();

	// INTERFASE DE PULSOS ( sensor de ultrasonido )
	IO_config_UPULSE_RUN();
	IO_config_UPULSE_WIDTH();

	// TICK:
	//IO_config_TICK();

	// PWR_SLEEP
	IO_config_PWR_SLEEP();
//	IO_set_PWR_SLEEP();

	// XBEE
	IO_config_XBEE_PWR();
	IO_config_XBEE_SLEEP();
	IO_config_XBEE_RESET();

}
//-----------------------------------------------------------
void u_configure_systemMainClock(void)
{
/*	Configura el clock principal del sistema
	Inicialmente se arranca en 2Mhz.
	La configuracion del reloj tiene 2 componentes: el clock y el oscilador.
	OSCILADOR:
	Primero debo elejir cual oscilador voy a usar para alimentar los prescalers que me den
	el clock del sistema y esperar a que este este estable.
	CLOCK:
	Elijo cual oscilador ( puedo tener mas de uno prendido ) va a ser la fuente principal
	del closck del sistema.
	Luego configuro el prescaler para derivar los clocks de los perifericos.
	Puedo por ultimo 'lockear' esta configuracion para que no se cambie por error.
	Los registros para configurar el clock son 'protegidos' por lo que los cambio
	utilizando la funcion CCPwrite.

	Para nuestra aplicacion vamos a usar un clock de 32Mhz.
	Como vamos a usar el ADC debemos prestar atencion al clock de perifericos clk_per ya que luego
	el ADC clock derivado del clk_per debe estar entre 100khz y 1.4Mhz ( AVR1300 ).

	Opcionalmente podriamos deshabilitar el oscilador de 2Mhz para ahorrar energia.
*/

#if SYSMAINCLK == 32
	// Habilito el oscilador de 32Mhz
	OSC.CTRL |= OSC_RC32MEN_bm;

	// Espero que este estable
	do {} while ( (OSC.STATUS & OSC_RC32MRDY_bm) == 0 );

	// Seteo el clock para que use el oscilador de 32Mhz.
	// Uso la funcion CCPWrite porque hay que se cuidadoso al tocar estos
	// registros que son protegidos.
	CCPWrite(&CLK.CTRL, CLK_SCLKSEL_RC32M_gc);
	//
	// El prescaler A ( CLK.PSCCTRL ), B y C ( PSBCDIV ) los dejo en 0 de modo que no
	// hago division y con esto tengo un clk_per = clk_sys. ( 32 Mhz ).
	//
#endif

#if SYSMAINCLK == 8
	// Habilito el oscilador de 32Mhz y lo divido por 4
	OSC.CTRL |= OSC_RC32MEN_bm;

	// Espero que este estable
	do {} while ( (OSC.STATUS & OSC_RC32MRDY_bm) == 0 );

	// Seteo el clock para que use el oscilador de 32Mhz.
	// Uso la funcion CCPWrite porque hay que se cuidadoso al tocar estos
	// registros que son protegidos.
	CCPWrite(&CLK.CTRL, CLK_SCLKSEL_RC32M_gc);
	//
	// Pongo el prescaler A por 4 y el B y C en 0.
	CLKSYS_Prescalers_Config( CLK_PSADIV_4_gc, CLK_PSBCDIV_1_1_gc );

	//
#endif

#if SYSMAINCLK == 2
	// Este es el oscilador por defecto por lo cual no tendria porque configurarlo.
	// Habilito el oscilador de 2Mhz
	OSC.CTRL |= OSC_RC2MEN_bm;
	// Espero que este estable
	do {} while ( (OSC.STATUS & OSC_RC2MRDY_bm) == 0 );

	// Seteo el clock para que use el oscilador de 2Mhz.
	// Uso la funcion CCPWrite porque hay que se cuidadoso al tocar estos
	// registros que son protegidos.
	CCPWrite(&CLK.CTRL, CLK_SCLKSEL_RC2M_gc);
	//
	// El prescaler A ( CLK.PSCCTRL ), B y C ( PSBCDIV ) los dejo en 0 de modo que no
	// hago division y con esto tengo un clk_per = clk_sys. ( 2 Mhz ).
	//
#endif

//#ifdef configUSE_TICKLESS_IDLE
	// Para el modo TICKLESS
	// Configuro el RTC con el osc externo de 32Khz
	// Pongo como fuente el xtal externo de 32768 contando a 32Khz.
	//CLK.RTCCTRL = CLK_RTCSRC_TOSC32_gc | CLK_RTCEN_bm;
	//do {} while ( ( RTC.STATUS & RTC_SYNCBUSY_bm ) );

	// Disable RTC interrupt.
	// RTC.INTCTRL = 0x00;
	//
	// Si uso el RTC32, habilito el oscilador para 1ms.

	RTC32_ToscEnable(true);
//#endif

	// Lockeo la configuracion.
	CCPWrite( &CLK.LOCK, CLK_LOCK_bm );

}
//-----------------------------------------------------------
void RTC32_ToscEnable( bool use1khz )
{
	/* Enable 32 kHz XTAL oscillator, with 1 kHz or 1 Hz output. */
	if (use1khz)
		VBAT.CTRL |= ( VBAT_XOSCEN_bm | VBAT_XOSCSEL_bm );
	else
		VBAT.CTRL |= ( VBAT_XOSCEN_bm );

	RTC32.PER = 10;
	RTC32.CNT = 0;

	/* Wait for oscillator to stabilize before returning. */
//	do { } while ( RTC32_ToscBusy() );
}
//-----------------------------------------------------------
void u_configure_RTC32(void)
{
	// El RTC32 lo utilizo para desperarme en el modo tickless.
	// V-bat needs to be reset, and activated
	VBAT.CTRL |= VBAT_ACCEN_bm;
	// Este registro esta protegido de escritura con CCP.
	CCPWrite(&VBAT.CTRL, VBAT_RESET_bm);

	// Pongo el reloj en 1.024Khz.
	VBAT.CTRL |=  VBAT_XOSCSEL_bm | VBAT_XOSCFDEN_bm ;

	// wait for 200us see AVR1321 Application note page 8
	_delay_us(200);

	// Turn on 32.768kHz crystal oscillator
	VBAT.CTRL |= VBAT_XOSCEN_bm;

	// Wait for stable oscillator
	while(!(VBAT.STATUS & VBAT_XOSCRDY_bm));

	// Disable RTC32 module before setting counter values
	RTC32.CTRL = 0;

	// Wait for sync
	do { } while ( RTC32.SYNCCTRL & RTC32_SYNCBUSY_bm );

	// EL RTC corre a 1024 hz y quiero generar un tick de 10ms,
	RTC32.PER = 1024;
	RTC32.CNT = 0;

	// Interrupt: on Overflow
	RTC32.INTCTRL = RTC32_OVFINTLVL_LO_gc;

	// Enable RTC32 module
	RTC32.CTRL = RTC32_ENABLE_bm;

	/* Wait for sync */
	do { } while ( RTC32.SYNCCTRL & RTC32_SYNCBUSY_bm );
}
//-----------------------------------------------------------
void pub_analog_read_battery ( float *mag_val )
{

uint16_t an_raw_val;
float an_mag_val;

	// La bateria esta en el canal 1 (bus) del INA 1.
	an_raw_val = ACH_read_battery();

	// Convierto el raw_value a la magnitud ( 8mV por count del A/D)
	an_mag_val = 0.008 * an_raw_val;
	*mag_val = an_mag_val;

}
//------------------------------------------------------------------------------------
void pub_analog_read_channel ( uint8_t channel, uint16_t *raw_val )
{

	// Lee un canal analogico y devuelve en raw_val el valor leido del conversor A/D y en
	// mag_val el valor convertido a la magnitud configurada.
	// Se utiliza desde el modo comando como desde el modulo de poleo de las entradas.

uint16_t an_raw_val;

	an_raw_val = ACH_read_channel(channel);
	*raw_val = an_raw_val;

}
//------------------------------------------------------------------------------------
void pub_output_set_consigna_diurna(void)
{
	// En consigna diurna la valvula A (JP28) queda abierta y la valvula B (JP2) cerrada.
	//
	// Proporciono corriente.
	OUT_power_on();
	// Espero 10s que se carguen los condensasores
	vTaskDelay( ( TickType_t)( 10000 / portTICK_RATE_MS ) );

	OUT_valve( 'A', V_OPEN, 100 );
	vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
	OUT_valve( 'B', V_CLOSE, 100 );

	OUT_power_off();

	xprintf_P( PSTR("OUTPUTS: Aplico Consigna Diurna\r\n\0") );
}
//----------------------------------------------------------------------------------------
void pub_output_set_consigna_nocturna(void)
{

	// Proporciono corriente.
	OUT_power_on();
	// Espero 10s que se carguen los condensasores
	vTaskDelay( ( TickType_t)( 10000 / portTICK_RATE_MS ) );

	OUT_valve( 'A', V_CLOSE, 100 );
	vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
	OUT_valve( 'B', V_OPEN, 100 );

	OUT_power_off();

	xprintf_P( PSTR("OUTPUTS: Aplico Consigna Nocturna\r\n\0") );
}
//----------------------------------------------------------------------------------------

