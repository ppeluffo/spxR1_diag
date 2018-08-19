/*
 * sp5K_tkCmd.c
 *
 *  Created on: 27/12/2013
 *      Author: root
 */

#include "FRTOS-CMD.h"
#include "spx_tkGprs/spx_tkGprs.h"

//----------------------------------------------------------------------------------------
// FUNCIONES DE USO PRIVADO
//----------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void );
static void pv_snprintfP_ERR(void);

//----------------------------------------------------------------------------------------
// FUNCIONES DE CMDMODE
//----------------------------------------------------------------------------------------
static void cmdHelpFunction(void);
static void cmdClearScreen(void);
static void cmdResetFunction(void);
static void cmdWriteFunction(void);
static void cmdReadFunction(void);

static void pv_cmd_rwEE(uint8_t cmd_mode );
static void pv_cmd_rwRTC(uint8_t cmd_mode );
static void pv_cmd_rwRTC_SRAM(uint8_t cmd_mode );
static void pv_cmd_sens12V(void);
static void pv_cmd_rwGPRS(uint8_t cmd_mode );
static void pv_cmd_rwBT(void);
static void pv_cmd_rwXBEE(uint8_t cmd_mode );
static void pv_cmd_INA(uint8_t cmd_mode );
static void pv_cmd_rwACH(uint8_t cmd_mode );
static void pv_cmd_rdBATTERY(void);
static void pv_cmd_range(void);
static void pv_cmd_wrOUT8814(void);

#define WR_CMD 0
#define RD_CMD 1

#define WDG_CMD_TIMEOUT	60

RtcTimeType_t rtc;

//------------------------------------------------------------------------------------
void tkCmd(void * pvParameters)
{

uint8_t c;
uint8_t ticks;

( void ) pvParameters;

	vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	FRTOS_CMD_init();

	// Registro los comandos y los mapeo al cmd string.
	FRTOS_CMD_register( "cls\0", cmdClearScreen );
	FRTOS_CMD_register( "reset\0", cmdResetFunction);
	FRTOS_CMD_register( "write\0", cmdWriteFunction);
	FRTOS_CMD_register( "read\0", cmdReadFunction);
	FRTOS_CMD_register( "help\0", cmdHelpFunction );

	// Fijo el timeout del READ
	ticks = 5;
	frtos_ioctl( fdUSB,ioctl_SET_TIMEOUT, &ticks );
	//frtos_ioctl( fdBT,ioctl_SET_TIMEOUT, &ticks );

	xprintf_P( PSTR("starting tkCmd..\r\n\0") );

	//FRTOS_CMD_regtest();
	// loop
	for( ;; )
	{

		c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
		// el read se bloquea 50ms. lo que genera la espera.
		//while ( CMD_read( (char *)&c, 1 ) == 1 ) {
		while ( ( frtos_read( fdUSB, (char *)&c, 1 ) == 1 ) || ( frtos_read( fdBT, (char *)&c, 1 ) == 1 ) ) {
			FRTOS_CMD_process(c);
		}
	}
}
//------------------------------------------------------------------------------------
static void cmdResetFunction(void)
{
	// Resetea al micro prendiendo el watchdog

	FRTOS_CMD_makeArgv();

	cmdClearScreen();

	CCPWrite( &RST.CTRL, RST_SWRST_bm );   /* Issue a Software Reset to initilize the CPU */


}
//------------------------------------------------------------------------------------
static void cmdWriteFunction(void)
{

	FRTOS_CMD_makeArgv();

	// RTC
	// write rtc YYMMDDhhmm
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0")) ) {
		pv_cmd_rwRTC(WR_CMD);
		return;
	}

	// EE
	// write ee pos string
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0"))) {
		pv_cmd_rwEE(WR_CMD);
		return;
	}

	// RTC SRAM
	// write rtcram pos string
	if (!strcmp_P( strupr(argv[1]), PSTR("RTCRAM\0"))) {
		pv_cmd_rwRTC_SRAM(WR_CMD);
		return;
	}

	// DEBUG I2C
	// write debugi2c {on|off}
	if (!strcmp_P( strupr(argv[1]), PSTR("DEBUGI2C\0"))) {
		if (!strcmp_P( strupr(argv[2]), PSTR("ON\0"))) {
			debug_i2c = true;
			pv_snprintfP_OK();
			return;
		}
		if (!strcmp_P( strupr(argv[2]), PSTR("OFF\0"))) {
			debug_i2c = false;
			pv_snprintfP_OK();
			return;
		}
		pv_snprintfP_ERR();
		return;
	}

	// SENS12V
	// write sens12V {on|off}
	if (!strcmp_P( strupr(argv[1]), PSTR("SENS12V\0")) ) {
		pv_cmd_sens12V();
		return;
	}

	// GPRS
	// write gprs pwr|sw|rts {on|off}
	// write gprs cmd {atcmd}
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRS\0")) ) {
		pv_cmd_rwGPRS(WR_CMD);
		return;
	}

	// XBEE
	// write xbee pwr {on|off}
	// write xbee msg
	if (!strcmp_P( strupr(argv[1]), PSTR("XBEE\0")) ) {
		pv_cmd_rwXBEE(WR_CMD);
		return;
	}

	// BT
	if (!strcmp_P( strupr(argv[1]), PSTR("BT\0")) ) {
		pv_cmd_rwBT();
		return;
	}

	// INA
	// write ina confReg Value
	// Solo escribimos el registro 0 de configuracion.
	if (!strcmp_P( strupr(argv[1]), PSTR("INA\0"))) {
		pv_cmd_INA(WR_CMD);
		return;
	}

	// ANALOG
	// write analog {ina_id} conf128
	if (!strcmp_P( strupr(argv[1]), PSTR("ANALOG\0"))) {
		pv_cmd_rwACH(WR_CMD);
		return;
	}

	// CLRD
	// write clrd {0|1}
	if (!strcmp_P( strupr(argv[1]), PSTR("CLRD\0"))) {
		if ( atoi( argv[2]) == 0 ) { IO_clr_CLRD(); }
		if ( atoi( argv[2]) == 1 ) { IO_set_CLRD(); }
		return;
	}

	// RANGE
	// write range on|off
	if (!strcmp_P( strupr(argv[1]), PSTR("RANGE\0"))) {
		if (!strcmp_P( strupr(argv[2]), PSTR("ON\0")) ) {
			IO_set_UPULSE_RUN();
			pv_snprintfP_OK();
			return;
		}

		if (!strcmp_P( strupr(argv[2]), PSTR("OFF\0")) ) {
			IO_clr_UPULSE_RUN();
			pv_snprintfP_OK();
			return;
		}

		xprintf_P( PSTR("cmd ERROR: ( write range {on|off} )\r\n\0"));
		return;
		return;
	}

	// OUT 8814
	// write out sleep|reset|phase(A/B)|enable(A/B)| {0|1}
	//       out pulse (A/B) (+/-) (ms)
	//       out power {on|off}
	if (!strcmp_P( strupr(argv[1]), PSTR("OUT\0")) ) {
		pv_cmd_wrOUT8814();
		return;
	}

	// CONSIGNA
	// write consigna {diurna|nocturna}
	if (!strcmp_P( strupr(argv[1]), PSTR("CONSIGNA\0")) ) {

		if (!strcmp_P( strupr(argv[2]), PSTR("DIURNA\0")) ) {
			pub_output_set_consigna_diurna();
			pv_snprintfP_OK();
			return;
		}

		if (!strcmp_P( strupr(argv[2]), PSTR("NOCTURNA\0")) ) {
			pub_output_set_consigna_nocturna();
			pv_snprintfP_OK();
			return;
		}

		pv_snprintfP_ERR();
		return;
	}


	// CMD NOT FOUND
	xprintf_P( PSTR("ERROR\r\nCMD NOT DEFINED\r\n\0"));
	return;
}
//------------------------------------------------------------------------------------
static void cmdReadFunction(void)
{

uint16_t raw_val;

	FRTOS_CMD_makeArgv();

	// RTC
	// read rtc
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0")) ) {
		pv_cmd_rwRTC(RD_CMD);
		return;
	}

	// SIGNATURE
	// read id
	if (!strcmp_P( strupr(argv[1]), PSTR("ID\0"))) {
		xprintf_P( PSTR("uID=%s\r\n\0"), NVMEE_readID() );
		return;
	}

	// EE
	// read ee address length
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0")) ) {
		pv_cmd_rwEE(RD_CMD);
		return;
	}

	// RTC SRAM
	// read rtcram address length
	if (!strcmp_P( strupr(argv[1]), PSTR("RTCRAM\0")) ) {
		pv_cmd_rwRTC_SRAM(RD_CMD);
		return;
	}

	// GPRS
	// read gprs (rsp,cts,dcd,ri)
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRS\0")) ) {
		pv_cmd_rwGPRS(RD_CMD);
		return;
	}

	// INA
	if (!strcmp_P( strupr(argv[1]), PSTR("INA\0"))) {
		pv_cmd_INA(RD_CMD);
		return;
	}

	// BATTERY
	// read battery
	if (!strcmp_P( strupr(argv[1]), PSTR("BATTERY\0")) ) {
		pv_cmd_rdBATTERY();
		return;
	}

	// ACH { 0..5}
	// read ach x
	if (!strcmp_P( strupr(argv[1]), PSTR("ACH\0")) ) {
		if ( atoi(argv[2]) > 5) {
			pv_snprintfP_ERR();
			return;
		}
		pub_analog_read_channel( atoi(argv[2]),&raw_val );
		xprintf_P( PSTR("CH[%02d] raw=%d\r\n\0"),atoi(argv[2]),raw_val );
		return;
	}

	// TERM
	// read term
	if (!strcmp_P( strupr(argv[1]), PSTR("TERM\0")) ) {
		xprintf_P( PSTR("TERM=%d\r\n\0"), IO_read_TERMCTL_PIN() );
		return;
	}

	// DIN
	// read din
	if (!strcmp_P( strupr(argv[1]), PSTR("DIN\0")) ) {
		xprintf_P( PSTR("D0=%d (Nivel: JP15->jp21(3,2) : PB2)\r\n\0"), IO_read_PB2() );
		xprintf_P( PSTR("D1=%d (Nivel: JP14->jp30(1,2) : PB7)\r\n\0"), IO_read_PB7() );
		xprintf_P( PSTR("D2=%d (Nivel: JP13->jp31(1,2) : PA0)\r\n\0"), IO_read_PA0() );
		xprintf_P( PSTR("D3=%d (Nivel: JP16->jp20(3,2) : PA2)\r\n\0"), IO_read_PA2() );
		return;
	}

	// CONTADORES
	// read contadores
	if (!strcmp_P( strupr(argv[1]), PSTR("CONTADORES\0")) ) {
		xprintf_P( PSTR("D0=%d (Nivel:    JP15->jp30(3,2) : PB7)\r\n\0"), IO_read_PB7() );
		xprintf_P( PSTR("D1=%d (Contador: JP14->jp21(1,2) : PB2)\r\n\0"), IO_read_PB2() );
		xprintf_P( PSTR("D2=%d (Contador: JP13->jp20(1,2) : PA2)\r\n\0"), IO_read_PA2() );
		xprintf_P( PSTR("D3=%d (Nivel:    JP16->jp31(3,2) : PA0)\r\n\0"), IO_read_PA0() );
		return;
	}

	// RANGE
	// read range
	if (!strcmp_P( strupr(argv[1]), PSTR("RANGE\0"))) {
		pv_cmd_range();
		return;
	}

	// CMD NOT FOUND
	xprintf_P( PSTR("ERROR\r\nCMD NOT DEFINED\r\n\0"));
	return;

}
//------------------------------------------------------------------------------------
static void cmdClearScreen(void)
{
	// ESC [ 2 J
	xprintf_P( PSTR("\x1B[2J\0"));
}
//------------------------------------------------------------------------------------
static void cmdHelpFunction(void)
{

	FRTOS_CMD_makeArgv();

	// HELP WRITE
	if (!strcmp_P( strupr(argv[1]), PSTR("WRITE\0"))) {
		xprintf_P( PSTR("-write\r\n\0"));
		xprintf_P( PSTR("  debugi2c {on|off}\r\n\0"));
		xprintf_P( PSTR("  rtc YYMMDDhhmm\r\n\0"));
		xprintf_P( PSTR("  ee,rtcram {pos} {string}\r\n\0"));
		xprintf_P( PSTR("  sens12V {on|off}\r\n\0"));
		xprintf_P( PSTR("  gprs (pwr|sw|cts|dtr) {on|off}\r\n\0"));
		xprintf_P( PSTR("       cmd {atcmd}\r\n\0"));
		xprintf_P( PSTR("  xbee (pwr|sleep|reset) {on|off}\r\n\0"));
		xprintf_P( PSTR("  bt {on|off}\r\n\0"));
		xprintf_P( PSTR("      msg {string}\r\n\0"));
		xprintf_P( PSTR("      config\r\n\0"));
		xprintf_P( PSTR("  ina (id) conf {value}\r\n\0"));
		xprintf_P( PSTR("  analog {ina_id} conf128 \r\n\0"));
		xprintf_P( PSTR("  clrd {0|1}\r\n\0"));
		xprintf_P( PSTR("  range {on|off}\r\n\0"));
		xprintf_P( PSTR("  out { (enable|disable),(set|reset),(sleep|awake),(ph01|ph10) } {A/B}\r\n\0"));
		xprintf_P( PSTR("      valve (open|close) (A|B) (ms)\r\n\0"));
		xprintf_P( PSTR("      power {on|off}\r\n\0"));
		xprintf_P( PSTR("  consigna (diurna|nocturna)\r\n\0"));
		return;
	}

	// HELP READ
	else if (!strcmp_P( strupr(argv[1]), PSTR("READ\0"))) {
		xprintf_P( PSTR("-read\r\n\0"));
		xprintf_P( PSTR("  rtc, id\r\n\0"));
		xprintf_P( PSTR("  ee,rtcram {pos} {lenght}\r\n\0"));
		xprintf_P( PSTR("  gprs (rsp,rts,dcd,ri)\r\n\0"));
		xprintf_P( PSTR("  ina {id} {conf|chXshv|chXbusv|mfid|dieid}\r\n\0"));
		xprintf_P( PSTR("  ach {0..5}, battery\r\n\0"));
		xprintf_P( PSTR("  term\r\n\0"));
		xprintf_P( PSTR("  din\r\n\0"));
		xprintf_P( PSTR("  contadores\r\n\0"));
		xprintf_P( PSTR("  range\r\n\0"));
		return;

	}

	// HELP RESET
	else if (!strcmp_P( strupr(argv[1]), PSTR("RESET\0"))) {
		xprintf_P( PSTR("-reset\r\n\0"));
		return;

	} else {

		// HELP GENERAL
		xprintf_P( PSTR("\r\nSpymovil %s %s %s %s\r\n\0"), SPX_HW_MODELO, SPX_FTROS_VERSION, SPX_FW_REV, SPX_FW_DATE);
		xprintf_P( PSTR("Clock %d Mhz, Tick %d Hz\r\n\0"),SYSMAINCLK, configTICK_RATE_HZ );
		xprintf_P( PSTR("Available commands are:\r\n\0"));
		xprintf_P( PSTR("-cls\r\n\0"));
		xprintf_P( PSTR("-help\r\n\0"));
		xprintf_P( PSTR("-reset...\r\n\0"));
		xprintf_P( PSTR("-write...\r\n\0"));
		xprintf_P( PSTR("-read...\r\n\0"));

	}

	xprintf_P( PSTR("\r\n\0"));

}
//------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void )
{
	xprintf_P( PSTR("ok\r\n\0"));
}
//------------------------------------------------------------------------------------
static void pv_snprintfP_ERR(void)
{
	xprintf_P( PSTR("error\r\n\0"));
}
//------------------------------------------------------------------------------------
static void pv_cmd_rwEE(uint8_t cmd_mode )
{

int xBytes = 0;
uint8_t length = 0;
char buffer[32];
char *p;

	// read ee {pos} {lenght}
	if ( cmd_mode == RD_CMD ) {
		xBytes = EE_read( (uint32_t)(atol(argv[2])), buffer, (uint8_t)(atoi(argv[3]) ) );
		if ( xBytes == -1 )
			xprintf_P(PSTR("ERROR: I2C:EE:pv_cmd_rwEE\r\n\0"));

		if ( xBytes > 0 ) {
			xprintf_P( PSTR( "%s\r\n\0"),buffer);
		}
		( xBytes > 0 ) ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// write ee pos string
	if ( cmd_mode == WR_CMD ) {
		// Calculamos el largo del texto a escribir en la eeprom.
		p = argv[3];
		while (*p != 0) {
			p++;
			length++;
		}

		xBytes = EE_write( (uint32_t)(atol(argv[2])), argv[3], length );
		if ( xBytes == -1 )
			xprintf_P(PSTR("ERROR: I2C:EE:pv_cmd_rwEE\r\n\0"));

		( xBytes > 0 ) ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_rwRTC(uint8_t cmd_mode )
{

char datetime[24];
RtcTimeType_t rtc;
int8_t xBytes;

	if ( cmd_mode == WR_CMD ) {
		RTC_str2rtc(argv[2], &rtc);				// Convierto el string YYMMDDHHMM a RTC.
		xBytes = RTC_write_dtime(&rtc);		// Grabo el RTC
		if ( xBytes == -1 )
			xprintf_P(PSTR("ERROR: I2C:RTC:pv_cmd_rwRTC\r\n\0"));

		( xBytes > 0)? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	if ( cmd_mode == RD_CMD ) {
		xBytes = RTC_read_dtime(&rtc);
		if ( xBytes == -1 )
			xprintf_P(PSTR("ERROR: I2C:RTC:pv_cmd_rwRTC\r\n\0"));

		RTC_rtc2str(datetime,&rtc);
		xprintf_P( PSTR("%s\r\n\0"), datetime );
		return;
	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_rwRTC_SRAM(uint8_t cmd_mode )
{
	// Como se usa para leer memoria, la impresion la hacemos en hex
	// La RTCram comienza en RTC79410_SRAM_INIT.

uint8_t rtc_sram_buffer[32];
uint8_t i;
int8_t xBytes;
uint8_t length = 0;
char *p;

	// read rtcram {pos} {lenght}
	if ( cmd_mode == RD_CMD ) {
		memset(rtc_sram_buffer, '\0', sizeof(rtc_sram_buffer));
		xBytes = RTC_read( ( RTC79410_SRAM_INIT + (uint8_t)(atoi(argv[2]))), (char *)&rtc_sram_buffer, (uint8_t)(atoi(argv[3])) );
		if ( xBytes == -1 )
			xprintf_P(PSTR("ERROR: I2C:RTC:pv_cmd_rwRTC_SRAM\r\n\0"));

		if ( xBytes > 0 ) {
			// El string leido lo devuelve en cmd_printfBuff por lo que le agrego el CR.
			xprintf_P ( PSTR( "\r\n\0 ") );
			for (i=0; i < atoi(argv[3]); i++ ) {
				xprintf_P (PSTR("[0x%02x]"),rtc_sram_buffer[i]);
			}
			xprintf_P ( PSTR( "\r\n\0 ") );
		}
		( xBytes > 0 ) ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// write rtcram pos string
	if ( cmd_mode == WR_CMD ) {
		// Calculamos el largo del texto a escribir en la eeprom.
		p = argv[3];
		while (*p != 0) {
			p++;
			length++;
		}

		xBytes = RTC_write( ( RTC79410_SRAM_INIT + (uint32_t)(atol(argv[2]))), argv[3], length );
		if ( xBytes == -1 )
			xprintf_P(PSTR("ERROR: I2C:RTC:pv_cmd_rwRTC_SRAM\r\n\0"));

		( xBytes > 0 )? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_sens12V(void)
{
	// sens12V on|off
	if (!strcmp_P( strupr(argv[2]), PSTR("ON\0")) ) {
		IO_set_SENS_12V_CTL();
		pv_snprintfP_OK();
		return;
	}

	if (!strcmp_P( strupr(argv[2]), PSTR("OFF\0")) ) {
		IO_clr_SENS_12V_CTL();
		pv_snprintfP_OK();
		return;
	}

	xprintf_P( PSTR("cmd ERROR: ( write sens12V on{off} )\r\n\0"));
	return;
}
//------------------------------------------------------------------------------------
static void pv_cmd_rwBT(void)
{
	// write bt on|off
	if (!strcmp_P( strupr(argv[2]), PSTR("ON\0")) ) {
		IO_set_BT_PWR_CTL();
//		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
//		xCom_printf_P( fdBT, PSTR("AT+BAUD4\r\n\0"));
//		xprintf_P( PSTR("Init BT9600 ( AT+BAUD4)\r\n\0"),argv[3] );
		pv_snprintfP_OK();
		return;
	}

	if (!strcmp_P( strupr(argv[2]), PSTR("OFF\0")) ) {
		IO_clr_BT_PWR_CTL();
		pv_snprintfP_OK();
		return;
	}

	// write bt msg {string}
	if (!strcmp_P(strupr(argv[2]), PSTR("MSG\0"))) {
		xprintf_P( PSTR("%s\r\0"),argv[3] );
		xCom_printf_P( fdBT,PSTR("%s\r\0"),argv[3] );
		xprintf_P( PSTR("sent->%s\r\n\0"),argv[3] );
		return;
	}

	// write bt config
	if (!strcmp_P(strupr(argv[2]), PSTR("CONFIG\0"))) {
		IO_clr_BT_PWR_CTL();
		vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
		IO_set_BT_PWR_CTL();
		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
		xCom_printf_P( fdBT, PSTR("AT+BAUD4\r\n\0"));

		xCom_printf_P( fdUSB,PSTR("Init BT9600 ( AT+BAUD4)\r\n\0"),argv[3] );
		return;
	}


	xprintf_P( PSTR("cmd ERROR: ( write bt .... )\r\n\0"));
	return;
}
//------------------------------------------------------------------------------------
static void pv_cmd_rwGPRS(uint8_t cmd_mode )
{

uint8_t pin;

	if ( cmd_mode == WR_CMD ) {

		// write gprs (pwr|sw|rts|dtr) {on|off}

		if (!strcmp_P( strupr(argv[2]), PSTR("PWR\0")) ) {
			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_set_GPRS_PWR(); pv_snprintfP_OK(); return;
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_clr_GPRS_PWR(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		if (!strcmp_P( strupr(argv[2]), PSTR("SW\0")) ) {
			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_set_GPRS_SW();
				pv_snprintfP_OK(); return;
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_clr_GPRS_SW(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		if (!strcmp_P( strupr(argv[2]), PSTR("CTS\0")) ) {
			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_set_GPRS_CTS(); pv_snprintfP_OK(); return;
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_clr_GPRS_CTS(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		// Por ahora cableo DTR a CTS.

		if (!strcmp_P( strupr(argv[2]), PSTR("DTR\0")) ) {
			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_set_GPRS_CTS(); pv_snprintfP_OK(); return;
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_clr_GPRS_CTS(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		// ATCMD
		// // write gprs cmd {atcmd}
		if (!strcmp_P(strupr(argv[2]), PSTR("CMD\0"))) {
			xprintf_P( PSTR("%s\r\0"),argv[3] );

			pub_gprs_flush_RX_buffer();
			xCom_printf_P( fdGPRS,PSTR("%s\r\0"),argv[3] );

			xprintf_P( PSTR("sent->%s\r\n\0"),argv[3] );
			return;
		}

		return;
	}

	if ( cmd_mode == RD_CMD ) {
		// read gprs (rsp,cts,dcd,ri)

			// ATCMD
			// read gprs rsp
			if (!strcmp_P(strupr(argv[2]), PSTR("RSP\0"))) {
				pub_gprs_print_RX_Buffer();
				return;
			}

			// DCD
			if (!strcmp_P( strupr(argv[2]), PSTR("DCD\0")) ) {
				pin = IO_read_DCD();
				xprintf_P( PSTR("DCD=%d\r\n\0"),pin);
				pv_snprintfP_OK();
				return;
			}

			// RI
			if (!strcmp_P( strupr(argv[2]), PSTR("RI\0")) ) {
				pin = IO_read_RI();
				xprintf_P( PSTR("RI=%d\r\n\0"),pin);
				pv_snprintfP_OK();
				return;
			}

			// RTS
			if (!strcmp_P( strupr(argv[2]), PSTR("RTS\0")) ) {
				pin = IO_read_RTS();
				xprintf_P( PSTR("RTS=%d\r\n\0"),pin);
				pv_snprintfP_OK();
				return;
			}


			pv_snprintfP_ERR();
			return;
	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_rwXBEE(uint8_t cmd_mode )
{

	if ( cmd_mode == WR_CMD ) {

		// write xbee pwr {on|off}
		if (!strcmp_P( strupr(argv[2]), PSTR("PWR\0")) ) {
			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_set_XBEE_PWR(); pv_snprintfP_OK(); return;
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_clr_XBEE_PWR(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		// write xbee sleep {on|off}
		if (!strcmp_P( strupr(argv[2]), PSTR("SLEEP\0")) ) {
			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_set_XBEE_SLEEP(); pv_snprintfP_OK(); return;
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_clr_XBEE_SLEEP(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		// write xbee reset {on|off}
		if (!strcmp_P( strupr(argv[2]), PSTR("RESET\0")) ) {
			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_set_XBEE_RESET(); pv_snprintfP_OK(); return;
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_clr_XBEE_RESET(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		// MSG
		// write xbee msg
		// Transmito el msg por el puerto del XBEE al dispositivo remoto.

		if (!strcmp_P(strupr(argv[2]), PSTR("MSG\0"))) {
			xprintf_P( PSTR("%s\r\0"),argv[3] );
//			FreeRTOS_write( &pdUART_XBEE, cmd_printfBuff, sizeof(cmd_printfBuff) );

			xprintf_P( PSTR("xbee_sent->%s\r\n\0"),argv[3] );
			return;
		}

		return;
	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_INA(uint8_t cmd_mode )
{

uint16_t val;
uint8_t ina_id;
char data[3];
int8_t xBytes;

	// write ina id conf {value}
	if ( cmd_mode == WR_CMD ) {

		if (!strcmp_P( strupr(argv[3]), PSTR("CONF\0")) ) {
			ina_id = atoi(argv[2]);
			val = atoi( argv[4]);
			data[0] = ( val & 0xFF00 ) >> 8;
			data[1] = ( val & 0x00FF );
			xBytes = INA_write( ina_id, INA3231_CONF, data, 2 );
			if ( xBytes == -1 )
				xprintf_P(PSTR("ERROR: I2C:INA:pv_cmd_INA\r\n\0"));

			pv_snprintfP_OK();
			return;
		}
	}

	// read ina id {conf|chxshv|chxbusv|mfid|dieid}
	if ( cmd_mode == RD_CMD ) {

		ina_id = atoi(argv[2]);

		if (!strcmp_P( strupr(argv[3]), PSTR("CONF\0"))) {
			xBytes = INA_read(  ina_id, INA3231_CONF, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH1SHV\0"))) {
			xBytes = INA_read(  ina_id, INA3221_CH1_SHV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH1BUSV\0"))) {
			xBytes = INA_read(  ina_id, INA3221_CH1_BUSV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH2SHV\0"))) {
			xBytes = INA_read(  ina_id, INA3221_CH2_SHV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH2BUSV\0"))) {
			xBytes = INA_read(  ina_id, INA3221_CH2_BUSV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH3SHV\0"))) {
			xBytes = INA_read(  ina_id, INA3221_CH3_SHV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH3BUSV\0"))) {
			xBytes = INA_read(  ina_id, INA3221_CH3_BUSV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("MFID\0"))) {
			xBytes = INA_read(  ina_id, INA3221_MFID, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("DIEID\0"))) {
			xBytes = INA_read(  ina_id, INA3221_DIEID, data, 2 );
		} else {
			pv_snprintfP_ERR();
			return;
		}

		if ( xBytes == -1 )
			xprintf_P(PSTR("ERROR: I2C:INA:pv_cmd_INA\r\n\0"));

		val = ( data[0]<< 8 ) + data	[1];
		xprintf_P( PSTR("INAID=%d\r\n\0"), ina_id);
		xprintf_P( PSTR("VAL=0x%04x\r\n\0"), val);
		pv_snprintfP_OK();
		return;

	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_rwACH(uint8_t cmd_mode )
{

uint16_t val = 0;
uint8_t channel;

	// read aCh {ch}, bat
	if ( cmd_mode == RD_CMD ) {
		// Bateria
		if (!strcmp_P( strupr( (char *)argv[2]), PSTR("BAT\0"))) {
			val = ACH_read_battery();
			xprintf_P( PSTR("BAT=%d\r\n\0"), val);
			pv_snprintfP_OK();
			return;
		}

		// Canales
		channel = atoi(argv[2]);
		if (  channel > 5 ) {
			pv_snprintfP_ERR();
			return;
		} else {
			val = ACH_read_channel( channel );
			xprintf_P( PSTR("CH%0d=%d\r\n\0"), channel, val);
			pv_snprintfP_OK();
			return;
		}
		pv_snprintfP_ERR();
		return;
	}

	// write ach {id} conf128
	if ( cmd_mode == WR_CMD ) {
		ACH_config_avg128(atoi(argv[2]));
		pv_snprintfP_OK();
		return;
	}
}
//------------------------------------------------------------------------------------
static void pv_cmd_rdBATTERY(void)
{

float battery;

	pub_analog_read_battery(&battery);
	xprintf_P( PSTR("BATTERY=%.02f\r\n\0"), battery );
	return;
}
//------------------------------------------------------------------------------------
static void pv_cmd_range(void)
{
int16_t range;

	pub_rangeMeter_ping(&range);
	xprintf_P( PSTR("RANGE=%d\r\n\0"),range);
	pv_snprintfP_OK();
	return;

}
//------------------------------------------------------------------------------------
static void pv_cmd_wrOUT8814(void)
{
	// write out { (enable|disable),(set|reset),(sleep|awake),(ph01|ph10) } {A/B}
	//             power {on|off}
	//             valve (open|close) (A|B) (ms)

	// write out enable (A|B)
	if (!strcmp_P( strupr(argv[2]), PSTR("ENABLE\0")) ) {
		( OUT_enable_pin( toupper(argv[3][0]), 1) > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// write out disable (A|B)
	if (!strcmp_P( strupr(argv[2]), PSTR("DISABLE\0")) ) {
		( OUT_enable_pin( toupper(argv[3][0]), 0) > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// write out set
	if (!strcmp_P( strupr(argv[2]), PSTR("SET\0")) ) {
		( OUT_reset_pin (1) > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// write out reset
	if (!strcmp_P( strupr(argv[2]), PSTR("RESET\0")) ) {
		( OUT_reset_pin (0) > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// write out sleep
	if (!strcmp_P( strupr(argv[2]), PSTR("SLEEP\0")) ) {
		( OUT_sleep_pin (1) > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// write out awake
	if (!strcmp_P( strupr(argv[2]), PSTR("AWAKE\0")) ) {
		( OUT_sleep_pin (0) > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// write out ph01 (A|B)
	if (!strcmp_P( strupr(argv[2]), PSTR("PH01\0")) ) {
		( OUT_phase_pin( toupper(argv[3][0]), 1) > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// write out ph10 (A|B)
	if (!strcmp_P( strupr(argv[2]), PSTR("PH10\0")) ) {
		( OUT_phase_pin( toupper(argv[3][0]), 0) > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// write out power on|off
	if (!strcmp_P( strupr(argv[2]), PSTR("POWER\0")) ) {

		if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
			OUT_power_on();
			pv_snprintfP_OK();
			return;
		}
		if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
			OUT_power_off();
			pv_snprintfP_OK();
			return;
		}
		pv_snprintfP_ERR();
		return;
	}

	//  write out valve (open|close) (A|B) (ms)
	if (!strcmp_P( strupr(argv[2]), PSTR("VALVE\0")) ) {

		// Proporciono corriente.
		OUT_power_on();
		// Espero 10s que se carguen los condensasores
		vTaskDelay( ( TickType_t)( 10000 / portTICK_RATE_MS ) );

		if (!strcmp_P( strupr(argv[3]), PSTR("OPEN\0")) ) {
			( OUT_valve( toupper(argv[4][0]), V_OPEN, atoi(argv[5]) )  > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
			OUT_power_off();
			return;
		}
		if (!strcmp_P( strupr(argv[3]), PSTR("CLOSE\0")) ) {
			( OUT_valve( toupper(argv[4][0]), V_CLOSE, atoi(argv[5]) )  > 0 ) ?  pv_snprintfP_OK() : pv_snprintfP_ERR();
			OUT_power_off();
			return;
		}

		OUT_power_off();
		pv_snprintfP_ERR();
		return;
	}

	pv_snprintfP_ERR();
	return;

}
//------------------------------------------------------------------------------------


