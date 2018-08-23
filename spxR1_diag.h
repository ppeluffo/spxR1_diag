/*
 * spx.h
 *
 *  Created on: 20 de oct. de 2016
 *      Author: pablo
 */

#ifndef SRC_SPXR1_H_
#define SRC_SPXR1_H_

//------------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------------
#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <compat/deprecated.h>
#include <avr/pgmspace.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <avr/sleep.h>
#include <ctype.h>
#include "avr_compiler.h"
#include "clksys_driver.h"
#include <inttypes.h>

#include "TC_driver.h"
#include "pmic_driver.h"
#include "wdt_driver.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"
#include "croutine.h"
#include "semphr.h"
#include "timers.h"
#include "limits.h"

#include "frtos-io.h"

#include "l_iopines.h"
#include "l_eeprom.h"
#include "l_nvm.h"
#include "l_ina3221.h"
#include "l_rtc79410.h"
#include "l_printf.h"
#include "l_ain.h"
#include "l_outputs.h"

//------------------------------------------------------------------------------------
// DEFINES
//------------------------------------------------------------------------------------
#define SPX_FW_REV "1.0.3"
#define SPX_FW_DATE "@ 20180822"

#define SPX_HW_MODELO "spxR1_diag HW:xmega256A3B R1.0"
#define SPX_FTROS_VERSION "FW:FRTOS10 TICKLESS"

// El datalogger tiene 6 canales fisicos pero 5 disponibles
// ya que uno esta para monitorear la bateria.
//
#define NRO_ANALOG_CHANNELS		5
#define NRO_DIGITAL_CHANNELS	4

#define F_CPU (32000000UL)

//#define SYSMAINCLK 2
//#define SYSMAINCLK 8
#define SYSMAINCLK 32

#define CHAR32	32
#define CHAR64	64
#define CHAR128	128
#define CHAR256	256

#define tkCtl_STACK_SIZE		384
#define tkCmd_STACK_SIZE		384
#define tkGprs_rx_STACK_SIZE	512
#define tkGprs_tx_STACK_SIZE	640
#define tkDigital_STACK_SIZE	640

#define tkCtl_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
#define tkCmd_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
#define tkGprs_rx_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define tkGprs_tx_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define tkDigital_TASK_PRIORITY	( tskIDLE_PRIORITY + 1 )
#
TaskHandle_t xHandle_idle, xHandle_tkCtl,xHandle_tkCmd, xHandle_tkGprsRx, xHandle_tkDigital;

char stdout_buff[CHAR64];

//------------------------------------------------------------------------------------

// Estructura para manejar la hora de aplicar las consignas
typedef struct {
	uint8_t hour;
	uint8_t min;
} time_t;

typedef struct {
	uint8_t modo;
	time_t hora_start;
	time_t hora_fin;
} pwrsave_t;

// Estructura para manejar las OUTPUTS
typedef struct {
	uint8_t modo;
	uint8_t out_A;
	uint8_t out_B;
	time_t consigna_diurna;
	time_t consigna_nocturna;
	uint8_t consigna_aplicada;
} outputs_t;

// Estructura para manejar los canales ANALOGICOS
typedef struct {
	float mag_val[NRO_ANALOG_CHANNELS];
} st_analog_frame;

// Estructura para manejar los canales DIGITALES.
typedef struct {
	uint8_t level[NRO_DIGITAL_CHANNELS];
	uint16_t counter[NRO_DIGITAL_CHANNELS];
	float magnitud[NRO_DIGITAL_CHANNELS];
} st_digital_frame;

typedef struct {
	float analog_val[NRO_ANALOG_CHANNELS];
	float digital_val[NRO_DIGITAL_CHANNELS];
} st_remote_values;

// Estructura de datos manejados por la tarea DATA = ANALOGICO + DIGITAL + RANGE_METER.
typedef struct {
	RtcTimeType_t rtc;
	st_analog_frame analog_frame;
	float battery;
	st_digital_frame digital_frame;
	int16_t range;
} st_data_frame;

//------------------------------------------------------------------------------------
// PROTOTIPOS
//------------------------------------------------------------------------------------
void tkCtl(void * pvParameters);
void tkCmd(void * pvParameters);
void tkGprsRx(void * pvParameters);
void tkDigital(void * pvParameters);

xSemaphoreHandle sem_SYSVars;
StaticSemaphore_t SYSVARS_xMutexBuffer;
#define MSTOTAKESYSVARSSEMPH ((  TickType_t ) 10 )

// Utils
void u_configure_systemMainClock(void);
void u_configure_RTC32(void);
void initMCU(void);

void pub_analog_read_battery ( float *mag_val );
void pub_analog_read_channel ( uint8_t channel, uint16_t *raw_val );

void pub_rangeMeter_init(void);
void pub_rangeMeter_ping(int16_t *range);

void pub_output_set_consigna_diurna(void);
void pub_output_set_consigna_nocturna(void);


#endif /* SRC_SPXR1_H_ */
