/*
 * rfid_reader.h
 *
 *  Created on: 24 kwi 2020
 *      Author: kurza
 */

#ifndef RFID_READER_H
#define RFID_READER_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************
 *      INCLUDES
 *********************/
#include "esp_types.h"
#include "enum_factory.h"
#include "as3933_enums.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *      TYPEDEFS
 **********************/

typedef enum
{
	RFID_1,
	RFID_2,
	RFID_MAX,
}rfid_module_type_t;

#define RFID_GET_CMD_ENUM( RFID_G_CMD ) \
		RFID_G_CMD( RFID_GET_CFG, ) \
		RFID_G_CMD( RFID_GET_RSSI_MAX, ) \
		RFID_G_CMD( RFID_GET_RSSI, ) \
		RFID_G_CMD( RFID_GET_RSSI_1_3, ) \
		RFID_G_CMD( RFID_GET_FALSE_W_UP, ) \
		RFID_G_CMD( RFID_EVENT_W_UP, ) \
		RFID_G_CMD( RFID_GET_MAX, ) \

DECLARE_ENUM(rfid_get_command_event_id_t, RFID_GET_CMD_ENUM);

#define RFID_SET_CMD_ENUM( RFID_S_CMD ) \
		RFID_S_CMD( RFID_RESET, = RFID_GET_MAX ) \
		RFID_S_CMD( RFID_INIT, ) \
		RFID_S_CMD( RFID_SWITCH, ) \
		RFID_S_CMD( RFID_W_UP_INTR_INIT, ) \
		RFID_S_CMD( RFID_CLEAR_WAKE_UP, ) \
		RFID_S_CMD( RFID_CLEAR_FALSE_WAKE_UP, ) \
		RFID_S_CMD( RFID_RESET_RSSI, ) \
		RFID_S_CMD( RFID_SET_FREQ_TOLERANCE, ) \
		RFID_S_CMD( RFID_SET_HYSTERESIS, ) \
		RFID_S_CMD( RFID_SET_GAIN_BOOST, ) \
		RFID_S_CMD( RFID_SET_DISABLE_AGC, ) \
		RFID_S_CMD( RFID_SET_AGC_FIRST_CARRIER, ) \
		RFID_S_CMD( RFID_SET_ANTENA_DUMPER_ENABLE, ) \
		RFID_S_CMD( RFID_SET_ANTENA_DUMPER_VALUE, ) \
		RFID_S_CMD( RFID_SET_INTERNAL_CAPACITOR, ) \
		RFID_S_CMD( RFID_SET_INTERNAL_CAPACITOR_1, ) \
		RFID_S_CMD( RFID_SET_INTERNAL_CAPACITOR_2, ) \
		RFID_S_CMD( RFID_SET_INTERNAL_CAPACITOR_3, ) \
		RFID_S_CMD( RFID_SET_GAIN_REDUCTION, ) \
		RFID_S_CMD( RFID_SET_CORRELATOR_ENABLE, ) \
		RFID_S_CMD( RFID_SET_DATA_MASK, ) \
		RFID_S_CMD( RFID_SET_CHANNEL_ACTIVE, ) \
		RFID_S_CMD( RFID_SET_CHANNEL_DEACTIVE, ) \
		RFID_S_CMD( RFID_SET_ALL_CHANNEL_ON_OFF, ) \
		RFID_S_CMD( RFID_SET_ROUTE_FREQ_DAT, ) \
		RFID_S_CMD( RFID_SET_ROUTE_FREQ_DAT_OFF, ) \
		RFID_S_CMD( RFID_SET_ROUTE_OSC_CLK_DAT, ) \
		RFID_S_CMD( RFID_SET_AUTO_TIMEOUT, ) \
		RFID_S_CMD( RFID_SET_BAND, ) \
		RFID_S_CMD( RFID_SET_MANCHESTER, ) \
		RFID_S_CMD( RFID_SET_BITRATE, ) \
		RFID_S_CMD( RFID_SET_DATA_SLICER, ) \
		RFID_S_CMD( RFID_SET_DATA_SLICER_THR_REDUCTION, ) \
		RFID_S_CMD( RFID_SET_FAST_ENVELOPE_DETECTOR, ) \
		RFID_S_CMD( RFID_SET_MIN_PREAMBLE_LENGTH, ) \
		RFID_S_CMD( RFID_SET_AGC_UP_AND_DOWN, ) \
		RFID_S_CMD( RFID_SET_ARTIFICIAL_WAKE_UP, ) \
		RFID_S_CMD( RFID_SET_MAX, ) \

DECLARE_ENUM(rfid_set_command_event_id_t, RFID_SET_CMD_ENUM);

#define RFID_RESPONSE_ENUM( RFID_RESPONSE ) \
		RFID_RESPONSE( RFID_RESPONSE_RSSI_MAX, ) \
		RFID_RESPONSE( RFID_RESPONSE_RSSI_1_3, ) \
		RFID_RESPONSE( RFID_RESPONSE_RSSI_1, ) \
		RFID_RESPONSE( RFID_RESPONSE_RSSI_2, ) \
		RFID_RESPONSE( RFID_RESPONSE_RSSI_3, ) \
		RFID_RESPONSE( RFID_RESPONSE_FALSE_W_UP, ) \
		RFID_RESPONSE( RFID_RESPONSE_EVENT_W_UP, ) \
		RFID_RESPONSE( RFID_RESPONSE_MAX, ) \

DECLARE_ENUM(rfid_response_event_id_t, RFID_RESPONSE_ENUM);

typedef struct
{
	union
	{
		rfid_get_command_event_id_t get;
		rfid_set_command_event_id_t set;
	};
}__attribute__((packed)) rfid_command_event_id_t;

typedef union
{
	bool _boolean;
	uint8_t _uint8;
	uint16_t _uint16;
	uint32_t _uint32;
}rfid_event_parametr_t;

typedef struct
{
	union
	{
		rfid_command_event_id_t cmd;
		rfid_response_event_id_t resp;
		uint16_t num;
	};
	rfid_event_parametr_t arg;
}__attribute__((packed)) rfid_event_t;

typedef void (*rfid_get_callback_t)(rfid_event_t event);

/**********************
 * GLOBAL VARIABLES
 **********************/
static uint32_t band_module[RFID_MAX] =
{	
	CONFIG_AS3933_1_DATA_GPIO, 
	CONFIG_AS3933_2_DATA_GPIO}
;

/**********************
 * GLOBAL PROTOTYPES
 **********************/
esp_err_t rfid_create(void);
esp_err_t rfid_destroy(void);
esp_err_t rfid_send_event(rfid_event_t *event);
rfid_get_callback_t rfid_register_get_callback_fn(rfid_get_callback_t cb);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* RFID_READER_H */
