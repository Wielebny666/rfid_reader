/*
 * rfid_reader.c
 *
 *  Created on: 23 maj 2020
 *      Author: kurza
 */

/*********************
 *      INCLUDES
 *********************/
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <string.h>

#include "esp_log.h"
#include "esp_err.h"

#include "as3933_hal.h"
#include "as3933.h"

#include "rfid_reader.h"

/*********************
 *      DEFINES
 *********************/
#define RFID_CONFIG                      \
	{                                    \
		{CONFIG_AS3933_SPI_NUM,          \
		 CONFIG_AS3933_SPI_CLOCK,        \
		 CONFIG_AS3933_MOSI_GPIO,        \
		 CONFIG_AS3933_MISO_GPIO,        \
		 CONFIG_AS3933_CLK_GPIO,         \
		 CONFIG_AS3933_1_CS_GPIO,        \
		 CONFIG_AS3933_1_DATA_GPIO,      \
		 CONFIG_AS3933_1_W_UP_GPIO},     \
			{CONFIG_AS3933_SPI_NUM,      \
			 CONFIG_AS3933_SPI_CLOCK,    \
			 CONFIG_AS3933_MOSI_GPIO,    \
			 CONFIG_AS3933_MISO_GPIO,    \
			 CONFIG_AS3933_CLK_GPIO,     \
			 CONFIG_AS3933_2_CS_GPIO,    \
			 CONFIG_AS3933_2_DATA_GPIO,  \
			 CONFIG_AS3933_2_W_UP_GPIO}, \
	}

/**********************
 *      TYPEDEFS
 **********************/
typedef enum
{
	RFID_DESTROY,
	RFID_QUEUE_CREATE_FAIL,
	RFID_CFG_ALLOC_FAIL,
	RFID_DRV_INIT_FAIL,
	RFID_TASK_CREATE_FAIL,
} rfid_rollback_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static as3933_cfg_t *as3933_cfg; //[RFID_MAX] = RFID_CONFIG;
static void rfid_control_task(void *pvParameter __attribute__((unused)));
static void IRAM_ATTR rfid_wake_up(bool value);
static esp_err_t rfid_rollback(rfid_rollback_t step);

/**********************
 *  STATIC VARIABLES
 **********************/
static const char *TAG = "rfid_reader";

static TaskHandle_t rfid_task_handle = NULL;
static QueueHandle_t rfid_queue_handle = NULL;
static as3933_handle_t as3933_drv_handle[RFID_MAX] = {NULL, NULL};
static rfid_get_callback_t rfid_get_callback = NULL;

/**********************
 *  GLOBAL VARIABLES
 **********************/
DEFINE_ENUM(rfid_get_command_event_id_t, RFID_GET_CMD_ENUM);
DEFINE_ENUM(rfid_set_command_event_id_t, RFID_SET_CMD_ENUM);
DEFINE_ENUM(rfid_response_event_id_t, RFID_RESPONSE_ENUM);

/**********************
 *      MACROS
 **********************/
#define CHECK(a, ret_val, str, ...)                                           \
	if (!(a))                                                                 \
	{                                                                         \
		ESP_LOGE(TAG, "%s(%u): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
		return ret_val;                                                       \
	}

#define MAX(a, b) (((a) > (b)) ? (a) : (b))

#define RFID_BOOL_EVENT_SEND(g_id, g_arg)                                           \
	if (rfid_get_callback == NULL)                                                  \
		return;                                                                     \
	ESP_LOGI(TAG, "RFID event send %s", get_rfid_response_event_id_t_string(g_id)); \
	rfid_event_t event;                                                             \
	event.resp = g_id;                                                              \
	event.arg._boolean = g_arg;                                                     \
	rfid_get_callback(event);

#define RFID_UINTX_EVENT_SEND(g_id, g_arg, type)                                    \
	if (rfid_get_callback == NULL)                                                  \
		return;                                                                     \
	ESP_LOGI(TAG, "RFID event send %s", get_rfid_response_event_id_t_string(g_id)); \
	rfid_event_t event;                                                             \
	event.resp = g_id;                                                              \
	event.arg._uint##type = g_arg;                                                  \
	rfid_get_callback(event);

#define RFID_UINT8_EVENT_SEND(g_id, g_arg) \
	RFID_UINTX_EVENT_SEND(g_id, g_arg, 8)

#define RFID_UINT16_EVENT_SEND(g_id, g_arg) \
	RFID_UINTX_EVENT_SEND(g_id, g_arg, 16)

#define RFID_UINT32_EVENT_SEND(g_id, g_arg) \
	RFID_UINTX_EVENT_SEND(g_id, g_arg, 32)

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
esp_err_t rfid_create(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	rfid_queue_handle = xQueueCreate(CONFIG_RFID_QUEUE_SIZE, sizeof(rfid_event_t));
	CHECK((rfid_queue_handle != NULL), ESP_FAIL, "RFID CREATE QUEUE FAIL");

	vQueueAddToRegistry(rfid_queue_handle, "rfid_queue");

	as3933_cfg = calloc(RFID_MAX, sizeof(as3933_cfg_t));
	if (as3933_cfg == NULL)
	{
		rfid_rollback(RFID_CFG_ALLOC_FAIL);
		return ESP_FAIL;
	}

	memcpy(as3933_cfg, (as3933_cfg_t[])RFID_CONFIG, RFID_MAX * sizeof(as3933_cfg_t));

	as3933_drv_handle[RFID_1] = as3933_create(&as3933_cfg[RFID_1]);
	//as3933_drv_handle[RFID_2] = as3933_create(&as3933_cfg[RFID_2]);

	if (as3933_drv_handle[RFID_1] == NULL) // || as3933_drv_handle[RFID_2] == NULL)
	{
		rfid_rollback(RFID_DRV_INIT_FAIL);
		return ESP_FAIL;
	}

	portBASE_TYPE res = xTaskCreatePinnedToCore(rfid_control_task, "rfid_reader_task", 1024 * 3, NULL, configMAX_PRIORITIES - 1, &rfid_task_handle, 0);

	if (res == pdFAIL || rfid_task_handle == NULL)
	{
		rfid_rollback(RFID_TASK_CREATE_FAIL);
		return ESP_FAIL;
	}

	return ESP_OK;
}

esp_err_t rfid_destroy(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	esp_err_t err = rfid_rollback(RFID_DESTROY);
	CHECK((err == ESP_OK), ESP_FAIL, "RFID DESTROY FAILED");
	return err;
}

esp_err_t rfid_send_event(rfid_event_t *event)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	CHECK((rfid_task_handle != NULL), ESP_FAIL, "RFID TASK NOT EXIST");
	CHECK((rfid_queue_handle != NULL), ESP_FAIL, "RFID QUEUE NOT EXIST");

	if (xQueueSendToBack(rfid_queue_handle, (void *)event, portMAX_DELAY) == errQUEUE_FULL)
	{
		ESP_LOGW(TAG, "RFID QUEUE FULL");
		return ESP_FAIL;
	}
	return ESP_OK;
}

rfid_get_callback_t rfid_register_get_callback_fn(rfid_get_callback_t cb)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	CHECK((cb != NULL), NULL, "CALLBACK FUNC NOT EXIST");
	rfid_get_callback_t prev_cb = rfid_get_callback;
	rfid_get_callback = cb;

	return prev_cb;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
static void rfid_control_task(void *pvParameter __attribute__((unused)))
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	rfid_event_t evt;

	while (xQueueReceive(rfid_queue_handle, &evt, portMAX_DELAY) == pdTRUE)
	{
		if ((evt.cmd.get < RFID_GET_MAX && rfid_get_callback == NULL) || evt.cmd.set >= RFID_SET_MAX)
			continue;

		if (evt.num < RFID_GET_MAX)
			ESP_LOGI(TAG, "RFID event get %s", get_rfid_get_command_event_id_t_string(evt.cmd.get));
		else
			ESP_LOGI(TAG, "RFID event set %s", get_rfid_set_command_event_id_t_string(evt.cmd.set));

		switch (evt.cmd.get)
		{
		case RFID_GET_CFG:
		{
			as3933_read_config();
			as3933_print_config();
			break;
		}
		case RFID_GET_RSSI:
		{
			RFID_UINT8_EVENT_SEND(RFID_RESPONSE_RSSI_MAX + evt.arg._uint8, as3933_get_rssi(evt.arg._uint8));
			break;
		}
		case RFID_GET_RSSI_1_3:
		{
			uint8_t rssi[3] = {0, 0, 0};
			as3933_read_bytes(10, rssi, evt.arg._uint8);
			RFID_UINT32_EVENT_SEND(RFID_RESPONSE_RSSI_1_3, rssi[2] << 16 | rssi[1] << 8 | rssi[0]);
			break;
		}
		case RFID_GET_RSSI_MAX:
		{
			uint8_t rssi[3] = {0, 0, 0};
			as3933_read_bytes(10, rssi, 3);
			RFID_UINT8_EVENT_SEND(RFID_RESPONSE_RSSI_1, MAX(MAX(rssi[0], rssi[1]), rssi[2]));
			break;
		}
		case RFID_GET_FALSE_W_UP:
		{
			RFID_UINT8_EVENT_SEND(RFID_RESPONSE_FALSE_W_UP, as3933_get_false_wake_up_register());
			break;
		}
		case RFID_EVENT_W_UP:
		{
			RFID_BOOL_EVENT_SEND(RFID_RESPONSE_EVENT_W_UP, evt.arg._boolean);
			break;
		}
		default:
			//ESP_LOGE(TAG, "Event type SET: %d", evt.cmd.set);
			break;
		}

		switch (evt.cmd.set)
		{
		case RFID_INIT:
		{
			as3933_init_config();
			break;
		}
		case RFID_SWITCH:
		{
			if (evt.arg._uint8 >= RFID_MAX)
				evt.arg._uint8 = RFID_2;
			as3933_switch(as3933_drv_handle[evt.arg._uint8]);
			break;
		}
		case RFID_RESET:
		{
			as3933_reset();
			break;
		}
		case RFID_W_UP_INTR_INIT:
		{
			if (evt.arg._uint8 >= RFID_MAX)
				evt.arg._uint8 = RFID_2;

			esp_err_t resp = as3933_w_up_intr_init(as3933_cfg[evt.arg._uint8].w_up, rfid_wake_up);

			if (resp != ESP_OK)
			{
				ESP_LOGE(TAG, "W UP INTERUPT INIT FAIL");
			}
			break;
		}
		case RFID_CLEAR_WAKE_UP:
		{
			as3933_clear_wake_up();
			break;
		}
		case RFID_CLEAR_FALSE_WAKE_UP:
		{
			as3933_clear_false_wake_up();
			break;
		}
		case RFID_RESET_RSSI:
		{
			as3933_reset_rssi();
			break;
		}
		case RFID_SET_FREQ_TOLERANCE:
		{
			if (evt.arg._uint8 > 3)
				evt.arg._uint8 = 3;
			as3933_set_freq_tolerance(evt.arg._uint8);
			break;
		}
		case RFID_SET_HYSTERESIS:
		{
			if (evt.arg._uint8 > 3)
				evt.arg._uint8 = 3;
			as3933_set_comparator_hysteresis(evt.arg._uint8);
			break;
		}
		case RFID_SET_CORRELATOR_ENABLE:
		{
			as3933_set_patern_correlation(evt.arg._boolean);
			break;
		}
		case RFID_SET_DATA_MASK:
		{
			as3933_set_mask_data_before_wu(evt.arg._boolean);
			break;
		}
		case RFID_SET_GAIN_BOOST:
		{
			as3933_set_3db_gain_boost(evt.arg._boolean);
			break;
		}
		case RFID_SET_DISABLE_AGC:
		{
			as3933_set_disable_agc(evt.arg._boolean);
			break;
		}
		case RFID_SET_AGC_FIRST_CARRIER:
		{
			as3933_set_agc_first_carrier(evt.arg._boolean);
			break;
		}
		case RFID_SET_ANTENA_DUMPER_ENABLE:
		{
			as3933_enable_antenna_damper(evt.arg._boolean);
			break;
		}
		case RFID_SET_ANTENA_DUMPER_VALUE:
		{
			if (evt.arg._uint8 > 3)
				evt.arg._uint8 = 3;
			as3933_set_antenna_damper(evt.arg._uint8);
			break;
		}
		case RFID_SET_INTERNAL_CAPACITOR_1:
		case RFID_SET_INTERNAL_CAPACITOR_2:
		case RFID_SET_INTERNAL_CAPACITOR_3:
		{
			if (evt.arg._uint8 > 31)
				evt.arg._uint8 = 31;
			as3933_set_capacity(evt.cmd.set - RFID_SET_INTERNAL_CAPACITOR, evt.arg._uint8);
			break;
		}
		case RFID_SET_GAIN_REDUCTION:
		{
			as3933_set_gain_reduction(evt.arg._uint8);
			break;
		}
		case RFID_SET_CHANNEL_ACTIVE:
		{
			as3933_set_channel(evt.arg._uint8, true);
			break;
		}
		case RFID_SET_ALL_CHANNEL_ON_OFF:
		{
			as3933_set_channel(0, evt.arg._boolean);
			break;
		}
		case RFID_SET_ROUTE_FREQ_DAT_OFF:
		{
			as3933_route_res_freq_on_dat(0, evt.arg._boolean);
			break;
		}
		case RFID_SET_ROUTE_FREQ_DAT:
		{
			if (evt.arg._uint8 > 0 && evt.arg._uint8 < 4)
			{
				as3933_route_res_freq_on_dat(evt.arg._uint8, true);
			}
			break;
		}
		case RFID_SET_ROUTE_OSC_CLK_DAT:
		{
			as3933_route_clock_on_dat(evt.arg._boolean);
			break;
		}
		case RFID_SET_AUTO_TIMEOUT:
		{
			as3933_set_auto_time_out(evt.arg._uint8);
			break;
		}
		case RFID_SET_BAND:
		{
			as3933_band_select(evt.arg._uint8);
			break;
		}
		case RFID_SET_BITRATE:
		{
			as3933_set_bitrate(evt.arg._uint8);
			break;
		}
		default:
			break;
		}
	}

	ESP_LOGE(TAG, "%s finish !!", __FUNCTION__);

	vTaskDelay(pdMS_TO_TICKS(500)); //wait for 500 ms
	vTaskDelete(NULL);
}

#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
static esp_err_t rfid_rollback(rfid_rollback_t step)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	switch (step)
	{
	case RFID_DESTROY:
		if (step == RFID_DESTROY)
			ESP_LOGI(TAG, "RFID DESTROY");
		rfid_get_callback = NULL;
		vTaskDelete(rfid_task_handle);
		/* no break */
	case RFID_TASK_CREATE_FAIL:
		if (step == RFID_TASK_CREATE_FAIL)
			ESP_LOGE(TAG, "RFID TASK CREATE FAIL");

		if (as3933_drv_handle[RFID_1] != NULL)
		{
			as3933_destroy(as3933_drv_handle[RFID_1], false);
			as3933_drv_handle[RFID_1] = NULL;
		}

		if (as3933_drv_handle[RFID_2] != NULL)
		{
			as3933_destroy(as3933_drv_handle[RFID_2], false);
			as3933_drv_handle[RFID_2] = NULL;
		}
		/* no break */
	case RFID_DRV_INIT_FAIL:
		if (step == RFID_DRV_INIT_FAIL)
			ESP_LOGE(TAG, "RFID HWD INIT FAIL");
		free(as3933_cfg);
		/* no break */
	case RFID_CFG_ALLOC_FAIL:
		if (step == RFID_CFG_ALLOC_FAIL)
			ESP_LOGE(TAG, "RFID CFG ALLOC FAIL");
		vQueueUnregisterQueue(rfid_queue_handle);
		vQueueDelete(rfid_queue_handle);
		rfid_queue_handle = NULL;
		/* no break */
	case RFID_QUEUE_CREATE_FAIL:
		if (step == RFID_QUEUE_CREATE_FAIL)
			ESP_LOGE(TAG, "RFID QUEUE CREATE FAIL");
		/* no break */
	default:
		return ESP_OK;
		break;
	}
	return ESP_FAIL;
}

static void IRAM_ATTR rfid_wake_up(bool value)
{
	ESP_EARLY_LOGD(TAG, "%s", __FUNCTION__);
	CHECK((rfid_task_handle != NULL), , "RFID TASK NOT EXIST");
	CHECK((rfid_queue_handle != NULL), , "RFID QUEUE NOT EXIST");

	//RFID_BOOL_EVENT_SEND(RFID_RESPONSE_EVENT_W_UP, value);
	portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;

	rfid_event_t rfid_evt;
	rfid_evt.cmd.get = RFID_EVENT_W_UP;
	rfid_evt.arg._boolean = value;

	if (xQueueSendToBackFromISR(rfid_queue_handle, &rfid_evt, &pxHigherPriorityTaskWoken) == errQUEUE_FULL)
	{
		ESP_EARLY_LOGW(TAG, "RFID QUEUE IS FULL");
	}

	if (pxHigherPriorityTaskWoken == pdTRUE)
	{
		portYIELD_FROM_ISR();
	}
}
