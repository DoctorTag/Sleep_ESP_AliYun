/*
 
 */
#include "sdkconfig.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "esp_types.h"
#include "esp_attr.h"
#include "esp_intr_alloc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"

#include "esp_err.h"
#include "esp_log.h"
#include "health_hw.h"

#include "biosensor_sleep.h"

#include "analysis_sleep.h"

#include "sleep_task.h"

QueueHandle_t *hsqueue;
static uint8_t halg_loop = 0;
static const char *TAG = "FeelKit_Alg";
static RingbufHandle_t buf_handle;

static void health_alg_task()
{
	sleep_result alg_result;	
	uint8_t is_inbed = BODY_UNKNOW;
	__sensor_32_type sensor_data;
	ESP_LOGI(TAG, "health_alg_task \n");

	while (halg_loop)
	{
		sensor_data.sensor_u32 = 0;
		//Waiting for data from biosensor.
		if (biosensor_recv(&sensor_data) > 0)
		{

			switch (sensor_data.sensor_byte[3])
			{
			case SAMPLE_PIEZO:
			{
				if (is_inbed < STATUS_OUTBED)
				{
					sensor_data.sensor_byte[3] = 0;
					if (sleep_analysis(sensor_data.sensor_u32, &alg_result) == 0)
					{
						
						//Send an item
					if (xRingbufferSend(buf_handle, (void *)&alg_result, sizeof(sleep_result), pdMS_TO_TICKS(100)) != pdTRUE)
						{
							ESP_LOGI(TAG, "Failed to send item\n");
						}
					}
				}
			}
			break;
			//Event of HW FIFO overflow detected
			case EXT_AD_SLEEP_INBED:
			{
				inbed_result *tmp_bed;
				sensor_data.sensor_byte[3] = 0;
				tmp_bed = sleep_analysis_inbed(sensor_data.sensor_u32);
				//	ESP_LOGI(TAG, "sleep inbed raw data:0x%2x,inbed:%d\n",sensor_data.sensor_u32,tmp_bed);

				if (tmp_bed->inbed != is_inbed)
				{
					
					if (xRingbufferSend(buf_handle, (void *)tmp_bed, sizeof(inbed_result), pdMS_TO_TICKS(100)) != pdTRUE)
					{
						ESP_LOGI(TAG, "Failed to send item\n");
					}
					is_inbed = tmp_bed->inbed;
					ESP_LOGI(TAG, "sleep_analysis_inbed change: %d \n",is_inbed);

					if (is_inbed < STATUS_OUTBED)
						bioSleepStart();
					else
					{
						bioSleepStop();
					}
					
				}
			}
			break;
				//Others
			default:
				ESP_LOGI(TAG, "health alg  data type: %d\n", sensor_data.sensor_byte[3]);
				break;
			}
		}
	}
	health_hw_deinit();
	vRingbufferDelete(buf_handle);
	buf_handle = 0;
	vTaskDelete(NULL);
}

RingbufHandle_t sleep_task_init(void)
{

	buf_handle = xRingbufferCreate(1028, RINGBUF_TYPE_NOSPLIT);
	if (buf_handle == NULL)
	{
		ESP_LOGI(TAG, "Failed to create ring buffer\n");
	}

	sleep_analysis_init(0x00ffff00, 0x001fff00);
	health_hw_init();
	halg_loop = 1;
	xTaskCreate(health_alg_task, "health_alg_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
	ESP_LOGI(TAG, "sleep_task_init \n");
	return buf_handle;
}

void sleep_task_deinit()
{
	halg_loop = 0;
}
