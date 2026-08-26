/*
 * Copyright (c) 2024 Dennis Liu, dennis48161025@gmail.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "hand_global.h"
#include "hand_config.h"
#include "hand_task_priority.h"
#include "hand_task.h"
#include "hand_data/hand_data.h"
#include "hand_data/proto/hand_data.pb.h"
#include "hand_data/coding/hand_coding.h"

#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// for for loop using
#define HAND_MSG_SOURCE_CH101_BASE (HandChipInstance_CH101_SENSOR1)

static const char* TAG = "HAND_TASK";

void hand_task_vl53l1x_collect_data(void* __attribute__((unused)) arg)
{
  VL53L1_Dev_t* vl53l1x_dev_p = hand_global_devs_handle.vl53l1x_dev;
  VL53L1_Error ret;
  EventBits_t event_bits;

  /* start measurement procedure on vl53l1x group  */
  for (uint8_t dev_index = 0; dev_index < HAND_DEV_MAX_NUM_VL53L1X; ++dev_index)
  {
    ret = VL53L1_StartMeasurement(&(vl53l1x_dev_p[dev_index]));
    if (ret == VL53L1_ERROR_NONE)
    {
      /* start successfully */
      ESP_LOGI(TAG, "VL53L1X_{%d} started measurement successfully",
               dev_index + 1);
    }
    else
    {
      ESP_LOGE(
          TAG,
          "VL53L1X_{%d} FAILED to start measurement. VL53L1 Error code: {%d} ",
          dev_index + 1, ret);
    }

    /* TODO: record start measurement states and only collect data from active
     * device */
  }

  /* TODO: add a stop flag in arg */
  /* collect data permanently */
  while (1)
  {
    /* TODO: Allow the task to continue after waiting for n milliseconds when
   only one flag is set. This ensures that if one VL53L1X sensor fails, the
   other can still operate. */
    event_bits = xEventGroupWaitBits(
        hand_global_vl53l1x_event_group,
        HAND_EG_VL53L1X_1_DATA_READY_BIT | HAND_EG_VL53L1X_2_DATA_READY_BIT,
        pdTRUE,  // clear wait bit when exit
        pdTRUE,  // fire only at all bits are set
        portMAX_DELAY);

    int64_t timestamp = esp_timer_get_time();

    /* init new vl53l1x data element */
    hand_vl53l1x_data_element_t new_vl53l1x_data = {0};
    new_vl53l1x_data.timestamp = timestamp;

    /* CRITICAL REGION: call hardware related function */
    /* TODO: make it a loop (like ) */
    /* pseudo code
       for (int i = 0, int v = 0; v < MAX_SHIFT_NUM; ++i, v <<= 1)
       {
          if (active_dev[i] == true)
          {
            // do measurement by &(vl53l1x_dev_p[i])
          }
       }
     */
    if (event_bits & HAND_EG_VL53L1X_1_DATA_READY_BIT)
    {
      VL53L1_RangingMeasurementData_t range_data;
      VL53L1_Error status =
          VL53L1_GetRangingMeasurementData(&(vl53l1x_dev_p[0]), &range_data);

      if (status == VL53L1_ERROR_NONE)
      {
        ESP_LOGV(TAG, "VL53L1X_1: %3.1f (cm)",
                 range_data.RangeMilliMeter / 10.0);
        new_vl53l1x_data.data1 = range_data.RangeMilliMeter / 10.0;
      }
      else
      {
        /* TODO: error handle (set failure bit) */
      }

      status = VL53L1_ClearInterruptAndStartMeasurement(&(vl53l1x_dev_p[0]));

      if (status != VL53L1_ERROR_NONE)
      {
        ESP_LOGW(TAG,
                 "Status of VL53L1X_{%d} in "
                 "`VL53L1_ClearInterruptAndStartMeasurement` is: {%d}",
                 1, status);
        /* TODO: error handle (set failure bit) */
      }
    }

    if (event_bits & HAND_EG_VL53L1X_2_DATA_READY_BIT)
    {
      VL53L1_RangingMeasurementData_t range_data;
      VL53L1_Error status =
          VL53L1_GetRangingMeasurementData(&(vl53l1x_dev_p[1]), &range_data);

      if (status == VL53L1_ERROR_NONE)
      {
        ESP_LOGV(TAG, "VL53L1X_2: %3.1f (cm)",
                 range_data.RangeMilliMeter / 10.0);
        new_vl53l1x_data.data2 = range_data.RangeMilliMeter / 10.0;
      }
      else
      {
        /* TODO: error handle (set failure bit) */
      }

      status = VL53L1_ClearInterruptAndStartMeasurement(&(vl53l1x_dev_p[1]));

      if (status != VL53L1_ERROR_NONE)
      {
        ESP_LOGW(TAG,
                 "Status of VL53L1X_{%d} in "
                 "`VL53L1_ClearInterruptAndStartMeasurement` is: {%d}",
                 2, status);
        /* TODO: error handle (set failure bit) */
      }
    }

    /* TODO: check that using portMAX_DELAY is appropriate */
    xQueueSend(hand_global_vl53l1x_data_queue, &new_vl53l1x_data,
               pdMS_TO_TICKS(HAND_MS_VL53L1X_QUEUE_MAX_DELAY));

    /* TODO: check other bit is set or not -> error handle here (activate_dev[i]
     * = false...) */
  }
}

void hand_task_vl53l1x_from_queue_to_ppb(void* __attribute__((unused)) arg)
{
  hand_vl53l1x_data_element_t new_vl53l1x_data;
  volatile hand_ppb_vl53l1x_data_t* const ppb_p =
      &hand_global_vl53l1x_ping_pong_buffer;

  /* parse task arg (currently, remain unused))*/

  while (1)
  {
    /* pop new_vl53l1x_data from queue */
    if (xQueueReceive(hand_global_vl53l1x_data_queue, &new_vl53l1x_data,
                      portMAX_DELAY) == pdTRUE)
    {
      xSemaphoreTake(hand_global_vl53l1x_ping_pong_mutex, portMAX_DELAY);

      /* get current buffer index */
      uint8_t buf_index = ppb_p->ping_pong_flag ? HAND_PPB_SET_BUFFER_INDEX
                                                : HAND_PPB_UNSET_BUFFER_INDEX;

      /* assign new value */
      ppb_p->data[buf_index].timestamps[ppb_p->current_index] =
          new_vl53l1x_data.timestamp;
      ppb_p->data[buf_index].data1[ppb_p->current_index] =
          new_vl53l1x_data.data1;
      ppb_p->data[buf_index].data2[ppb_p->current_index] =
          new_vl53l1x_data.data2;

      /* increase current_index */
      ++(ppb_p->current_index);

      /* current_index check */
      if (ppb_p->current_index >= HAND_SIZE_PPB_VL53L1X)
      {
        ppb_p->current_index = 0;
        ppb_p->ping_pong_flag = !ppb_p->ping_pong_flag;
      }

      xSemaphoreGive(hand_global_vl53l1x_ping_pong_mutex);
    }
  }
}

void hand_task_vl53l1x_send_data(void* arg)
{
  volatile hand_ppb_vl53l1x_data_t* const ppb_p =
      &hand_global_vl53l1x_ping_pong_buffer;
  uint8_t buffer[HAND_SIZE_NANOPB_BUFFER_VL53L1X] = {0};

  hand_task_arg_vl53l1x_send_data_t* task_arg_p =
      (hand_task_arg_vl53l1x_send_data_t*)arg;

  /* parse task arg */
  int client_socket = *task_arg_p;

  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    // local var
    uint8_t send_buffer_index = 0;
    uint16_t send_data_index = 0;

    HandMsg hand_msg = HandMsg_init_zero;
    // Set direction and message type
    hand_msg.bytes_count = HAND_MSG_BYTES_COUNT_PLACEHOLDER_VALUE;
    hand_msg.direction = HandMsgDirection_FROM_HAND;
    hand_msg.msg_type = HandMainMsgType_DATA;
    hand_msg.chip_type = HandChipType_VL53L1X;

    // Set content
    hand_msg.which_content = HandMsg_data_wrapper_tag;

    // Take the mutex before accessing shared resources, may blocked here
    if (xSemaphoreTake(hand_global_vl53l1x_ping_pong_mutex, portMAX_DELAY) ==
        pdTRUE)
    {
      // Determine which buffer to send
      send_buffer_index = ppb_p->ping_pong_flag ? HAND_PPB_SET_BUFFER_INDEX
                                                : HAND_PPB_UNSET_BUFFER_INDEX;
      // Index for storing the next data item
      send_data_index = ppb_p->current_index;
      // For to swap buffers
      ppb_p->current_index = 0;
      ppb_p->ping_pong_flag = !ppb_p->ping_pong_flag;

      // Release the mutex immediately after accessing shared data
      xSemaphoreGive(hand_global_vl53l1x_ping_pong_mutex);
    }

    if (send_data_index != 0)
    {
      ESP_LOGD(TAG, "vl53l1x send data index is %d", send_data_index);
      /* format args */
      hand_timestamps_arr_arg_t timestamps_arg = {
          .ts_p = ppb_p->data[send_buffer_index].timestamps,
          .count = send_data_index  // data number
      };

      hand_vl53l1x_data_arg_t vl53l1x_sensor1_data_arg = {
          .fp = ppb_p->data[send_buffer_index].data1,
          .count = send_data_index  // data number
      };

      hand_vl53l1x_data_arg_t vl53l1x_sensor2_data_arg = {
          .fp = ppb_p->data[send_buffer_index].data2,
          .count = send_data_index  // data number
      };

      // Fill first sensor data
      HandDataMsg data_msg1 = HandDataMsg_init_zero;
      data_msg1.source = HandChipInstance_VL53L1X_SENSOR1;
      data_msg1.data_type = HandDataType_FLOAT;
      data_msg1.data_count = send_data_index;
      // use timestamps instead of timestamp
      data_msg1.has_timestamp = false;
      data_msg1.timestamps.funcs.encode = hand_encode_timestamps_array;
      data_msg1.timestamps.arg = &timestamps_arg;
      data_msg1.data.funcs.encode = hand_encode_float_array;
      data_msg1.data.arg = &vl53l1x_sensor1_data_arg;

      // Fill second sensor data
      HandDataMsg data_msg2 = HandDataMsg_init_zero;
      data_msg2.source = HandChipInstance_VL53L1X_SENSOR2;
      data_msg2.data_type = HandDataType_FLOAT;
      data_msg2.data_count = send_data_index;
      data_msg2.has_timestamp = false;
      data_msg2.timestamps.funcs.encode = hand_encode_timestamps_array;
      data_msg2.timestamps.arg = &timestamps_arg;
      data_msg2.data.funcs.encode = hand_encode_float_array;
      data_msg2.data.arg = &vl53l1x_sensor2_data_arg;

      HandDataMsg* data_msgs[HAND_DEV_MAX_NUM_VL53L1X] = {&data_msg1,
                                                          &data_msg2};

      // create msg arg
      hand_data_msgs_arr_arg_t msg_arg = {
          .msgs_pp = data_msgs,
          .count = HAND_DEV_MAX_NUM_VL53L1X  // total 2 vl53l1x data msgs
      };

      hand_msg.content.data_wrapper.data_msgs.funcs.encode =
          hand_encode_data_msg_pointers_array;
      hand_msg.content.data_wrapper.data_msgs.arg =
          &msg_arg;  // TODO: check this correct or not

      /* start to encode */
      int64_t start_time = esp_timer_get_time();

      /* prepare ostream */
      pb_ostream_t stream =
          pb_ostream_from_buffer(buffer, HAND_SIZE_NANOPB_BUFFER_VL53L1X);

      /* encode */
      if (!pb_encode(&stream, HandMsg_fields, &hand_msg))
      {
        ESP_LOGE(TAG, "VL53L1X msgs encoding failed: %s",
                 PB_GET_ERROR(&stream));
        // XXX: may cause forever loop
        continue;
      }

      int64_t end_time = esp_timer_get_time();
      int64_t encode_duration = end_time - start_time;
      ESP_LOGD(TAG,
               "VL53L1X message encoded successfully, size: %zu bytes, time: "
               "%lld us",
               stream.bytes_written, encode_duration);

      // Overwrite buffer[1:4] by the function
      hand_overwrite_buf_bytes_count(buffer, stream.bytes_written);

      // Timing start - sending data
      start_time = esp_timer_get_time();

      int ret = send(client_socket, buffer, stream.bytes_written, 0);
      if (ret < 0)
      {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
      }
      else
      {
        ESP_LOGV(TAG, "Message sent successfully");
      }

      end_time = esp_timer_get_time();
      int64_t transmit_duration = end_time - start_time;
      ESP_LOGD(TAG, "Data transmitted successfully, time: %lld us",
               transmit_duration);
    }
    // Delay until the next cycle
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(HAND_MS_VL53L1X_SEND_DATA));
  }
}

static uint8_t hand_ch101_current_tx_dev = HAND_CH101_INVALID_DEV;
static uint8_t hand_ch101_pending_tx_dev = HAND_CH101_INVALID_DEV;
static uint8_t hand_ch101_pending_tx_count = 0U;

enum
{
  HAND_CH101_TRACE_SAMPLE_CAPACITY =
      sizeof(((hand_chx01_iq_data_unit_t*)0)->iq_data) /
      sizeof(ch_iq_sample_t)
};

static float _hand_ch101_clampf(float value, float lower, float upper)
{
  if (value < lower) return lower;
  if (value > upper) return upper;
  return value;
}

static bool _hand_ch101_is_control_eligible(uint8_t dev_num)
{
  return (dev_num < HAND_DEV_MAX_NUM_CH101) &&
         ((HAND_CH101_CONTROL_DEV_MASK & (1U << dev_num)) != 0U);
}

static float _hand_ch101_median_noise_filter(const ch_iq_sample_t* iq_data,
                                             uint16_t first_noise_sample,
                                             uint16_t sample_count,
                                             float* scratch)
{
  if ((iq_data == NULL) || (scratch == NULL) ||
      (first_noise_sample >= sample_count))
  {
    return NAN;
  }

  // Calculate noise energy and store it in the scratch buffer
  const uint16_t count = (uint16_t)(sample_count - first_noise_sample);
  for (uint16_t i = 0U; i < count; ++i)
  {
    scratch[i] = ch_iq_to_amplitude(&iq_data[first_noise_sample + i]);
  }

  // Merge-insertion sort (perform in-place sorting directly in scratch)
  for (uint16_t i = 1U; i < count; ++i)
  {
    const float key = scratch[i];
    uint16_t j = i;
    while ((j > 0U) && (scratch[j - 1U] > key))
    {
      scratch[j] = scratch[j - 1U];
      --j;
    }
    scratch[j] = key;
  }

  // Extract the median
  if ((count & 1U) != 0U)
  {
    return scratch[count / 2U];
  }

  return 0.5f * (scratch[(count / 2U) - 1U] + scratch[count / 2U]);
}

/**
 * @brief Round-robin scanning for initial target acquisition without cross-talk disturbance.
 */
typedef enum {
    HAND_CH101_STATE_INITIAL_SCAN,
    HAND_CH101_STATE_TRACKING
} hand_ch101_state_t;

static hand_ch101_state_t g_ch101_state = HAND_CH101_STATE_INITIAL_SCAN;
static uint8_t g_scan_dev_index = 0;


static void hand_ch101_perform_initial_scan(ch_group_t* grp_ptr) 
{
    const uint8_t num_ports = ch_get_num_ports(grp_ptr);
    uint8_t target_tx = HAND_CH101_INVALID_DEV;

    // Find next eligible control sensor according to HAND_CH101_CONTROL_DEV_MASK (0x0D)
    for (uint8_t i = 0; i < num_ports; ++i) 
    {
        uint8_t candidate = (g_scan_dev_index + i) % num_ports;
        if (_hand_ch101_is_control_eligible(candidate))
        { 
          ch_dev_t* dev_ptr = ch_get_dev_ptr(grp_ptr, candidate);
          if ((dev_ptr != NULL) && ch_sensor_is_connected(dev_ptr))
          {
              target_tx = candidate;
              g_scan_dev_index = (candidate + 1) % num_ports;
              break;
          }
        }
    }
    // Reconfigure modes transactionally 1 TX_RX device rest RX_ONLY

    for (uint8_t dev = 0U; dev < num_ports; ++dev) 
    {
      ch_dev_t* dev_ptr = ch_get_dev_ptr(grp_ptr, dev);
      
      if ((dev_ptr != NULL) && ch_sensor_is_connected(dev_ptr)) 
      {
        if (dev == target_tx) 
        {
            ch_set_mode(dev_ptr, CH_MODE_TRIGGERED_TX_RX);
        } 
        else 
        {
            ch_set_mode(dev_ptr, CH_MODE_TRIGGERED_RX_ONLY);
        }
      }
    }
}

static void _hand_ch101_handle_data_ready(ch_group_t* grp_ptr)
{
  hand_chx01_simple_data_element_t simple_data = {0};
  hand_chx01_amp_data_element_t amp_data = {0};
  hand_chx01_iq_data_element_t iq_data = {0};

  uint8_t dev_num;
  int num_samples = 0;
  // uint8_t amp_error = 0;
  uint8_t iq_error = 0;

  hand_ch101_range_quality_t quality[HAND_DEV_MAX_NUM_CH101] = {0};
  float noise_power_scratch[HAND_CH101_TRACE_SAMPLE_CAPACITY] = {0.0f};

  uint8_t tx_count = 0U;
  uint8_t actual_tx_dev = HAND_CH101_INVALID_DEV;

  const uint8_t num_ports = ch_get_num_ports(grp_ptr);

  for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++)
  {
    uint32_t range = 0;

    ch_dev_t* dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
    
    if (ch_sensor_is_connected(dev_ptr))
    {
      /* Get measurement results from each connected sensor
      *   For sensor in transmit/receive mode, report one-way echo
      *   distance,  For sensor(s) in receive-only mode, report direct
      *   one-way distance from transmitting sensor
      */
      simple_data.simple_data[dev_num].timestamps = esp_timer_get_time();
      ch_mode_t mode = ch_get_mode(dev_ptr);
     
      if (ch_get_mode(dev_ptr) == CH_MODE_TRIGGERED_TX_RX)
      {
        actual_tx_dev = dev_num;
        ++tx_count;

        range = ch_get_range(dev_ptr, CH_RANGE_ECHO_ROUND_TRIP);
        simple_data.simple_data[dev_num].range = range / 32.0f;
        simple_data.simple_data[dev_num].amp = ch_get_amplitude(dev_ptr);

        if (range == CH_NO_TARGET)
        {
          simple_data.simple_data[dev_num].range = 0;
          simple_data.simple_data[dev_num].amp = 0;
          
          // DEBUG only
          ESP_LOGI(TAG, "Port {%d} target not found", dev_num);
        }
      }
      else if (ch_get_mode(dev_ptr) == CH_MODE_TRIGGERED_RX_ONLY)
      {
        range = ch_get_range(dev_ptr, CH_RANGE_ECHO_ROUND_TRIP);
        simple_data.simple_data[dev_num].range = range / 32.0f;
        simple_data.simple_data[dev_num].amp = ch_get_amplitude(dev_ptr);
        if (range == CH_NO_TARGET)
        {
          simple_data.simple_data[dev_num].range = 0;
          simple_data.simple_data[dev_num].amp = 0;
          
          // DEBUG only
          ESP_LOGI(TAG, "Port {%d} target not found", dev_num);
        }
      }

      


      num_samples = ch_get_num_samples(dev_ptr);
      simple_data.simple_data[dev_num].sample_num = num_samples;
      
      /*                                  
      if (!amp_error)
      {
        ESP_LOGI(TAG,"dev %d no amp queue error exists...", dev_num);
      } 
      else
      {
        ESP_LOGI(TAG, "CH101 Read amp error exists...");
      }
      */

      iq_error = ch_get_iq_data(dev_ptr, 
                                iq_data.iq_data[dev_num].iq_data, 
                                0, 
                                num_samples, 
                                CH_IO_MODE_BLOCK);
      
      if (!iq_error)
      {
        // Step A: Calculate Instantaneous Magnitude from raw I and Q samples
        for (int s = 0; s < num_samples; ++s)
        {
          amp_data.amp_data[dev_num].amp_data[s] = ch_iq_to_amplitude(&iq_data.iq_data[dev_num].iq_data[s]);
        }
        /*
        for (int i = 0; i < num_samples; ++i)
        {
          ESP_LOGI(TAG, "dev %d num sample %d q: %d, i: %d",
                   dev_num,
                   i + 1,
                   iq_data.iq_data[dev_num].q[i], 
                   iq_data.iq_data[dev_num].i[i]);
        }
        */
      }

      /* Build quality evaluation information */
      hand_ch101_range_quality_t* const metric = &quality[dev_num];
      metric->is_connected = true;
      metric->is_control_eligible = _hand_ch101_is_control_eligible(dev_num);
      metric->operating_mode = mode;
      
      if ((range != CH_NO_TARGET) && (range != 0U))
      {
        metric->measured_path_mm = range / 32.0f;
      }
      else
      {
        metric->measured_path_mm = NAN;
      }
      
      metric->has_valid_dsp_range = isfinite(metric->measured_path_mm) && (metric->measured_path_mm > 0.0f);
      
      if (metric->has_valid_dsp_range)
      {
        metric->dsp_echo_amplitude_lsb = (float)simple_data.simple_data[dev_num].amp;
      }
      else
      {
        metric->dsp_echo_amplitude_lsb = 0.0f;
      }
      
      metric->has_current_iq_trace = (!iq_error) && (num_samples > 0);
      metric->echo_snr_db = -INFINITY;
      metric->relative_handoff_level_db = -INFINITY;
      
      /* Calculate noise and SNR */
      if (metric->is_control_eligible && metric->has_current_iq_trace)
      {
        const uint16_t first_noise_sample = ch_mm_to_samples(dev_ptr, HAND_CH101_NOISE_ESTIMATOR_START_MM);
        if ((first_noise_sample < num_samples) && 
            ((uint16_t)(num_samples - first_noise_sample) >= (uint16_t)HAND_CH101_MIN_NOISE_SAMPLES))
        {
          metric->iq_noise_power_median_lsb2 = _hand_ch101_median_noise_filter(
              iq_data.iq_data[dev_num].iq_data,
              first_noise_sample,
              (uint16_t)num_samples,
              noise_power_scratch);
      
          if (!isfinite(metric->iq_noise_power_median_lsb2) || (metric->iq_noise_power_median_lsb2 < 1.0f))
          {
            metric->iq_noise_power_median_lsb2 = 1.0f;
          }
      
          if (metric->has_valid_dsp_range && 
              (metric->measured_path_mm >= HAND_CH101_MIN_CONTROL_RANGE_MM) && 
              (metric->dsp_echo_amplitude_lsb > 0.0f))
          {
            const float target_power_lsb2 = metric->dsp_echo_amplitude_lsb * metric->dsp_echo_amplitude_lsb;
            metric->echo_signal_power_lsb2 = fmaxf(target_power_lsb2 - metric->iq_noise_power_median_lsb2, 0.0f);
            metric->echo_snr_linear = metric->echo_signal_power_lsb2 / metric->iq_noise_power_median_lsb2;
            
            if (metric->echo_snr_linear > 0.0f)
            {
              metric->echo_snr_db = 10.0f * log10f(metric->echo_snr_linear);
            }
            else
            {
              metric->echo_snr_db = -INFINITY;
            }
      
            metric->snr_reliability_score = metric->echo_signal_power_lsb2 / (metric->echo_signal_power_lsb2 + metric->iq_noise_power_median_lsb2);
          }
        }
      }
    }      
  }

  /* Integrated State Machine: Initial Scan vs Tracking Mode */
  if (g_ch101_state == HAND_CH101_STATE_INITIAL_SCAN) 
  {
    bool target_detected = false;

    for (uint8_t dev = 0U; dev < num_ports; ++dev) 
    {
      if (quality[dev].has_valid_dsp_range) 
      {
        target_detected = true;
        break;
      }
    }

    if (target_detected) 
    {
      ESP_LOGI(TAG, "Initial target acquired! Lock-in tracking mode.");
      g_ch101_state = HAND_CH101_STATE_TRACKING;
    } 
    else 
    {
      /* Inline round-robin scan logic */
      uint8_t target_tx = HAND_CH101_INVALID_DEV;

      for (uint8_t i = 0; i < num_ports; ++i) 
      {
        uint8_t candidate = (g_scan_dev_index + i) % num_ports;
        if (_hand_ch101_is_control_eligible(candidate))
        { 
          ch_dev_t* dev_ptr = ch_get_dev_ptr(grp_ptr, candidate);
          if ((dev_ptr != NULL) && ch_sensor_is_connected(dev_ptr))
          {
            target_tx = candidate;
            g_scan_dev_index = (candidate + 1) % num_ports;
            break;
          }
        }
      }

      for (uint8_t dev = 0U; dev < num_ports; ++dev) 
      {
        ch_dev_t* dev_ptr = ch_get_dev_ptr(grp_ptr, dev);
        if ((dev_ptr != NULL) && ch_sensor_is_connected(dev_ptr)) 
        {
          if (dev == target_tx) 
          {
            ch_set_mode(dev_ptr, CH_MODE_TRIGGERED_TX_RX);
          } 
          else 
          {
            ch_set_mode(dev_ptr, CH_MODE_TRIGGERED_RX_ONLY);
          }
        }
      }

      /* Push collected frame data to queue during initial scan and exit early */
      xQueueSend(hand_global_ch101_simple_data_queue, &simple_data, pdMS_TO_TICKS(HAND_MS_CH101_QUEUE_MAX_DELAY));
      xQueueSend(hand_global_ch101_amp_data_queue, &amp_data, pdMS_TO_TICKS(HAND_MS_CH101_QUEUE_MAX_DELAY));
      xQueueSend(hand_global_ch101_iq_data_queue, &iq_data, pdMS_TO_TICKS(HAND_MS_CH101_QUEUE_MAX_DELAY));
      return;
    }
  }

  /* Tracking Mode Logic */
  if ((tx_count == 1U) && (actual_tx_dev < num_ports))
  {
    hand_ch101_current_tx_dev = actual_tx_dev;
  }

  const bool tx_range_available = (tx_count == 1U) &&
                                  (actual_tx_dev < num_ports) &&
                                  quality[actual_tx_dev].is_control_eligible &&
                                  quality[actual_tx_dev].has_valid_dsp_range &&
                                  (quality[actual_tx_dev].measured_path_mm >= HAND_CH101_MIN_CONTROL_RANGE_MM);

  float tx_target_range_mm = tx_range_available ? quality[actual_tx_dev].measured_path_mm : NAN;
  float maximum_handoff_comparison_value = 0.0f;

  for (uint8_t dev = 0U; dev < num_ports; ++dev)
  {
    hand_ch101_range_quality_t* const metric = &quality[dev];
    if (!metric->is_control_eligible ||
        !metric->has_valid_dsp_range ||
        !metric->has_current_iq_trace ||
        (metric->measured_path_mm < HAND_CH101_MIN_CONTROL_RANGE_MM) ||
        (metric->dsp_echo_amplitude_lsb <= 0.0f))
    {
      continue;
    }

    if (tx_range_available)
    {
      if ((dev == actual_tx_dev) && (metric->operating_mode == CH_MODE_TRIGGERED_TX_RX))
      {
        metric->handoff_comparison_value = metric->dsp_echo_amplitude_lsb * tx_target_range_mm * tx_target_range_mm;
        metric->has_valid_handoff_metric = true;
        metric->handoff_metric_is_spreading_compensated = true;
      }
      else if (metric->operating_mode == CH_MODE_TRIGGERED_RX_ONLY)
      {
        const float target_to_receiver_mm = metric->measured_path_mm - tx_target_range_mm;
        if (target_to_receiver_mm > 0.0f)
        {
          metric->handoff_comparison_value = metric->dsp_echo_amplitude_lsb * tx_target_range_mm * target_to_receiver_mm;
          metric->has_valid_handoff_metric = true;
          metric->handoff_metric_is_spreading_compensated = true;
        }
      }
    }
    else
    {
      metric->handoff_comparison_value = metric->dsp_echo_amplitude_lsb;
      metric->has_valid_handoff_metric = true;
      metric->handoff_metric_is_spreading_compensated = false;
    }

    if (metric->has_valid_handoff_metric && (metric->handoff_comparison_value > maximum_handoff_comparison_value))
    {
      maximum_handoff_comparison_value = metric->handoff_comparison_value;
    }
  }

  const float empirical_fov_amplitude_ratio = powf(10.0f, HAND_CH101_EMPIRICAL_FOV_CONTOUR_DB / 20.0f);

  for (uint8_t dev = 0U; dev < num_ports; ++dev)
  {
    hand_ch101_range_quality_t* const metric = &quality[dev];
    if (metric->has_valid_handoff_metric && (maximum_handoff_comparison_value > 0.0f))
    {
      const float relative_amplitude = _hand_ch101_clampf(metric->handoff_comparison_value / maximum_handoff_comparison_value, 0.0f, 1.0f);

      if (relative_amplitude > 0.0f)
      {
        metric->relative_handoff_level_db = 20.0f * log10f(relative_amplitude);
      }
      else
      {
        metric->relative_handoff_level_db = -INFINITY;
      }

      metric->relative_handoff_reliability_score = _hand_ch101_clampf(relative_amplitude / empirical_fov_amplitude_ratio, 0.0f, 1.0f);
    }

    metric->combined_range_quality_score = _hand_ch101_clampf(metric->snr_reliability_score * metric->relative_handoff_reliability_score, 0.0f, 1.0f);
  }

  /* Select best transmitter candidate */
  uint8_t best_candidate = HAND_CH101_INVALID_DEV;
  float best_quality = -1.0f;
  for (uint8_t dev = 0U; dev < num_ports; ++dev)
  {
    const hand_ch101_range_quality_t* const metric = &quality[dev];
    if (!metric->is_control_eligible ||
        !metric->has_valid_dsp_range ||
        !metric->has_current_iq_trace ||
        !metric->has_valid_handoff_metric ||
        (metric->relative_handoff_level_db < HAND_CH101_EMPIRICAL_FOV_CONTOUR_DB) ||
        (metric->combined_range_quality_score <= 0.0f))
    {
      continue;
    }

    if (metric->combined_range_quality_score > best_quality)
    {
      best_quality = metric->combined_range_quality_score;
      best_candidate = dev;
    }
  }

  /* TX switch decision */
  uint8_t target_tx_to_switch = HAND_CH101_INVALID_DEV;
  bool should_reset_pending = false;

  if (tx_count != 1U)
  {
    uint8_t repair_candidate = best_candidate;
    if (repair_candidate == HAND_CH101_INVALID_DEV)
    {
      for (uint8_t dev = 0U; dev < num_ports; ++dev)
      {
        ch_dev_t* const dev_ptr = ch_get_dev_ptr(grp_ptr, dev);
        if (_hand_ch101_is_control_eligible(dev) && (dev_ptr != NULL) && ch_sensor_is_connected(dev_ptr))
        {
          repair_candidate = dev;
          break;
        }
      }
    }

    ESP_LOGE(TAG, "CH101 mode invariant violated: %u TX/RX devices", (unsigned int)tx_count);
    if (repair_candidate != HAND_CH101_INVALID_DEV)
    {
      target_tx_to_switch = repair_candidate;
    }
    should_reset_pending = true;
  }
  else if ((best_candidate != HAND_CH101_INVALID_DEV) && (best_candidate != actual_tx_dev))
  {
    const hand_ch101_range_quality_t* const current = &quality[actual_tx_dev];

    const bool current_is_usable = current->is_control_eligible &&
                                   current->has_valid_dsp_range &&
                                   current->has_current_iq_trace &&
                                   current->has_valid_handoff_metric &&
                                   (current->combined_range_quality_score > 0.0f);

    const bool current_is_below_empirical_contour = !current_is_usable ||
                                                    (current->relative_handoff_level_db < HAND_CH101_EMPIRICAL_FOV_CONTOUR_DB);

    const bool candidate_is_better = best_quality > (current->combined_range_quality_score + HAND_CH101_SWITCH_QUALITY_MARGIN);

    if (current_is_below_empirical_contour && candidate_is_better)
    {
      if (hand_ch101_pending_tx_dev == best_candidate)
      {
        if (hand_ch101_pending_tx_count < UINT8_MAX)
        {
          ++hand_ch101_pending_tx_count;
        }
      }
      else
      {
        hand_ch101_pending_tx_dev = best_candidate;
        hand_ch101_pending_tx_count = 1U;
      }

      if (hand_ch101_pending_tx_count >= HAND_CH101_SWITCH_HOLD_CYCLES)
      {
        target_tx_to_switch = best_candidate;
        should_reset_pending = true;
      }
    }
    else
    {
      should_reset_pending = true;
    }
  }
  else
  {
    should_reset_pending = true;
  }

  /* Execute transactional TX switch */
  if (target_tx_to_switch != HAND_CH101_INVALID_DEV)
  {
    const uint8_t next_tx_dev = target_tx_to_switch;
    bool switch_success = false;

    if ((next_tx_dev < num_ports) && _hand_ch101_is_control_eligible(next_tx_dev))
    {
      ch_dev_t* const next_tx_ptr = ch_get_dev_ptr(grp_ptr, next_tx_dev);
      if ((next_tx_ptr != NULL) && ch_sensor_is_connected(next_tx_ptr))
      {
        ch_mode_t previous_modes[HAND_DEV_MAX_NUM_CH101] = {CH_MODE_IDLE};
        bool connected[HAND_DEV_MAX_NUM_CH101] = {false};
        bool failed = false;

        for (uint8_t dev = 0U; dev < num_ports; ++dev)
        {
          ch_dev_t* const dev_ptr = ch_get_dev_ptr(grp_ptr, dev);
          connected[dev] = (dev_ptr != NULL) && ch_sensor_is_connected(dev_ptr);
          if (connected[dev]) previous_modes[dev] = ch_get_mode(dev_ptr);
        }

        for (uint8_t dev = 0U; dev < num_ports; ++dev)
        {
          if (!connected[dev] || (dev == next_tx_dev)) continue;
          ch_dev_t* const dev_ptr = ch_get_dev_ptr(grp_ptr, dev);
          if ((ch_get_mode(dev_ptr) != CH_MODE_TRIGGERED_RX_ONLY) &&
              (ch_set_mode(dev_ptr, CH_MODE_TRIGGERED_RX_ONLY) != RET_OK))
          {
            failed = true;
            break;
          }
        }

        if (!failed &&
            (ch_get_mode(next_tx_ptr) != CH_MODE_TRIGGERED_TX_RX) &&
            (ch_set_mode(next_tx_ptr, CH_MODE_TRIGGERED_TX_RX) != RET_OK))
        {
          failed = true;
        }

        if (failed)
        {
          for (uint8_t dev = 0U; dev < num_ports; ++dev)
          {
            if (!connected[dev]) continue;
            ch_dev_t* const dev_ptr = ch_get_dev_ptr(grp_ptr, dev);
            ch_set_mode(dev_ptr, previous_modes[dev]);
          }
        }
        else
        {
          switch_success = true;
        }
      }
    }

    if (switch_success)
    {
      hand_ch101_current_tx_dev = next_tx_dev;
    }
  }

  if (should_reset_pending)
  {
    hand_ch101_pending_tx_dev = HAND_CH101_INVALID_DEV;
    hand_ch101_pending_tx_count = 0U;
  }


  ESP_LOGI(TAG, "range: %.3f, %.3f, %.3f, %.3f", simple_data.simple_data[0].range,
  simple_data.simple_data[1].range, simple_data.simple_data[2].range,
  simple_data.simple_data[3].range);

  ESP_LOGI(TAG, "amp: %d, %d, %d, %d", simple_data.simple_data[0].amp,
  simple_data.simple_data[1].amp, simple_data.simple_data[2].amp,
  simple_data.simple_data[3].amp);
  printf("\n");    

  /* push to queue */
  xQueueSend(hand_global_ch101_simple_data_queue, &simple_data,
    pdMS_TO_TICKS(HAND_MS_CH101_QUEUE_MAX_DELAY));

  xQueueSend(hand_global_ch101_amp_data_queue, &amp_data,
    pdMS_TO_TICKS(HAND_MS_CH101_QUEUE_MAX_DELAY));
  
  xQueueSend(hand_global_ch101_iq_data_queue, &iq_data,
    pdMS_TO_TICKS(HAND_MS_CH101_QUEUE_MAX_DELAY));
}


void hand_task_ch101_collect_data(void* __attribute__((unused)) arg)
{ 
  // start periodic timer 
  chbsp_periodic_timer_init(HAND_MS_CH101_DEFAULT_MEASURE_PERIOD,
                            hand_cb_ch101_periodic_timer);
  /*  
  // Disable interrupt and start timer to trigger sensor sampling 
  chbsp_periodic_timer_irq_enable();
  */
  chbsp_periodic_timer_start();


  ESP_LOGI(TAG, "CH101 measurement is starting!");
  
  while (1)
  {
    // wait for event, no ret for only one bit is waited 
    xEventGroupWaitBits(hand_global_ch101_event_group,
                        HAND_EG_CH101_ALL_ACTIVE_DEV_DATA_READY_BIT,
                        true,  // clear on exit
                        true, // wait for any of bits
                        pdMS_TO_TICKS(HAND_MS_CH101_DEFAULT_MEASURE_PERIOD * 2));

    _hand_ch101_handle_data_ready(&hand_global_devs_handle.ch101_group);                           
  }
}

void hand_task_ch101_from_queue_to_ppb(void* __attribute__((unused)) arg)
{
  hand_chx01_simple_data_element_t simple_data = {0};
  hand_chx01_amp_data_element_t amp_data = {0};
  hand_chx01_iq_data_element_t iq_data = {0};

  volatile hand_ppb_ch101_simple_data_t* const simple_ppb_p =
    &hand_global_ch101_simple_ping_pong_buffer;

  volatile hand_ppb_ch101_amp_data_t* const amp_ppb_p =
    &hand_global_ch101_amp_ping_pong_buffer;
  
  volatile hand_ppb_ch101_iq_data_t* const iq_ppb_p =
    &hand_global_ch101_iq_ping_pong_buffer;
 
  
  /* parse task arg (currently, remain unused))*/
  
  while (1)
  {
    if (xQueueReceive(hand_global_ch101_simple_data_queue, &simple_data,
        portMAX_DELAY) == pdTRUE)
    {
      xSemaphoreTake(hand_global_ch101_simple_ping_pong_mutex, portMAX_DELAY);
      
      /* get current buffer index */
      uint8_t buf_index = simple_ppb_p->ping_pong_flag ? HAND_PPB_SET_BUFFER_INDEX
      : HAND_PPB_UNSET_BUFFER_INDEX;
      
      
      
      // i: dev index
      for (uint8_t i = 0; i < HAND_DEV_MAX_NUM_CH101; ++i)
      {
        simple_ppb_p->simple_data[buf_index].simple_data[i][simple_ppb_p->current_index].timestamps =
        simple_data.simple_data[i].timestamps;
        simple_ppb_p->simple_data[buf_index].simple_data[i][simple_ppb_p->current_index].amp =
        simple_data.simple_data[i].amp;
        simple_ppb_p->simple_data[buf_index].simple_data[i][simple_ppb_p->current_index].range =
        simple_data.simple_data[i].range;
        simple_ppb_p->simple_data[buf_index].simple_data[i][simple_ppb_p->current_index].sample_num =
        simple_data.simple_data[i].sample_num; 
      }
      
      /* increase current_index */
      ++(simple_ppb_p->current_index);
      
      /* current_index check */
      if (simple_ppb_p->current_index >= HAND_SIZE_PPB_CH101_SIMPLE)
      {
        simple_ppb_p->current_index = 0;
        simple_ppb_p->ping_pong_flag = !simple_ppb_p->ping_pong_flag;
      }
      
      xSemaphoreGive(hand_global_ch101_simple_ping_pong_mutex);
    }
    
    if (xQueueReceive(hand_global_ch101_amp_data_queue, &amp_data,
        portMAX_DELAY) == pdTRUE)
    {
      xSemaphoreTake(hand_global_ch101_amp_ping_pong_mutex, portMAX_DELAY);
      
      /* get current buffer index */
      uint8_t buf_index = amp_ppb_p->ping_pong_flag ? HAND_PPB_SET_BUFFER_INDEX
      : HAND_PPB_UNSET_BUFFER_INDEX;
      
      // i: dev index
      for (uint8_t i = 0; i < HAND_DEV_MAX_NUM_CH101; ++i)
      {
        for (uint16_t j = 0; j < simple_data.simple_data[i].sample_num; ++j)
        {
          amp_ppb_p->amp_data[buf_index].amp_data[i][amp_ppb_p->current_index].amp_data[j] =
          amp_data.amp_data[i].amp_data[j];
        }
      }
      
      /* increase current_index */
      ++(amp_ppb_p->current_index);
      
      /* current_index check */
      if (amp_ppb_p->current_index >= HAND_SIZE_PPB_CH101_AMP)
      {
        amp_ppb_p->current_index = 0;
        amp_ppb_p->ping_pong_flag = !amp_ppb_p->ping_pong_flag;
      }
      
      xSemaphoreGive(hand_global_ch101_amp_ping_pong_mutex);
    }
    
    if (xQueueReceive(hand_global_ch101_iq_data_queue, &iq_data,
        portMAX_DELAY) == pdTRUE)
    {
      xSemaphoreTake(hand_global_ch101_iq_ping_pong_mutex, portMAX_DELAY);
      
      /* get current buffer index */
      uint8_t buf_index = iq_ppb_p->ping_pong_flag ? HAND_PPB_SET_BUFFER_INDEX
      : HAND_PPB_UNSET_BUFFER_INDEX;
      
      // i: dev index
      for (uint8_t i = 0; i < HAND_DEV_MAX_NUM_CH101; ++i)
      {
        for (uint16_t j = 0; j < simple_data.simple_data[i].sample_num; ++j)
        {
          iq_ppb_p->iq_data[buf_index].iq_data[i][iq_ppb_p->current_index].iq_data[j].i =
          iq_data.iq_data[i].iq_data[j].i;
          
          iq_ppb_p->iq_data[buf_index].iq_data[i][iq_ppb_p->current_index].iq_data[j].q =
          iq_data.iq_data[i].iq_data[j].q;
        }
      }
      
      /* increase current_index */
      ++(iq_ppb_p->current_index);
      
      /* current_index check */
      if (iq_ppb_p->current_index >= HAND_SIZE_PPB_CH101_IQ)
      {
        iq_ppb_p->current_index = 0;
        iq_ppb_p->ping_pong_flag = !iq_ppb_p->ping_pong_flag;
      }
      
      xSemaphoreGive(hand_global_ch101_iq_ping_pong_mutex);
    }
  }
}
  
void hand_task_ch101_send_data(void* arg)
{
  volatile hand_ppb_ch101_simple_data_t* const simple_ppb_p =
  &hand_global_ch101_simple_ping_pong_buffer;

  volatile hand_ppb_ch101_amp_data_t* const amp_ppb_p =
  &hand_global_ch101_amp_ping_pong_buffer;
  
  volatile hand_ppb_ch101_iq_data_t* const iq_ppb_p =
  &hand_global_ch101_iq_ping_pong_buffer;
  
  uint8_t buffer[HAND_SIZE_NANOPB_BUFFER_CH101_SIMPLE] = {0};
  
  uint8_t amp_buffer[HAND_SIZE_NANOPB_BUFFER_CH101_AMP] = {0};
  
  uint8_t iq_buffer[HAND_SIZE_NANOPB_BUFFER_CH101_IQ] = {0};
  
  hand_task_arg_ch101_send_data_t* task_arg_p =
  (hand_task_arg_ch101_send_data_t*)arg;
  
  int client_socket = *task_arg_p;
  
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  while (1)
  {
    // local var
    uint8_t send_simple_buffer_index = 0;
    uint16_t send_simple_data_index = 0;

    uint8_t send_amp_buffer_index = 0U;
    uint16_t send_amp_data_index = 0U;
    
    uint8_t send_iq_buffer_index = 0U;
    uint16_t send_iq_data_index = 0U;

    // Take the mutex before accessing shared resources, may blocked here
    if (xSemaphoreTake(hand_global_ch101_simple_ping_pong_mutex, portMAX_DELAY) == pdTRUE)
    {
      // Determine which buffer to send
      send_simple_buffer_index = simple_ppb_p->ping_pong_flag ? HAND_PPB_SET_BUFFER_INDEX
      : HAND_PPB_UNSET_BUFFER_INDEX;
      
      // Index for storing the next data item
      send_simple_data_index = simple_ppb_p->current_index;
      
      // For to swap buffers
      simple_ppb_p->current_index = 0;
      simple_ppb_p->ping_pong_flag = !simple_ppb_p->ping_pong_flag;
      
      // Release the mutex immediately after accessing shared data
      xSemaphoreGive(hand_global_ch101_simple_ping_pong_mutex);
    }

    if (xSemaphoreTake(hand_global_ch101_amp_ping_pong_mutex, portMAX_DELAY) == pdTRUE)
    {
      // Determine which buffer to send
      send_amp_buffer_index = amp_ppb_p->ping_pong_flag ? HAND_PPB_SET_BUFFER_INDEX
      : HAND_PPB_UNSET_BUFFER_INDEX;
      
      // Index for storing the next data item
      send_amp_data_index = amp_ppb_p->current_index;
      
      // For to swap buffers
      amp_ppb_p->current_index = 0;
      amp_ppb_p->ping_pong_flag = !amp_ppb_p->ping_pong_flag;
      
      // Release the mutex immediately after accessing shared data
      xSemaphoreGive(hand_global_ch101_amp_ping_pong_mutex);
    }

    if (xSemaphoreTake(hand_global_ch101_iq_ping_pong_mutex, portMAX_DELAY) == pdTRUE)
    {
      // Determine which buffer to send
      send_iq_buffer_index = iq_ppb_p->ping_pong_flag ? HAND_PPB_SET_BUFFER_INDEX
      : HAND_PPB_UNSET_BUFFER_INDEX;
      
      // Index for storing the next data item
      send_iq_data_index = iq_ppb_p->current_index;
      
      // For to swap buffers
      iq_ppb_p->current_index = 0;
      iq_ppb_p->ping_pong_flag = !iq_ppb_p->ping_pong_flag;
      
      // Release the mutex immediately after accessing shared data
      xSemaphoreGive(hand_global_ch101_iq_ping_pong_mutex);
    }
    
    if (send_simple_data_index != 0)
    {
      ESP_LOGI(TAG, "ch101 send data index is %d", send_simple_data_index);
      HandMsg hand_msg = HandMsg_init_zero;
      // Set direction and message type
      hand_msg.bytes_count = HAND_MSG_BYTES_COUNT_PLACEHOLDER_VALUE;
      hand_msg.direction = HandMsgDirection_FROM_HAND;
      hand_msg.msg_type = HandMainMsgType_DATA;
      hand_msg.chip_type = HandChipType_CH101;
      // Set content
      hand_msg.which_content = HandMsg_data_wrapper_tag;
    
      /* TODO: could be optimized */
      /*
      hand_timestamps_arr_arg_t simple_timestamps_arg = {
        .ts_p = simple_ppb_p->simple_data[send_buffer_index].timestamps,
        .count = send_data_index, // data number
      };
      */
      
      /* assign every row's first element to iterate every row array in encode
      * function */
      hand_ch101_simple_data_arg_t ch101_data_args[HAND_DEV_MAX_NUM_CH101] = 
      {
        {.simple_p = &(simple_ppb_p->simple_data[send_simple_buffer_index].simple_data[0][0]),
         .count = send_simple_data_index},
        {.simple_p = &(simple_ppb_p->simple_data[send_simple_buffer_index].simple_data[1][0]),
         .count = send_simple_data_index},
        {.simple_p = &(simple_ppb_p->simple_data[send_simple_buffer_index].simple_data[2][0]),
         .count = send_simple_data_index},
        {.simple_p = &(simple_ppb_p->simple_data[send_simple_buffer_index].simple_data[3][0]),
         .count = send_simple_data_index},
      };
         
      /* TODO: should init */
      
      /* assign fields for simples */
      HandDataMsgSimple simple_data_msgs[HAND_DEV_MAX_NUM_CH101];
      for (uint8_t i = 0; i < HAND_DEV_MAX_NUM_CH101; ++i)
      {
        simple_data_msgs[i].data_count = send_simple_data_index;
        /*
        simple_data_msgs[i].has_timestamp = false;
        simple_data_msgs[i].timestamps.funcs.encode = hand_encode_timestamps_array;
        //simple_data_msgs[i].timestamps.arg = &simple_timestamps_arg;
        */
        simple_data_msgs[i].data_type = HandDataType_CH101_SIMPLE;
        simple_data_msgs[i].data.funcs.encode = hand_encode_ch101_simple_data_array;
        simple_data_msgs[i].data.arg = &ch101_data_args[i];
        simple_data_msgs[i].source = HAND_MSG_SOURCE_CH101_BASE + i;
      }

      // create msg arg
      hand_active_simple_data_msgs_arr_arg_t msg_arg = {
        .msgs_p = simple_data_msgs,
        .max_count = HAND_DEV_MAX_NUM_CH101,
        .active_indicator = &hand_global_ch101_active_dev_num,
        .indicator_type = HandDataType_UINT8
      };
      
      hand_msg.content.data_wrapper.data_msgs_simple.funcs.encode =
      hand_encode_active_simple_data_msg_pointers_array;
      hand_msg.content.data_wrapper.data_msgs_simple.arg = &msg_arg;
      
      /* start to encode */
      int64_t start_time = esp_timer_get_time();
      
      /* prepare ostream */
      pb_ostream_t stream =
      pb_ostream_from_buffer(buffer, HAND_SIZE_NANOPB_BUFFER_CH101_SIMPLE);
      
      /* encode */
      if (!pb_encode(&stream, HandMsg_fields, &hand_msg))
      {
        ESP_LOGE(TAG, "CH101 msgs encoding failed: %s", PB_GET_ERROR(&stream));
        // XXX: may cause forever loop
        continue;
      }
      
      int64_t end_time = esp_timer_get_time();
      int64_t encode_duration = end_time - start_time;
      ESP_LOGV(TAG,
               "CH101 simple message encoded successfully, size: %zu bytes, time: %lld us",
               stream.bytes_written, encode_duration);
        
      hand_overwrite_buf_bytes_count(buffer, stream.bytes_written);
      
      // Timing start - sending data
      start_time = esp_timer_get_time();
      
      int ret = send(client_socket, buffer, stream.bytes_written, 0);
      
      if (ret < 0)
      {
        ESP_LOGE(TAG, "Error occurred during sending CH101 msgs: errno %d", errno);
      }
      else
      {
        ESP_LOGV(TAG, "Message sent (CH101) successfully");
      }
      
      end_time = esp_timer_get_time();
      int64_t transmit_duration = end_time - start_time;
      ESP_LOGV(TAG, "CH101 Data transmitted successfully, time: %lld us", transmit_duration);
    }          
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(HAND_MS_CH101_SEND_SIMPLE_DATA));

    if (send_amp_data_index != 0)
    {
      ESP_LOGI(TAG, "ch101 send data index is %d", send_amp_data_index);
      HandMsg hand_msg = HandMsg_init_zero;
      // Set direction and message type
      hand_msg.bytes_count = HAND_MSG_BYTES_COUNT_PLACEHOLDER_VALUE;
      hand_msg.direction = HandMsgDirection_FROM_HAND;
      hand_msg.msg_type = HandMainMsgType_DATA;
      hand_msg.chip_type = HandChipType_CH101;
      // Set content
      hand_msg.which_content = HandMsg_data_wrapper_tag;
    
      /* TODO: could be optimized */
      /*
      hand_timestamps_arr_arg_t simple_timestamps_arg = {
        .ts_p = simple_ppb_p->simple_data[send_buffer_index].timestamps,
        .count = send_data_index, // data number
      };
      */
      
      /* assign every row's first element to iterate every row array in encode
      * function */
      hand_ch101_amp_data_arg_t ch101_data_args[HAND_DEV_MAX_NUM_CH101] =
      {
        {
          .amp_p = &amp_ppb_p->amp_data[send_amp_buffer_index].amp_data[0][0].amp_data[0],
          .count = send_amp_data_index
        },
        {
          .amp_p = &amp_ppb_p->amp_data[send_amp_buffer_index].amp_data[1][0].amp_data[0],
          .count = send_amp_data_index
        },
        {
          .amp_p = &amp_ppb_p->amp_data[send_amp_buffer_index].amp_data[2][0].amp_data[0],
          .count = send_amp_data_index
        },
        {
          .amp_p = &amp_ppb_p->amp_data[send_amp_buffer_index].amp_data[3][0].amp_data[0],
          .count = send_amp_data_index
        },
      };
          
      /* TODO: should init */
      
      /* assign fields for amps */
      HandDataMsgAmp amp_data_msgs[HAND_DEV_MAX_NUM_CH101] = {0};
      for (uint8_t i = 0; i < HAND_DEV_MAX_NUM_CH101; ++i)
      {
        amp_data_msgs[i].data_count = send_amp_data_index;
        amp_data_msgs[i].data_type = HandDataType_CH101_AMP;
        amp_data_msgs[i].data.funcs.encode = hand_encode_ch101_amp_data_array;
        amp_data_msgs[i].data.arg = &ch101_data_args[i];
        amp_data_msgs[i].source = HAND_MSG_SOURCE_CH101_BASE + i; 
      }

      // create msg arg
      hand_active_amp_data_msgs_arr_arg_t msg_arg = {
        .msgs_p = &amp_data_msgs,
        .max_count = HAND_DEV_MAX_NUM_CH101,
        .active_indicator = &hand_global_ch101_active_dev_num,
        .indicator_type = HandDataType_UINT8
      };
      
      hand_msg.content.data_wrapper.data_msgs_amp.funcs.encode =
      hand_encode_active_amp_data_msg_pointers_array;
      hand_msg.content.data_wrapper.data_msgs_amp.arg = &msg_arg;
      
      /* start to encode */
      int64_t start_time = esp_timer_get_time();
      
      /* prepare ostream */
      pb_ostream_t stream =
      pb_ostream_from_buffer(amp_buffer, HAND_SIZE_NANOPB_BUFFER_CH101_AMP);
      
      /* encode */
      if (!pb_encode(&stream, HandMsg_fields, &hand_msg))
      {
        ESP_LOGE(TAG, "CH101 msgs encoding failed: %s", PB_GET_ERROR(&stream));
        // XXX: may cause forever loop
        continue;
      }
      
      int64_t end_time = esp_timer_get_time();
      int64_t encode_duration = end_time - start_time;
      ESP_LOGV(TAG,
               "CH101 amp message encoded successfully, size: %zu bytes, time: %lld us",
               stream.bytes_written, encode_duration);
        
      hand_overwrite_buf_bytes_count(amp_buffer, stream.bytes_written);
      
      // Timing start - sending data
      start_time = esp_timer_get_time();
      
      int ret = send(client_socket, amp_buffer, stream.bytes_written, 0);
      
      if (ret < 0)
      {
        ESP_LOGE(TAG, "Error occurred during sending CH101 msgs: errno %d", errno);
      }
      else
      {
        ESP_LOGV(TAG, "Message sent (CH101) successfully");
      }
      
      end_time = esp_timer_get_time();
      int64_t transmit_duration = end_time - start_time;
      ESP_LOGV(TAG, "CH101 Data transmitted successfully, time: %lld us", transmit_duration);
    }          
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(HAND_MS_CH101_SEND_AMP_DATA));
    if (send_iq_data_index != 0)
    {
      ESP_LOGI(TAG, "ch101 send data index is %d", send_iq_data_index);
      HandMsg hand_msg = HandMsg_init_zero;
      // Set direction and message type
      hand_msg.bytes_count = HAND_MSG_BYTES_COUNT_PLACEHOLDER_VALUE;
      hand_msg.direction = HandMsgDirection_FROM_HAND;
      hand_msg.msg_type = HandMainMsgType_DATA;
      hand_msg.chip_type = HandChipType_CH101;
      // Set content
      hand_msg.which_content = HandMsg_data_wrapper_tag;
      
      /* assign every row's first element to iterate every row array in encode
      * function */

      hand_ch101_iq_data_arg_t ch101_data_args[HAND_DEV_MAX_NUM_CH101] =
      {
        {
          .iq_p = &iq_ppb_p->iq_data[send_iq_buffer_index].iq_data[0][0].iq_data[0],
          .count = send_iq_data_index
        },
        {
          .iq_p = &iq_ppb_p->iq_data[send_iq_buffer_index].iq_data[1][0].iq_data[0],
          .count = send_iq_data_index
        },
        {
          .iq_p = &iq_ppb_p->iq_data[send_iq_buffer_index].iq_data[2][0].iq_data[0],
          .count = send_iq_data_index
        },
        {
          .iq_p = &iq_ppb_p->iq_data[send_iq_buffer_index].iq_data[3][0].iq_data[0],
          .count = send_iq_data_index
        },
      };
       
      /* TODO: should init */
      
      /* assign fields for iqs */
      HandDataMsgIq iq_data_msgs[HAND_DEV_MAX_NUM_CH101] = {0};

      for (uint8_t i = 0; i < HAND_DEV_MAX_NUM_CH101; ++i)
      {
        iq_data_msgs[i].data_count = send_iq_data_index;
        iq_data_msgs[i].data_type = HandDataType_CH101_IQ;
        iq_data_msgs[i].data.funcs.encode = hand_encode_ch101_iq_data_array;
        iq_data_msgs[i].data.arg = &ch101_data_args[i];
        iq_data_msgs[i].source = HAND_MSG_SOURCE_CH101_BASE + i;
      }

      // create msg arg
      hand_active_iq_data_msgs_arr_arg_t msg_arg = {
        .msgs_p = &iq_data_msgs,
        .max_count = HAND_DEV_MAX_NUM_CH101,
        .active_indicator = &hand_global_ch101_active_dev_num,
        .indicator_type = HandDataType_UINT8
      };
      
      hand_msg.content.data_wrapper.data_msgs_iq.funcs.encode =
      hand_encode_active_iq_data_msg_pointers_array;
      hand_msg.content.data_wrapper.data_msgs_iq.arg = &msg_arg;
      
      /* start to encode */
      int64_t start_time = esp_timer_get_time();
      
      /* prepare ostream */
      pb_ostream_t stream =
      pb_ostream_from_buffer(iq_buffer, HAND_SIZE_NANOPB_BUFFER_CH101_IQ);
      
      /* encode */
      if (!pb_encode(&stream, HandMsg_fields, &hand_msg))
      {
        ESP_LOGE(TAG, "CH101 msgs encoding failed: %s", PB_GET_ERROR(&stream));
        // XXX: may cause forever loop
        continue;
      }
      
      int64_t end_time = esp_timer_get_time();
      int64_t encode_duration = end_time - start_time;
      ESP_LOGV(TAG,
               "CH101 iq message encoded successfully, size: %zu bytes, time: %lld us",
               stream.bytes_written, encode_duration);
        
      hand_overwrite_buf_bytes_count(iq_buffer, stream.bytes_written);
      
      // Timing start - sending data
      start_time = esp_timer_get_time();
      
      int ret = send(client_socket, iq_buffer, stream.bytes_written, 0);
      
      if (ret < 0)
      {
        ESP_LOGE(TAG, "Error occurred during sending CH101 msgs: errno %d", errno);
      }
      else
      {
        ESP_LOGV(TAG, "Message sent (CH101) successfully");
      }
      
      end_time = esp_timer_get_time();
      int64_t transmit_duration = end_time - start_time;
      ESP_LOGV(TAG, "CH101 Data transmitted successfully, time: %lld us", transmit_duration);
    }          
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(HAND_MS_CH101_SEND_IQ_DATA));
  }
}

void hand_task_alive(void* arg)
{
  /* XXX: move to global states */
  static bool led_on = true;

  /* light up RGB LED blue */
  led_strip_set_pixel(hand_global_devs_handle.rgb_led_handle,
                      HAND_RGB_LED_INDEX, 0, 8, 16);
  led_strip_refresh(hand_global_devs_handle.rgb_led_handle);

  while (1)
  {
    if (xEventGroupGetBits(hand_global_system_event_group) &
        HAND_EG_SYSTEM_LED_CONTROL_BY_ALIVE)
    {
      if (led_on)
      {
        led_strip_clear(hand_global_devs_handle.rgb_led_handle);
        led_strip_refresh(hand_global_devs_handle.rgb_led_handle);
      }
      else
      {
        /* TODO: create hand_led lib */
        led_strip_set_pixel(hand_global_devs_handle.rgb_led_handle,
                            HAND_RGB_LED_INDEX, 0, 8, 16);
        led_strip_refresh(hand_global_devs_handle.rgb_led_handle);
      }
      led_on = !led_on;
    }

    /* access led to blink */
    vTaskDelay(pdMS_TO_TICKS(HAND_MS_ALIVE_BLINK_DELAY / 2));
  }
}