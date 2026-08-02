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

static void _hand_ch101_handle_data_ready(ch_group_t* grp_ptr)
{
  hand_chx01_simple_data_element_t simple_data = {0};
  hand_chx01_amp_data_element_t amp_data = {0};
  hand_chx01_iq_data_element_t iq_data = {0};

  uint8_t dev_num;
  int num_samples = 0;
  uint8_t amp_error = 0;
  uint8_t iq_error = 0;

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
        range = ch_get_range(dev_ptr, CH_RANGE_DIRECT);
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

        if (range == CH_NO_TARGET)
        {
          simple_data.simple_data[dev_num].range = 0;
          simple_data.simple_data[dev_num].amp = 0;
          
          // DEBUG only
          ESP_LOGI(TAG, "Port {%d} target not found", dev_num);
        }
      }

      simple_data.simple_data[dev_num].range = range / 32.0f;
      simple_data.simple_data[dev_num].amp = ch_get_amplitude(dev_ptr);
      


      num_samples = ch_get_num_samples(dev_ptr);
      simple_data.simple_data[dev_num].sample_num = num_samples;
      
      amp_error = ch_get_amplitude_data(dev_ptr, 
                                        amp_data.amp_data[dev_num].amp_data, 
                                        0, 
                                        num_samples, 
                                        CH_IO_MODE_BLOCK);
      
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
        float power_peak = 0.0f;
        uint16_t peak_sample_idx = 0;

        // Step A: Calculate Instantaneous Magnitude from raw I and Q samples
        for (int k = 0; k < num_samples; ++k)
        {
          int16_t i = iq_data.iq_data[dev_num].iq_data[k].i;
          int16_t q = iq_data.iq_data[dev_num].iq_data[k].q;

          // Mag = sqrt(I^2 + Q^2)
          float power = sqrtf((float)(i * i + q * q));

          if (power > power_peak)
          {
            power_peak = power;
            peak_sample_idx = k;
          }
        }

        // Step B: Incorporate AH101-180180 Physical Coupling Factor
        float effective_signal_amplitude = power_peak * HAND_CH101_SCORING_ACOUSTIC_COUPLING_ETA * HAND_CH101_SCORING_MASK_GAIN_FACTOR;

        // Step C: Compute Signal-to-Noise Ratio (SNR in dB)
        float snr_linear = effective_signal_amplitude / HAND_CH101_SCORING_BASELINE_NOISE_FLOOR;
        if (snr_linear < 1e-3f) snr_linear = 1e-3f; // Prevent log(0)
        
        float snr_db = 20.0f * log10f(snr_linear);

        // Step D: Calculate Confidence Score based on SNR Margin
        float m_snr = (snr_db - HAND_CH101_SCORING_SNR_TRESHOLD_MIN) / (HAND_CH101_SCORING_SNR_TRESHOLD_TARGET - HAND_CH101_SCORING_SNR_TRESHOLD_MIN);
        
        // Clamp score between [0.0, 1.0]
        float confidence_score = m_snr;
        if (confidence_score < 0.0f) confidence_score = 0.0f;
        if (confidence_score > 1.0f) confidence_score = 1.0f;

        // Step E: Side-Lobe Artifact Rejection Gate
        // If SNR is below minimum usable threshold, suppress target as side-lobe diffraction
        if (snr_db < HAND_CH101_SCORING_SNR_TRESHOLD_MIN)
        {
          ESP_LOGW(TAG, "Port {%d}: Suppressed Side-Lobe Artifact at %d mm (SNR: %.2f dB, Score: %.2f)",
                   dev_num, simple_data.simple_data[dev_num].range, snr_db, confidence_score);
          
          simple_data.simple_data[dev_num].range = 0;
          simple_data.simple_data[dev_num].amp = 0;
        }
        else
        {
          ESP_LOGI(TAG, "Port {%d}: Valid Target at %.1f mm | Peak IQ Amp: %.1f LSB | Effective Amp: %.1f LSB | SNR: %.2f dB | Score: %.2f",
                   dev_num, simple_data.simple_data[dev_num].range, power_peak, effective_signal_amplitude, snr_db, confidence_score);
        }
      } 
      else
      {
        ESP_LOGI(TAG, "CH101 Read iq error exists...");
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

    static uint8_t current_tx_dev = 0;
    static uint8_t pending_candidate_tx = 0;
    static uint8_t candidate_hold_count = 0;
  
    float sensor_scores[HAND_DEV_MAX_NUM_CH101] = {0.0f};
    uint8_t num_ports = ch_get_num_ports(grp_ptr);
    uint8_t best_candidate_tx = current_tx_dev;
    float max_score = -1.0f;
  
    for (uint8_t dev = 0; dev < num_ports; dev++)
    {
      ch_dev_t* dev_ptr = ch_get_dev_ptr(grp_ptr, dev);
      if (!ch_sensor_is_connected(dev_ptr)) continue;
  
      // 1. Amplitude Peak Evaluation
      float amp_score = (float)simple_data.simple_data[dev].amp;
  
      // 2. Compute Total Acoustic Energy & Peak SNR from Raw IQ Data
      float total_iq_energy = 0.0f;
      float peak_iq_power = 0.0f;
      uint16_t samples = simple_data.simple_data[dev].sample_num;
  
      for (uint16_t s = 0; s < samples; s++)
      {
        int16_t i = iq_data.iq_data[dev].iq_data[s].i;
        int16_t q = iq_data.iq_data[dev].iq_data[s].q;
        float power = (float)(i * i + q * q);
        
        total_iq_energy += power;
        if (power > peak_iq_power)
        {
          peak_iq_power = power;
        }
      }
  
      float snr_linear = (peak_iq_power > 0.0f) ? (peak_iq_power / HAND_CH101_SWITCH_BASELINE_NOISE_FLOOR) : 0.0f;
  
      // 3. Composite Theoretical Score Formulation: S = w1*Amp + w2*Energy + w3*SNR
      sensor_scores[dev] = (0.5f * amp_score) + 
                           (0.3f * sqrtf(total_iq_energy)) + 
                           (0.2f * snr_linear);
  
      if (sensor_scores[dev] > max_score)
      {
        max_score = sensor_scores[dev];
        best_candidate_tx = dev;
      }
    }
  
    // Hysteresis Decision Logic to handle out-of-beam attenuation and temporal jitter
    if (best_candidate_tx != current_tx_dev)
    {
      float current_tx_score = sensor_scores[current_tx_dev];
      
      // Check if new candidate exceeds current Tx score by hysteresis threshold
      if ((max_score - current_tx_score) > HAND_CH101_SWITCH_AMP_TRESHOLD_MIN)
      {
        if (best_candidate_tx == pending_candidate_tx)
        {
          candidate_hold_count++;
        }
        else
        {
          pending_candidate_tx = best_candidate_tx;
          candidate_hold_count = 1;
        }
  
        // Execute dynamic mode reconfiguration when target stability criteria is met
        if (candidate_hold_count >= HAND_CH101_SWITCH_STABILITY_HOLD_CYCLE)
        {
          ESP_LOGW(TAG, "DYNAMIC MODE SWITCH: Changing Primary Tx from Port %d to Port %d (Score Gain: %.1f)", 
                   current_tx_dev, best_candidate_tx, max_score - current_tx_score);
  
          // Reconfigure CH101 modes dynamically
          for (uint8_t dev = 0; dev < num_ports; dev++)
          {
            ch_dev_t* dev_ptr = ch_get_dev_ptr(grp_ptr, dev);
            if (!ch_sensor_is_connected(dev_ptr)) continue;
  
            if (dev == best_candidate_tx)
            {
              ch_set_mode(dev_ptr, CH_MODE_TRIGGERED_TX_RX);
            }
            else
            {
              ch_set_mode(dev_ptr, CH_MODE_TRIGGERED_RX_ONLY);
            }
          }
  
          // Update active tracker and global dev reference
          current_tx_dev = best_candidate_tx;
          hand_global_ch101_active_dev_num = current_tx_dev;
          candidate_hold_count = 0;
        }
      }
      else
      {
        candidate_hold_count = 0;
      }
    }
    else
    {
      candidate_hold_count = 0;
    }
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