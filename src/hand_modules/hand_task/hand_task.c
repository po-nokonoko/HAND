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
#include <stdint.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// for for loop using
#define HAND_MSG_SOURCE_CH101_BASE (HandChipInstance_CH101_SENSOR1)

static const char* TAG = "HAND_TASK";

/* CH101 quality/switching state is file-local because it is sensor-control state,
 * not a per-frame temporary. */
#define HAND_CH101_INVALID_DEV (UINT8_MAX)

enum
{
  HAND_CH101_TRACE_SAMPLE_CAPACITY =
      sizeof(((hand_chx01_iq_data_unit_t*)0)->iq_data) /
      sizeof(ch_iq_sample_t)
};

typedef struct hand_ch101_quality_t
{
  bool selected;
  bool connected;
  bool range_valid;
  bool iq_valid;
  bool directivity_valid;
  ch_mode_t mode;
  float range_mm;
  float target_amplitude;
  float noise_power;
  float signal_power;
  float snr_linear;
  float snr_db;
  float q_snr;
  float compensated_amplitude;
  float directivity_level_db;
  float q_fov;
  float q_range;
  float quality;
} hand_ch101_quality_t;

typedef struct hand_ch101_range_history_t
{
  ch_mode_t mode;
  float previous_mm;
  float latest_mm;
  uint8_t count;
} hand_ch101_range_history_t;

static uint8_t hand_ch101_current_tx_dev = HAND_CH101_INVALID_DEV;
static uint8_t hand_ch101_pending_tx_dev = HAND_CH101_INVALID_DEV;
static uint8_t hand_ch101_pending_tx_count = 0;
static hand_ch101_range_history_t
    hand_ch101_range_history[HAND_DEV_MAX_NUM_CH101] = {0};

static float _hand_ch101_clampf(float value, float lower, float upper)
{
  if (value < lower) return lower;
  if (value > upper) return upper;
  return value;
}

static bool _hand_ch101_is_selected(uint8_t dev_num)
{
  return (dev_num < HAND_DEV_MAX_NUM_CH101) &&
         ((HAND_CH101_QUALITY_DEV_MASK & (1U << dev_num)) != 0U);
}

static float _hand_ch101_iq_power(const ch_iq_sample_t* sample)
{
  const float i = (float)sample->i;
  const float q = (float)sample->q;
  return (i * i) + (q * q);
}

static void _hand_ch101_sort_float(float* values, uint16_t count)
{
  for (uint16_t i = 1; i < count; ++i)
  {
    const float key = values[i];
    uint16_t j = i;

    while ((j > 0U) && (values[j - 1U] > key))
    {
      values[j] = values[j - 1U];
      --j;
    }
    values[j] = key;
  }
}

static float _hand_ch101_median_noise_power(const ch_iq_sample_t* iq,
                                             uint16_t start_sample,
                                             uint16_t num_samples,
                                             float* scratch)
{
  if ((iq == NULL) || (scratch == NULL) || (start_sample >= num_samples))
  {
    return NAN;
  }

  const uint16_t count = num_samples - start_sample;
  for (uint16_t i = 0; i < count; ++i)
  {
    scratch[i] = _hand_ch101_iq_power(&iq[start_sample + i]);
  }

  _hand_ch101_sort_float(scratch, count);

  if ((count & 1U) != 0U)
  {
    return scratch[count / 2U];
  }

  return 0.5f * (scratch[(count / 2U) - 1U] + scratch[count / 2U]);
}

static void _hand_ch101_reset_range_history(void)
{
  for (uint8_t dev = 0; dev < HAND_DEV_MAX_NUM_CH101; ++dev)
  {
    hand_ch101_range_history[dev].mode = CH_MODE_IDLE;
    hand_ch101_range_history[dev].previous_mm = 0.0f;
    hand_ch101_range_history[dev].latest_mm = 0.0f;
    hand_ch101_range_history[dev].count = 0U;
  }
}

static float _hand_ch101_range_continuity_score(uint8_t dev_num,
                                                 ch_mode_t mode,
                                                 bool range_valid,
                                                 float range_mm)
{
  if ((dev_num >= HAND_DEV_MAX_NUM_CH101) || !range_valid ||
      !isfinite(range_mm) || (range_mm <= 0.0f))
  {
    if (dev_num < HAND_DEV_MAX_NUM_CH101)
    {
      hand_ch101_range_history[dev_num].count = 0U;
    }
    return 0.0f;
  }

  hand_ch101_range_history_t* const history =
      &hand_ch101_range_history[dev_num];

  if ((history->count == 0U) || (history->mode != mode))
  {
    history->mode = mode;
    history->previous_mm = range_mm;
    history->latest_mm = range_mm;
    history->count = 1U;
    return 1.0f;
  }

  if (history->count == 1U)
  {
    history->previous_mm = history->latest_mm;
    history->latest_mm = range_mm;
    history->count = 2U;
    return 1.0f;
  }

  float predicted_mm =
      history->latest_mm + (history->latest_mm - history->previous_mm);
  if (!isfinite(predicted_mm) || (predicted_mm <= 0.0f))
  {
    predicted_mm = history->latest_mm;
  }

  const float lower = fminf(range_mm, predicted_mm);
  const float upper = fmaxf(range_mm, predicted_mm);
  const float score = (upper > 0.0f) ? (lower / upper) : 0.0f;

  history->previous_mm = history->latest_mm;
  history->latest_mm = range_mm;

  return _hand_ch101_clampf(score, 0.0f, 1.0f);
}

static uint8_t _hand_ch101_port_count(ch_group_t* grp_ptr)
{
  if (grp_ptr == NULL) return 0U;

  const uint8_t reported = ch_get_num_ports(grp_ptr);
  return (reported < HAND_DEV_MAX_NUM_CH101)
             ? reported
             : HAND_DEV_MAX_NUM_CH101;
}

static uint8_t _hand_ch101_find_tx(ch_group_t* grp_ptr, uint8_t* tx_count)
{
  uint8_t found = HAND_CH101_INVALID_DEV;
  *tx_count = 0U;

  const uint8_t num_ports = _hand_ch101_port_count(grp_ptr);
  for (uint8_t dev = 0; dev < num_ports; ++dev)
  {
    ch_dev_t* const dev_ptr = ch_get_dev_ptr(grp_ptr, dev);
    if (ch_sensor_is_connected(dev_ptr) &&
        (ch_get_mode(dev_ptr) == CH_MODE_TRIGGERED_TX_RX))
    {
      found = dev;
      ++(*tx_count);
    }
  }

  return found;
}

static void _hand_ch101_recompute_triggered_count(ch_group_t* grp_ptr)
{
  uint8_t count = 0U;
  const uint8_t num_ports = _hand_ch101_port_count(grp_ptr);

  for (uint8_t dev = 0; dev < num_ports; ++dev)
  {
    ch_dev_t* const dev_ptr = ch_get_dev_ptr(grp_ptr, dev);
    if (!ch_sensor_is_connected(dev_ptr)) continue;

    const ch_mode_t mode = ch_get_mode(dev_ptr);
    if ((mode == CH_MODE_TRIGGERED_TX_RX) ||
        (mode == CH_MODE_TRIGGERED_RX_ONLY))
    {
      ++count;
    }
  }

  hand_global_ch101_triggered_dev_num = count;
}

static bool _hand_ch101_switch_tx(ch_group_t* grp_ptr, uint8_t next_tx_dev)
{
  const uint8_t num_ports = _hand_ch101_port_count(grp_ptr);
  ch_mode_t previous_modes[HAND_DEV_MAX_NUM_CH101] = {CH_MODE_IDLE};
  bool connected[HAND_DEV_MAX_NUM_CH101] = {false};
  bool mode_write_failed = false;

  if ((next_tx_dev >= num_ports) || !_hand_ch101_is_selected(next_tx_dev))
  {
    return false;
  }

  ch_dev_t* const next_tx_ptr = ch_get_dev_ptr(grp_ptr, next_tx_dev);
  if (!ch_sensor_is_connected(next_tx_ptr)) return false;

  for (uint8_t dev = 0; dev < num_ports; ++dev)
  {
    ch_dev_t* const dev_ptr = ch_get_dev_ptr(grp_ptr, dev);
    connected[dev] = ch_sensor_is_connected(dev_ptr);
    if (connected[dev]) previous_modes[dev] = ch_get_mode(dev_ptr);
  }

  /* Demote every other connected node first.  This prevents a transient state
   * in which two devices transmit the same ping. */
  for (uint8_t dev = 0; dev < num_ports; ++dev)
  {
    if (!connected[dev] || (dev == next_tx_dev)) continue;

    ch_dev_t* const dev_ptr = ch_get_dev_ptr(grp_ptr, dev);
    if ((ch_get_mode(dev_ptr) != CH_MODE_TRIGGERED_RX_ONLY) &&
        (ch_set_mode(dev_ptr, CH_MODE_TRIGGERED_RX_ONLY) != RET_OK))
    {
      mode_write_failed = true;
      break;
    }
  }

  if (!mode_write_failed &&
      (ch_get_mode(next_tx_ptr) != CH_MODE_TRIGGERED_TX_RX) &&
      (ch_set_mode(next_tx_ptr, CH_MODE_TRIGGERED_TX_RX) != RET_OK))
  {
    mode_write_failed = true;
  }

  uint8_t verified_tx_count = 0U;
  uint8_t verified_tx_dev = HAND_CH101_INVALID_DEV;
  if (!mode_write_failed)
  {
    verified_tx_dev = _hand_ch101_find_tx(grp_ptr, &verified_tx_count);
    mode_write_failed =
        (verified_tx_count != 1U) || (verified_tx_dev != next_tx_dev);
  }

  if (mode_write_failed)
  {
    bool rollback_failed = false;
    for (uint8_t dev = 0; dev < num_ports; ++dev)
    {
      if (!connected[dev]) continue;

      ch_dev_t* const dev_ptr = ch_get_dev_ptr(grp_ptr, dev);
      if ((ch_get_mode(dev_ptr) != previous_modes[dev]) &&
          (ch_set_mode(dev_ptr, previous_modes[dev]) != RET_OK))
      {
        rollback_failed = true;
      }
    }

    _hand_ch101_recompute_triggered_count(grp_ptr);
    ESP_LOGE(TAG,
             "CH101 TX switch to dev %u failed; mode rollback %s",
             (unsigned int)next_tx_dev,
             rollback_failed ? "FAILED" : "completed");
    return false;
  }

  _hand_ch101_recompute_triggered_count(grp_ptr);
  _hand_ch101_reset_range_history();
  ESP_LOGW(TAG, "CH101 TX/RX switched to dev %u",
           (unsigned int)next_tx_dev);
  return true;
}


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
  hand_ch101_quality_t quality[HAND_DEV_MAX_NUM_CH101] = {0};
  float power_scratch[HAND_CH101_TRACE_SAMPLE_CAPACITY] = {0.0f};

  if (grp_ptr == NULL) return;

  const uint8_t reported_num_ports = ch_get_num_ports(grp_ptr);
  const uint8_t num_ports =
      (reported_num_ports < HAND_DEV_MAX_NUM_CH101)
          ? reported_num_ports
          : HAND_DEV_MAX_NUM_CH101;
  const int64_t cycle_timestamp = esp_timer_get_time();

  /* SonicLib requires waveform I/O to finish before the next trigger.  The
   * timer is restarted after all blocking reads, scoring, mode writes and queue
   * operations are complete. */
  chbsp_periodic_timer_stop();

  for (uint8_t dev = 0; dev < HAND_DEV_MAX_NUM_CH101; ++dev)
  {
    simple_data.simple_data[dev].timestamps = cycle_timestamp;
    simple_data.simple_data[dev].range = NAN;
  }

  if (reported_num_ports > HAND_DEV_MAX_NUM_CH101)
  {
    ESP_LOGE(TAG, "CH101 group reports %u ports, but HAND supports %u",
             (unsigned int)reported_num_ports,
             (unsigned int)HAND_DEV_MAX_NUM_CH101);
  }

  uint8_t tx_count = 0U;
  uint8_t actual_tx_dev = _hand_ch101_find_tx(grp_ptr, &tx_count);
  if ((tx_count == 1U) && (actual_tx_dev != hand_ch101_current_tx_dev))
  {
    hand_ch101_current_tx_dev = actual_tx_dev;
    hand_ch101_pending_tx_dev = HAND_CH101_INVALID_DEV;
    hand_ch101_pending_tx_count = 0U;
    _hand_ch101_reset_range_history();
  }

  for (uint8_t dev = 0; dev < num_ports; ++dev)
  {
    ch_dev_t* const dev_ptr = ch_get_dev_ptr(grp_ptr, dev);
    hand_ch101_quality_t* const metric = &quality[dev];

    metric->selected = _hand_ch101_is_selected(dev);
    metric->connected = ch_sensor_is_connected(dev_ptr);
    metric->mode = ch_get_mode(dev_ptr);
    metric->range_mm = NAN;
    metric->snr_db = -INFINITY;
    metric->directivity_level_db = -INFINITY;

    if (!metric->connected) continue;

    /* dev1 is intentionally excluded from range quality, waveform transfer and
     * TX candidacy.  It remains in the SonicLib group so its interrupt still
     * participates in the existing all-connected synchronization mask. */
    if (!metric->selected)
    {
      ESP_LOGD(TAG, "CH101 dev %u excluded by quality mask 0x%02X",
               (unsigned int)dev,
               (unsigned int)HAND_CH101_QUALITY_DEV_MASK);
      continue;
    }

    ch_range_t range_type;
    if ((metric->mode == CH_MODE_TRIGGERED_TX_RX) ||
        (metric->mode == CH_MODE_FREERUN))
    {
      range_type = CH_RANGE_ECHO_ONE_WAY;
    }
    else if (metric->mode == CH_MODE_TRIGGERED_RX_ONLY)
    {
      /* CH_RANGE_DIRECT is SonicLib's documented selector for an RX-only
       * pitch-catch node.  In this GPR implementation it preserves the full,
       * undivided ToF path; a direct inter-sensor arrival is rejected later by
       * the bistatic-path consistency check d_i > r_T. */
      range_type = CH_RANGE_DIRECT;
    }
    else
    {
      ESP_LOGW(TAG, "CH101 dev %u is not in a ranging mode (0x%02X)",
               (unsigned int)dev, (unsigned int)metric->mode);
      continue;
    }

    const uint32_t raw_range = ch_get_range(dev_ptr, range_type);
    metric->range_valid = (raw_range != CH_NO_TARGET) && (raw_range != 0U);
    if (metric->range_valid)
    {
      metric->range_mm = (float)raw_range / 32.0f;
      metric->range_valid = isfinite(metric->range_mm) &&
                            (metric->range_mm > 0.0f);
    }

    const bool range_in_quality_region =
        metric->range_valid &&
        (metric->range_mm >= HAND_CH101_QUALITY_MIN_VALID_RANGE_MM);

    if (metric->range_valid)
    {
      const uint16_t target_amp = ch_get_amplitude(dev_ptr);
      simple_data.simple_data[dev].range = metric->range_mm;
      simple_data.simple_data[dev].amp = target_amp;
      metric->target_amplitude = (float)target_amp;
    }
    else
    {
      /* ch_get_amplitude() is deliberately not called after CH_NO_TARGET,
       * because SonicLib leaves the previous successful amplitude in place. */
      simple_data.simple_data[dev].range = NAN;
      simple_data.simple_data[dev].amp = 0U;
      metric->target_amplitude = 0.0f;
      ESP_LOGD(TAG, "CH101 dev %u: target not found",
               (unsigned int)dev);
    }

    uint16_t num_samples = ch_get_num_samples(dev_ptr);
    if (num_samples > HAND_CH101_TRACE_SAMPLE_CAPACITY)
    {
      ESP_LOGW(TAG,
               "CH101 dev %u sample count %u exceeds trace capacity %u; clipping",
               (unsigned int)dev, (unsigned int)num_samples,
               (unsigned int)HAND_CH101_TRACE_SAMPLE_CAPACITY);
      num_samples = HAND_CH101_TRACE_SAMPLE_CAPACITY;
    }

    const uint8_t iq_error =
        ch_get_iq_data(dev_ptr, iq_data.iq_data[dev].iq_data, 0U,
                       num_samples, CH_IO_MODE_BLOCK);
    metric->iq_valid = (iq_error == RET_OK) && (num_samples > 0U);

    if (!metric->iq_valid)
    {
      simple_data.simple_data[dev].sample_num = 0U;
      ESP_LOGE(TAG, "CH101 dev %u IQ read failed: %u",
               (unsigned int)dev, (unsigned int)iq_error);
      metric->q_range = _hand_ch101_range_continuity_score(
          dev, metric->mode, false, metric->range_mm);
      continue;
    }

    simple_data.simple_data[dev].sample_num = num_samples;

    /* GPR Open implements ch_get_amplitude_data() by reading the same IQ trace
     * and applying sqrt(I^2+Q^2).  Deriving AMP here keeps AMP/IQ sample-aligned
     * and avoids a second full I2C transfer. */
    for (uint16_t sample = 0; sample < num_samples; ++sample)
    {
      amp_data.amp_data[dev].amp_data[sample] =
          ch_iq_to_amplitude(&iq_data.iq_data[dev].iq_data[sample]);
    }

    uint16_t noise_start = ch_mm_to_samples(
        dev_ptr, HAND_CH101_QUALITY_RINGDOWN_EXCLUSION_MM);
    if ((noise_start >= num_samples) ||
        ((uint16_t)(num_samples - noise_start) <
         (uint16_t)HAND_CH101_QUALITY_MIN_NOISE_SAMPLES))
    {
      ESP_LOGW(TAG,
               "CH101 dev %u has insufficient post-ring-down samples for quality scoring",
               (unsigned int)dev);
      metric->q_range = _hand_ch101_range_continuity_score(
          dev, metric->mode, metric->range_valid, metric->range_mm);
      continue;
    }

    metric->noise_power = _hand_ch101_median_noise_power(
        iq_data.iq_data[dev].iq_data, noise_start, num_samples,
        power_scratch);

    if (!isfinite(metric->noise_power) || (metric->noise_power < 1.0f))
    {
      /* One squared ADC count is the smallest representable non-zero power;
       * this is a numerical guard, not a calibrated hardware noise floor. */
      metric->noise_power = 1.0f;
    }

    if (range_in_quality_region && (metric->target_amplitude > 0.0f))
    {
      const float target_power =
          metric->target_amplitude * metric->target_amplitude;
      metric->signal_power = fmaxf(target_power - metric->noise_power, 0.0f);
      metric->snr_linear = metric->signal_power / metric->noise_power;
      metric->snr_db = (metric->snr_linear > 0.0f)
                           ? 10.0f * log10f(metric->snr_linear)
                           : -INFINITY;
      metric->q_snr = metric->signal_power /
                      (metric->signal_power + metric->noise_power);
    }

    metric->q_range = _hand_ch101_range_continuity_score(
        dev, metric->mode, range_in_quality_region, metric->range_mm);
  }

  /* Part 2: relative FoV/directivity confidence.  The report states
   * I ~ 1/(r_T^2 r_i^2), hence received amplitude scales to first order as
   * 1/(r_T r_i).  The multiplication below removes only that leading spreading
   * term; target reflectivity, atmospheric absorption and sensor-to-sensor gain
   * remain uncalibrated and are intentionally not fabricated here. */
  const bool tx_range_available =
      (tx_count == 1U) && (actual_tx_dev < num_ports) &&
      quality[actual_tx_dev].selected &&
      quality[actual_tx_dev].range_valid &&
      (quality[actual_tx_dev].range_mm >=
       HAND_CH101_QUALITY_MIN_VALID_RANGE_MM);
  const float tx_range_mm = tx_range_available
                                ? quality[actual_tx_dev].range_mm
                                : NAN;
  float max_compensated_amplitude = 0.0f;
  bool raw_directivity_fallback = !tx_range_available;

  for (uint8_t dev = 0; dev < num_ports; ++dev)
  {
    hand_ch101_quality_t* const metric = &quality[dev];
    if (!metric->selected || !metric->connected || !metric->range_valid ||
        (metric->range_mm < HAND_CH101_QUALITY_MIN_VALID_RANGE_MM) ||
        (metric->target_amplitude <= 0.0f))
    {
      continue;
    }

    if (tx_range_available)
    {
      if ((dev == actual_tx_dev) &&
          (metric->mode == CH_MODE_TRIGGERED_TX_RX))
      {
        metric->compensated_amplitude =
            metric->target_amplitude * tx_range_mm * tx_range_mm;
        metric->directivity_valid = true;
      }
      else if (metric->mode == CH_MODE_TRIGGERED_RX_ONLY)
      {
        const float receiver_leg_mm = metric->range_mm - tx_range_mm;
        if (receiver_leg_mm > 0.0f)
        {
          metric->compensated_amplitude =
              metric->target_amplitude * tx_range_mm * receiver_leg_mm;
          metric->directivity_valid = true;
        }
        else
        {
          /* d_i = r_T + r_i must be greater than r_T. */
          ESP_LOGW(TAG,
                   "CH101 dev %u rejected: bistatic path %.2f mm <= TX leg %.2f mm",
                   (unsigned int)dev, metric->range_mm, tx_range_mm);
          simple_data.simple_data[dev].range = NAN;
          metric->range_valid = false;
          metric->q_range = 0.0f;
        }
      }
    }
    else
    {
      /* Reacquisition fallback when the current TX has no valid range.  Raw
       * amplitude is used only to choose a possible new TX and is logged. */
      metric->compensated_amplitude = metric->target_amplitude;
      metric->directivity_valid = true;
    }

    if (metric->directivity_valid &&
        (metric->compensated_amplitude > max_compensated_amplitude))
    {
      max_compensated_amplitude = metric->compensated_amplitude;
    }
  }

  const float half_power_ratio =
      powf(10.0f, HAND_CH101_FOV_HALF_POWER_DB / 20.0f);

  for (uint8_t dev = 0; dev < num_ports; ++dev)
  {
    hand_ch101_quality_t* const metric = &quality[dev];
    if (metric->directivity_valid &&
        (max_compensated_amplitude > 0.0f))
    {
      const float ratio = _hand_ch101_clampf(
          metric->compensated_amplitude / max_compensated_amplitude,
          0.0f, 1.0f);
      metric->directivity_level_db =
          (ratio > 0.0f) ? 20.0f * log10f(ratio) : -INFINITY;
      metric->q_fov = (ratio >= half_power_ratio)
                          ? 1.0f
                          : (ratio / half_power_ratio);
    }

    metric->quality = _hand_ch101_clampf(
        metric->q_snr * metric->q_fov * metric->q_range, 0.0f, 1.0f);

    if (metric->selected && metric->connected)
    {
      ESP_LOGI(
          TAG,
          "CH101 q dev=%u mode=0x%02X range=%.2fmm amp=%.0f noise=%.1f "
          "snr=%.2fdB q_snr=%.3f level=%.2fdB q_fov=%.3f "
          "q_range=%.3f q=%.3f%s",
          (unsigned int)dev, (unsigned int)metric->mode, metric->range_mm,
          metric->target_amplitude,
          metric->noise_power, metric->snr_db, metric->q_snr,
          metric->directivity_level_db, metric->q_fov, metric->q_range,
          metric->quality, raw_directivity_fallback ? " [RAW-FALLBACK]" : "");
    }

#if HAND_CH101_REJECT_BELOW_HALF_POWER
    if (metric->range_valid && metric->directivity_valid &&
        (metric->directivity_level_db < HAND_CH101_FOV_HALF_POWER_DB))
    {
      simple_data.simple_data[dev].range = NAN;
    }
#endif
  }

  uint8_t best_candidate = HAND_CH101_INVALID_DEV;
  float best_quality = -1.0f;
  for (uint8_t dev = 0; dev < num_ports; ++dev)
  {
    const hand_ch101_quality_t* const metric = &quality[dev];
    if (!metric->selected || !metric->connected || !metric->range_valid ||
        !metric->iq_valid || !metric->directivity_valid ||
        (metric->directivity_level_db < HAND_CH101_FOV_HALF_POWER_DB) ||
        (metric->quality <= 0.0f))
    {
      continue;
    }

    if (metric->quality > best_quality)
    {
      best_quality = metric->quality;
      best_candidate = dev;
    }
  }

  /* A malformed mode vector (zero or multiple transmitters) is repaired
   * immediately.  Normal beam-driven switching still uses the persistence
   * counter below. */
  if (tx_count != 1U)
  {
    uint8_t repair_candidate = best_candidate;
    if (repair_candidate == HAND_CH101_INVALID_DEV)
    {
      for (uint8_t dev = 0; dev < num_ports; ++dev)
      {
        ch_dev_t* const dev_ptr = ch_get_dev_ptr(grp_ptr, dev);
        if (_hand_ch101_is_selected(dev) && ch_sensor_is_connected(dev_ptr))
        {
          repair_candidate = dev;
          break;
        }
      }
    }

    ESP_LOGE(TAG, "CH101 mode invariant violated: found %u TX/RX devices",
             (unsigned int)tx_count);
    if ((repair_candidate != HAND_CH101_INVALID_DEV) &&
        _hand_ch101_switch_tx(grp_ptr, repair_candidate))
    {
      hand_ch101_current_tx_dev = repair_candidate;
    }
    hand_ch101_pending_tx_dev = HAND_CH101_INVALID_DEV;
    hand_ch101_pending_tx_count = 0U;
  }
  else
  {
    hand_ch101_current_tx_dev = actual_tx_dev;

    if ((best_candidate != HAND_CH101_INVALID_DEV) &&
        (best_candidate != hand_ch101_current_tx_dev))
    {
      const hand_ch101_quality_t* const current =
          &quality[hand_ch101_current_tx_dev];
      const bool current_usable = current->selected && current->connected &&
                                  current->range_valid && current->iq_valid &&
                                  (current->quality > 0.0f);
      const bool current_below_half_power =
          !current_usable || !current->directivity_valid ||
          (current->directivity_level_db < HAND_CH101_FOV_HALF_POWER_DB);
      const bool candidate_better =
          best_quality > (current->quality + HAND_CH101_SWITCH_SCORE_MARGIN);

      if (current_below_half_power && candidate_better)
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

        ESP_LOGW(TAG,
                 "CH101 TX candidate dev %u: hold %u/%u, q %.3f > %.3f",
                 (unsigned int)best_candidate,
                 (unsigned int)hand_ch101_pending_tx_count,
                 (unsigned int)HAND_CH101_SWITCH_STABILITY_HOLD_CYCLE,
                 best_quality,
                 current->quality);

        if (hand_ch101_pending_tx_count >=
            HAND_CH101_SWITCH_STABILITY_HOLD_CYCLE)
        {
          if (_hand_ch101_switch_tx(grp_ptr, best_candidate))
          {
            hand_ch101_current_tx_dev = best_candidate;
          }
          hand_ch101_pending_tx_dev = HAND_CH101_INVALID_DEV;
          hand_ch101_pending_tx_count = 0U;
        }
      }
      else
      {
        hand_ch101_pending_tx_dev = HAND_CH101_INVALID_DEV;
        hand_ch101_pending_tx_count = 0U;
      }
    }
    else
    {
      hand_ch101_pending_tx_dev = HAND_CH101_INVALID_DEV;
      hand_ch101_pending_tx_count = 0U;
    }
  }

  ESP_LOGI(TAG, "range: %.3f, %.3f, %.3f, %.3f",
           simple_data.simple_data[0].range,
           simple_data.simple_data[1].range,
           simple_data.simple_data[2].range,
           simple_data.simple_data[3].range);
  ESP_LOGI(TAG, "amp: %u, %u, %u, %u",
           (unsigned int)simple_data.simple_data[0].amp,
           (unsigned int)simple_data.simple_data[1].amp,
           (unsigned int)simple_data.simple_data[2].amp,
           (unsigned int)simple_data.simple_data[3].amp);

  /* All sensor I/O and mode writes are complete.  Resume triggering before
   * software queueing so a congested consumer cannot stretch the acoustic
   * measurement interval. */
  chbsp_periodic_timer_start();

  if (xQueueSend(hand_global_ch101_simple_data_queue, &simple_data,
                 pdMS_TO_TICKS(HAND_MS_CH101_QUEUE_MAX_DELAY)) != pdTRUE)
  {
    ESP_LOGE(TAG, "CH101 simple-data queue is full");
  }
  if (xQueueSend(hand_global_ch101_amp_data_queue, &amp_data,
                 pdMS_TO_TICKS(HAND_MS_CH101_QUEUE_MAX_DELAY)) != pdTRUE)
  {
    ESP_LOGE(TAG, "CH101 AMP-data queue is full");
  }
  if (xQueueSend(hand_global_ch101_iq_data_queue, &iq_data,
                 pdMS_TO_TICKS(HAND_MS_CH101_QUEUE_MAX_DELAY)) != pdTRUE)
  {
    ESP_LOGE(TAG, "CH101 IQ-data queue is full");
  }

}

void hand_task_ch101_collect_data(void* __attribute__((unused)) arg)
{
  ch_group_t* const grp_ptr = &hand_global_devs_handle.ch101_group;

  chbsp_periodic_timer_init(HAND_MS_CH101_DEFAULT_MEASURE_PERIOD,
                            hand_cb_ch101_periodic_timer);

  /* RX pre-trigger improves very-short-range pitch-catch capture but reduces
   * RX-only maximum range by about 200 mm.  The current far-fingertip priority
   * therefore disables it explicitly before measurements begin. */
  ch_set_rx_pretrigger(grp_ptr, HAND_CH101_RX_PRETRIGGER_ENABLE);
  chbsp_periodic_timer_start();

  ESP_LOGI(TAG, "CH101 measurement is starting (RX pre-trigger=%u)",
           (unsigned int)HAND_CH101_RX_PRETRIGGER_ENABLE);

  while (1)
  {
    const EventBits_t event_bits = xEventGroupWaitBits(
        hand_global_ch101_event_group,
        HAND_EG_CH101_ALL_ACTIVE_DEV_DATA_READY_BIT,
        pdTRUE,  // clear on exit
        pdTRUE,  // the requested bit must be present
        pdMS_TO_TICKS(HAND_MS_CH101_DEFAULT_MEASURE_PERIOD * 2U));

    if ((event_bits & HAND_EG_CH101_ALL_ACTIVE_DEV_DATA_READY_BIT) == 0U)
    {
      ESP_LOGW(TAG, "CH101 data-ready timeout; stale cycle skipped");
      continue;
    }

    _hand_ch101_handle_data_ready(grp_ptr);
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
