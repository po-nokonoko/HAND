#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define HAND_DEV_MAX_NUM_CH101 4U
#define HAND_CH101_QUALITY_DEV_MASK 0x0DU
#define HAND_CH101_QUALITY_RINGDOWN_EXCLUSION_MM 150U
#define HAND_CH101_QUALITY_MIN_VALID_RANGE_MM 150.0f
#define HAND_CH101_QUALITY_MIN_NOISE_SAMPLES 8U
#define HAND_CH101_FOV_HALF_POWER_DB (-6.0f)
#define HAND_CH101_SWITCH_STABILITY_HOLD_CYCLE 3U
#define HAND_CH101_SWITCH_SCORE_MARGIN 0.0f
#define HAND_CH101_REJECT_BELOW_HALF_POWER 0
#define HAND_MS_CH101_QUEUE_MAX_DELAY 100U
#define CH_NO_TARGET UINT32_MAX
#define RET_OK 0U
#define pdTRUE 1
#define pdMS_TO_TICKS(x) (x)
#define CH_IO_MODE_BLOCK 0

typedef enum {
  CH_RANGE_ECHO_ONE_WAY = 0,
  CH_RANGE_ECHO_ROUND_TRIP = 1,
  CH_RANGE_DIRECT = 2,
} ch_range_t;

typedef enum {
  CH_MODE_IDLE = 0x00,
  CH_MODE_FREERUN = 0x02,
  CH_MODE_TRIGGERED_TX_RX = 0x10,
  CH_MODE_TRIGGERED_RX_ONLY = 0x20,
} ch_mode_t;

typedef struct { int16_t q; int16_t i; } ch_iq_sample_t;
typedef struct { ch_iq_sample_t iq_data[150]; } hand_chx01_iq_data_unit_t;
typedef struct { uint16_t amp_data[150]; } hand_chx01_amp_data_unit_t;
typedef struct {
  int64_t timestamps;
  uint16_t sample_num;
  uint16_t amp;
  float range;
} hand_chx01_simple_data_unit_t;
typedef struct { hand_chx01_simple_data_unit_t simple_data[4]; } hand_chx01_simple_data_element_t;
typedef struct { hand_chx01_amp_data_unit_t amp_data[4]; } hand_chx01_amp_data_element_t;
typedef struct { hand_chx01_iq_data_unit_t iq_data[4]; } hand_chx01_iq_data_element_t;

typedef struct ch_dev_t {
  bool connected;
  ch_mode_t mode;
  uint32_t raw_range;
  uint16_t amplitude;
  uint16_t num_samples;
  ch_iq_sample_t iq[150];
  uint8_t iq_error;
  bool fail_promote;
  ch_range_t last_range_type;
} ch_dev_t;
typedef struct ch_group_t { uint8_t num_ports; ch_dev_t dev[4]; } ch_group_t;

static const char* TAG = "TEST";
static uint8_t hand_global_ch101_triggered_dev_num;
static int timer_stops;
static int timer_starts;
static int mode_writes;
static hand_chx01_simple_data_element_t captured_simple;
static hand_chx01_amp_data_element_t captured_amp;
static hand_chx01_iq_data_element_t captured_iq;
static void* hand_global_ch101_simple_data_queue = (void*)1;
static void* hand_global_ch101_amp_data_queue = (void*)2;
static void* hand_global_ch101_iq_data_queue = (void*)3;

static void log_stub(const char* tag, const char* fmt, ...)
    __attribute__((format(printf, 2, 3)));
static void log_stub(const char* tag, const char* fmt, ...)
{
  (void)tag;
  (void)fmt;
}
#define ESP_LOGE(tag, fmt, ...) log_stub((tag), (fmt), ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) log_stub((tag), (fmt), ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) log_stub((tag), (fmt), ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) log_stub((tag), (fmt), ##__VA_ARGS__)

static uint8_t ch_get_num_ports(ch_group_t* g) { return g->num_ports; }
static ch_dev_t* ch_get_dev_ptr(ch_group_t* g, uint8_t d) { assert(d < 4U); return &g->dev[d]; }
static bool ch_sensor_is_connected(ch_dev_t* d) { return d->connected; }
static ch_mode_t ch_get_mode(ch_dev_t* d) { return d->mode; }
static uint8_t ch_set_mode(ch_dev_t* d, ch_mode_t mode) {
  ++mode_writes;
  if (d->fail_promote && mode == CH_MODE_TRIGGERED_TX_RX) return 1U;
  d->mode = mode;
  return RET_OK;
}
static uint32_t ch_get_range(ch_dev_t* d, ch_range_t type) { d->last_range_type=type; return d->raw_range; }
static uint16_t ch_get_amplitude(ch_dev_t* d) { return d->amplitude; }
static uint16_t ch_get_num_samples(ch_dev_t* d) { return d->num_samples; }
static uint8_t ch_get_iq_data(ch_dev_t* d, ch_iq_sample_t* out, uint16_t start,
                              uint16_t count, int mode) {
  (void)mode;
  if (d->iq_error) return d->iq_error;
  memcpy(out, &d->iq[start], count * sizeof(*out));
  return RET_OK;
}
static uint16_t ch_iq_to_amplitude(ch_iq_sample_t* s) {
  const float i=(float)s->i, q=(float)s->q;
  return (uint16_t)lroundf(sqrtf(i*i+q*q));
}
static uint16_t ch_mm_to_samples(ch_dev_t* d, uint16_t mm) { (void)d; return (uint16_t)(mm / 8U); }
static int64_t esp_timer_get_time(void) { static int64_t t; return ++t; }
static uint8_t chbsp_periodic_timer_stop(void) { ++timer_stops; return 0U; }
static uint8_t chbsp_periodic_timer_start(void) { ++timer_starts; return 0U; }
static int xQueueSend(void* q, const void* data, unsigned ticks) {
  (void)ticks;
  if (q == hand_global_ch101_simple_data_queue) memcpy(&captured_simple, data, sizeof(captured_simple));
  if (q == hand_global_ch101_amp_data_queue) memcpy(&captured_amp, data, sizeof(captured_amp));
  if (q == hand_global_ch101_iq_data_queue) memcpy(&captured_iq, data, sizeof(captured_iq));
  return pdTRUE;
}

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

static void reset_state(ch_group_t* g, uint8_t tx_dev)
{
  memset(g, 0, sizeof(*g));
  g->num_ports = 4U;
  for (uint8_t d=0; d<4U; ++d) {
    g->dev[d].connected = true;
    g->dev[d].mode = (d == tx_dev) ? CH_MODE_TRIGGERED_TX_RX : CH_MODE_TRIGGERED_RX_ONLY;
    g->dev[d].raw_range = 6400U;
    g->dev[d].amplitude = 100U;
    g->dev[d].num_samples = 150U;
    for (uint16_t s=0; s<150U; ++s) { g->dev[d].iq[s].i = 10; g->dev[d].iq[s].q = 0; }
  }
  hand_ch101_current_tx_dev = HAND_CH101_INVALID_DEV;
  hand_ch101_pending_tx_dev = HAND_CH101_INVALID_DEV;
  hand_ch101_pending_tx_count = 0U;
  _hand_ch101_reset_range_history();
  timer_stops = timer_starts = mode_writes = 0;
  memset(&captured_simple, 0, sizeof(captured_simple));
}

static void test_switch_after_three_cycles(void)
{
  ch_group_t g;
  reset_state(&g, 3U);
  g.dev[3].raw_range = 6400U; /* 200 mm TX leg */
  g.dev[3].amplitude = 100U;
  g.dev[0].raw_range = 11520U; /* 360 mm total -> 160 mm RX leg */
  g.dev[0].amplitude = 500U;
  g.dev[2].raw_range = 12160U;
  g.dev[2].amplitude = 150U;
  g.dev[1].amplitude = 65000U; /* excluded dev1 must not win */

  _hand_ch101_handle_data_ready(&g);
  assert(g.dev[3].mode == CH_MODE_TRIGGERED_TX_RX);
  _hand_ch101_handle_data_ready(&g);
  assert(g.dev[3].mode == CH_MODE_TRIGGERED_TX_RX);
  _hand_ch101_handle_data_ready(&g);
  assert(g.dev[0].mode == CH_MODE_TRIGGERED_TX_RX);
  assert(g.dev[1].mode == CH_MODE_TRIGGERED_RX_ONLY);
  assert(timer_stops == 3 && timer_starts == 3);
  assert(isnan(captured_simple.simple_data[1].range));
  assert(captured_simple.simple_data[1].sample_num == 0U);
  assert(g.dev[3].last_range_type == CH_RANGE_ECHO_ONE_WAY);
  assert(g.dev[0].last_range_type == CH_RANGE_DIRECT);
}

static void test_half_power_hysteresis_retains_current(void)
{
  ch_group_t g;
  reset_state(&g, 3U);
  g.dev[3].raw_range = 6400U;
  g.dev[3].amplitude = 100U; /* D=4.0e6 */
  g.dev[0].raw_range = 11520U;
  g.dev[0].amplitude = 187U; /* D=5.984e6, current is about -3.5 dB */
  for (int k=0; k<6; ++k) _hand_ch101_handle_data_ready(&g);
  assert(g.dev[3].mode == CH_MODE_TRIGGERED_TX_RX);
}

static void test_no_target_cannot_become_tx(void)
{
  ch_group_t g;
  reset_state(&g, 3U);
  g.dev[0].raw_range = CH_NO_TARGET;
  g.dev[0].amplitude = 65000U;
  for (int k=0; k<4; ++k) _hand_ch101_handle_data_ready(&g);
  assert(g.dev[3].mode == CH_MODE_TRIGGERED_TX_RX);
  assert(isnan(captured_simple.simple_data[0].range));
}

static void test_invalid_bistatic_leg_is_rejected(void)
{
  ch_group_t g;
  reset_state(&g, 3U);
  g.dev[3].raw_range = 6400U; /* 200 mm */
  g.dev[0].raw_range = 6080U; /* 190 mm total/direct path, inconsistent */
  g.dev[0].amplitude = 1000U;
  _hand_ch101_handle_data_ready(&g);
  assert(isnan(captured_simple.simple_data[0].range));
  assert(g.dev[3].mode == CH_MODE_TRIGGERED_TX_RX);
}

static void test_failed_promotion_rolls_back(void)
{
  ch_group_t g;
  reset_state(&g, 3U);
  g.dev[3].raw_range = 6400U;
  g.dev[3].amplitude = 100U;
  g.dev[0].raw_range = 11520U;
  g.dev[0].amplitude = 500U;
  g.dev[0].fail_promote = true;
  for (int k=0; k<3; ++k) _hand_ch101_handle_data_ready(&g);
  assert(g.dev[3].mode == CH_MODE_TRIGGERED_TX_RX);
  assert(g.dev[0].mode == CH_MODE_TRIGGERED_RX_ONLY);
}

static void test_near_field_direct_path_cannot_drive_switch(void)
{
  ch_group_t g;
  reset_state(&g, 3U);
  g.dev[3].raw_range = 6400U; /* valid 200 mm TX target */
  g.dev[3].amplitude = 100U;
  g.dev[0].raw_range = 2192U; /* 68.5 mm direct inter-sensor path */
  g.dev[0].amplitude = 65000U;
  for (int k=0; k<5; ++k) _hand_ch101_handle_data_ready(&g);
  assert(g.dev[3].mode == CH_MODE_TRIGGERED_TX_RX);
}

static void test_mode_invariant_repair_and_port_cap(void)
{
  ch_group_t g;
  reset_state(&g, HAND_CH101_INVALID_DEV);
  g.num_ports = 7U; /* production code must cap the group at four HAND ports */
  for (uint8_t d=0; d<4U; ++d) g.dev[d].mode = CH_MODE_TRIGGERED_RX_ONLY;
  g.dev[2].amplitude = 500U;
  g.dev[2].raw_range = 11520U;
  _hand_ch101_handle_data_ready(&g);
  uint8_t tx_count=0U;
  for (uint8_t d=0; d<4U; ++d) if (g.dev[d].mode == CH_MODE_TRIGGERED_TX_RX) ++tx_count;
  assert(tx_count == 1U);
  assert(g.dev[1].mode == CH_MODE_TRIGGERED_RX_ONLY);
}

int main(void)
{
  test_switch_after_three_cycles();
  test_half_power_hysteresis_retains_current();
  test_no_target_cannot_become_tx();
  test_invalid_bistatic_leg_is_rejected();
  test_failed_promotion_rolls_back();
  test_near_field_direct_path_cannot_drive_switch();
  test_mode_invariant_repair_and_port_cap();
  printf("CH101 production scoring/switching host tests passed (mode writes=%d).\n", mode_writes);
  return 0;
}
