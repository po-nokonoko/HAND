#pragma once

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

#include "hand_hw_config.h"
#include "hand_wifi_config.h"

/* CH101 mode related */
// clang-format off
#define HAND_CH101_DEFAULT_MODE {CH_MODE_TRIGGERED_RX_ONLY, CH_MODE_TRIGGERED_RX_ONLY, CH_MODE_TRIGGERED_RX_ONLY, CH_MODE_TRIGGERED_TX_RX}

// Suggest FREERUN by Dennis Liu
//#define HAND_CH101_DEFAULT_MODE {CH_MODE_FREERUN, CH_MODE_FREERUN, CH_MODE_FREERUN, CH_MODE_FREERUN}

// Related to "shape of housing"
#define HAND_CH101_FW_INIT {ch101_gpr_open_init, ch101_gpr_open_init, ch101_gpr_open_init, ch101_gpr_open_init}
// #define HAND_CH101_FW_INIT {ch101_gpr_sr_open_init, ch101_gpr_sr_open_init, ch101_gpr_sr_open_init, ch101_gpr_sr_open_init}

// clang-format on

/* CPU ID related config (driver) */

/**
 * @brief If the CPU ID is specified as 0, an "INSUFFICIENT INTR ALLOCATION"
 * error may occur.
 *
 */
#define HAND_CPU_ID_SPI2 (ESP_INTR_CPU_AFFINITY_1)
#define HAND_CPU_ID_SPI3 (ESP_INTR_CPU_AFFINITY_1)

/* CPU ID related config (tasks) */

/* Size */

// SPI bus

#define HAND_SIZE_SPI2_TRANSFER      (4096)
#define HAND_SIZE_SPI2_BMI323_QUEUE  (10)
#define HAND_SIZE_SPI2_BOS1901_QUEUE (10)
#define HAND_SIZE_SPI2_KX132_QUEUE   (5)

#define HAND_SIZE_SPI3_TRANSFER    (4096)
#define HAND_SIZE_SPI3_KX132_QUEUE (5)

// Ping pong buffer (PPB)
// frame rate
#define HAND_SIZE_PPB_VL53L1X (35)
#define HAND_SIZE_PPB_CH101_SIMPLE (35)
#define HAND_SIZE_PPB_CH101_AMP (5)
#define HAND_SIZE_PPB_CH101_IQ (5)

// NanoPB stream buffer size
// size of messages
#define HAND_SIZE_NANOPB_BUFFER_VL53L1X (2048)
#define HAND_SIZE_NANOPB_BUFFER_CH101_SIMPLE (4192)
#define HAND_SIZE_NANOPB_BUFFER_CH101_AMP (6000)
#define HAND_SIZE_NANOPB_BUFFER_CH101_IQ  (12000)

// Queue size (size of the queue used for buffering real-time data before
// storing it in the ping-pong buffer.)

#define HAND_SIZE_QUEUE_VL53L1X (10)
#define HAND_SIZE_QUEUE_CH101_SIMPLE (10)
#define HAND_SIZE_QUEUE_CH101_AMP (10)
#define HAND_SIZE_QUEUE_CH101_IQ (10)

/* Time related (ms) [delay, polling...] */

// VL53L1X related
#define HAND_MS_VL53L1X_QUEUE_MAX_DELAY              (50)
#define HAND_MS_VL53L1X_NEW_DATA_READY_POLL_DURATION (100)
#define HAND_MS_VL53L1X_SEND_DATA                    (500)
// This must be uint16_t [20, 1000]
#define HAND_MS_VL53L1X_DEFAULT_TIMING_BUDGET (50)
// This argument should be equal or larger than (VL53L1X_TIMING_BUDGET_MS +
// 4)
#define HAND_MS_VL53L1X_DEFAULT_MEASURE_PERIOD (100)

// CH101 related
#define HAND_MS_CH101_QUEUE_MAX_DELAY (100)
/* XXX: I don't know why but this value will be doubled at somewhere? */
// Trying to figure...
#define HAND_MS_CH101_DEFAULT_MEASURE_PERIOD (100)
#define HAND_MS_CH101_SEND_SIMPLE_DATA       (100)
#define HAND_MS_CH101_SEND_AMP_DATA          (100)
#define HAND_MS_CH101_SEND_IQ_DATA           (100)

/* CH101 range-quality scoring and dynamic TX/RX switching.
 * dev1 is deliberately excluded because its DSP range/amplitude outputs are
 * not valid for this project, while dev0/dev2/dev3 remain eligible. */
#define HAND_CH101_QUALITY_DEV_MASK (0x0DU)

/* The HAND normal-mode measurements show severe near-field/ring-down
 * instability below approximately 15 cm.  This interval is excluded only from
 * the waveform noise estimator; it is not an unconditional range cutoff. */
#define HAND_CH101_QUALITY_RINGDOWN_EXCLUSION_MM (150U)
#define HAND_CH101_QUALITY_MIN_VALID_RANGE_MM     (150.0f)
#define HAND_CH101_QUALITY_MIN_NOISE_SAMPLES      (8U)

/* The measured HAND FoV lobe plot uses the normalized -6 dB half-power contour.
 * It is a relative directivity reference, not an online angle measurement. */
#define HAND_CH101_FOV_HALF_POWER_DB (-6.0f)

/* Switching control policy.  Hold count and score margin are controller
 * parameters to be confirmed on hardware, not CH101 physical constants. */
#define HAND_CH101_SWITCH_STABILITY_HOLD_CYCLE (3U)
#define HAND_CH101_SWITCH_SCORE_MARGIN          (0.0f)

/* RX pre-trigger improves very-short-range RX-only capture but shortens the
 * RX-only maximum range by about 200 mm.  Keep disabled for far-fingertip work. */
#define HAND_CH101_RX_PRETRIGGER_ENABLE (0U)

/* A -6 dB level cannot prove that an echo belongs to the main lobe because the
 * measured pattern contains side lobes above the same contour.  Therefore the
 * destructive gate is disabled by default; quality is used for weighting and
 * TX switching instead. */
#define HAND_CH101_REJECT_BELOW_HALF_POWER (0U)

// RGB LED related
#define HAND_MS_RGB_LED_BLINK_DELAY (1000)
#define HAND_MS_ALIVE_BLINK_DELAY   HAND_MS_RGB_LED_BLINK_DELAY
