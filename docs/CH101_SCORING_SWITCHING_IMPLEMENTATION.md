# CH101 signal-quality scoring and dynamic TX/RX switching

## Scope

This implementation replaces the arbitrary mixed-unit score in the uploaded, newer `hand_task.c` and keeps the scoring/switching path inside `_hand_ch101_handle_data_ready()`.

The checked-in `branch_v1` file layout is older than the uploaded source: `branch_v1` stores CH101 simple and IQ data in one group structure, whereas the uploaded source has separate simple/AMP/IQ queues and protobuf paths. For that reason, this branch carries an apply-ready patch payload and a complete reviewed source payload rather than overwriting the incompatible `branch_v1` source path.

## Sensor set

The processing mask is `0x0D`, so only `dev0`, `dev2`, and `dev3` contribute to quality scoring or TX selection. `dev1` remains connected for group interrupt synchronization, but its simple output and waveform sample count are set to zero in this processing path.

## Correct range semantics

- `CH_MODE_TRIGGERED_TX_RX` and `CH_MODE_FREERUN` use `CH_RANGE_ECHO_ONE_WAY`.
- `CH_MODE_TRIGGERED_RX_ONLY` uses `CH_RANGE_ECHO_ROUND_TRIP` for the bistatic Tx-target-Rx path requested by this project.
- `CH_RANGE_DIRECT` is not used for target echo ranging.
- `ch_get_amplitude()` is called only after a valid range because SonicLib leaves that value stale after `CH_NO_TARGET`.

## Part 1 — signal/noise quality

For each valid IQ sample:

\[
A_i[n]=\sqrt{I_i[n]^2+Q_i[n]^2},\qquad P_i[n]=I_i[n]^2+Q_i[n]^2.
\]

The early waveform up to the configurable 150 mm exclusion point is omitted from the raw peak/noise estimator because the HAND normal-mode measurements report strong near-field/ring-down instability below approximately 15 cm. The median post-ring-down power is used as an in-cycle robust noise-power proxy:

\[
\widehat N_i=\operatorname{median}\{P_i[n]\}.
\]

For a DSP-selected target amplitude `A_target`, the noise-subtracted signal power and SNR estimate are

\[
\widehat S_i=\max(A_{target,i}^2-\widehat N_i,0),\qquad
\widehat{\mathrm{SNR}}_i=\frac{\widehat S_i}{\widehat N_i}.
\]

The bounded signal-quality score is

\[
q_{\mathrm{snr},i}=\frac{\widehat S_i}{\widehat S_i+\widehat N_i}
=\frac{\widehat{\mathrm{SNR}}_i}{1+\widehat{\mathrm{SNR}}_i}.
\]

This avoids a fabricated fixed ADC noise floor and avoids mixing amplitude, energy, and SNR with arbitrary coefficients.

## Part 2 — relative FoV/directivity quality

For simultaneous valid echoes from the selected three sensors:

\[
A_{\max}=\max_j A_j,\qquad
L_i=20\log_{10}\left(\frac{A_i}{A_{\max}}\right).
\]

The HAND FoV experiment normalized the strongest amplitude to 0 dB and used -6 dB as the half-power boundary. The implementation therefore uses

\[
q_{\mathrm{fov},i}=
\min\left(1,
\frac{A_i/A_{\max}}{10^{-6/20}}
\right).
\]

This is a confidence term, not an online angle estimator. The measured pattern contains side lobes that may themselves cross -6 dB, so the code does not claim that this score proves main-lobe reception. The optional destructive `HAND_CH101_REJECT_BELOW_HALF_POWER` gate is disabled by default; the score is used for weighting and switching.

## Temporal range continuity

A local constant-velocity prediction produces `r_hat`. The unit-free range-continuity score is

\[
q_{\mathrm{range},i}=
\frac{\min(r_i,\widehat r_i)}{\max(r_i,\widehat r_i)}.
\]

It remains bounded in `[0,1]` without introducing an unsupported variance scale or exponential decay constant.

## Final confidence

\[
q_i=q_{\mathrm{snr},i}\,q_{\mathrm{fov},i}\,q_{\mathrm{range},i}.
\]

All factors are dimensionless. A sensor without a valid DSP range cannot become the next transmitter, even when its raw IQ trace exists. This explicitly prevents the `dev1` failure mode from being promoted into a weight or TX decision.

## TX switching policy

1. Discover the actual TX from `ch_get_mode()`; do not assume a fixed device index.
2. Find the highest-confidence valid candidate in mask `0x0D`.
3. Retain the current TX while it remains within -6 dB of the strongest simultaneous valid echo.
4. Consider switching when the current TX loses a valid target or falls below the measured -6 dB boundary, the candidate has a larger complete confidence score, and the same candidate persists for the configured hold cycles.
5. Demote all other connected sensors to `CH_MODE_TRIGGERED_RX_ONLY` before promoting the candidate to `CH_MODE_TRIGGERED_TX_RX`.
6. Verify exactly one TX; restore all previous modes if any write or verification fails.
7. Never write the TX index into `hand_global_ch101_active_dev_num`; that variable is the connected-device bit mask.

The default hold count of three cycles and optional score margin are explicit control-policy parameters, not claimed sensor constants.

## I/O and timing

SonicLib requires waveform I/O to complete before a new measurement. The implementation stops the periodic timer during blocking IQ reads and mode writes, then restarts it after completion. AMP samples are derived from the same IQ trace by default, which preserves sample alignment and avoids a second full I2C transfer. `HAND_CH101_READ_AMP_TRACE_FROM_SENSOR=1` restores a separate firmware amplitude read when the measurement interval is sufficiently long.

The data-ready task also checks its event-group timeout and skips incomplete cycles instead of reading stale sensor data.

## Required initialization outside this patch

Dynamic switching requires one triggered TX/RX node and the remaining connected nodes in triggered RX-only mode before the first measurement, for example:

```c
#define HAND_CH101_DEFAULT_MODE \
  {CH_MODE_TRIGGERED_RX_ONLY, CH_MODE_TRIGGERED_RX_ONLY, \
   CH_MODE_TRIGGERED_RX_ONLY, CH_MODE_TRIGGERED_TX_RX}
```

`ch_group_trigger()` already dispatches through the SonicLib/BSP hardware-trigger implementation. The low-level `chdrv_group_hw_trigger()` routine must not be copied into `hand_global.c` or called directly by the application.

Receive pre-triggering is independent of TX selection. It improves very-short-distance RX-only capture but reduces the configured RX-only maximum range by about 200 mm. For the stated far-fingertip priority, keep it disabled unless short-distance tests demonstrate a need.

## Artifacts and decoding

The two text payloads are gzip-compressed and Base64-encoded to preserve the exact uploaded-file patch/source while keeping the older branch schema untouched:

- `patches/hand_task_ch101_scoring.patch.gz.b64`
- `reference/hand_task_ch101_scored.c.gz.b64`

Decode them with:

```bash
base64 -d patches/hand_task_ch101_scoring.patch.gz.b64 | gzip -dc \
  > hand_task_ch101_scoring.patch
base64 -d reference/hand_task_ch101_scored.c.gz.b64 | gzip -dc \
  > hand_task_ch101_scored.c
```

Apply the patch only to the newer source revision whose CH101 path has separate simple/AMP/IQ structures. Do not apply it blindly to `branch_v1`; port the helper/state logic after that branch is synchronized with the newer schema.
