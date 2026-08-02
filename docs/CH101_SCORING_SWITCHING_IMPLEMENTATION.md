# CH101 signal-quality scoring and dynamic TX/RX switching

## Scope

This implementation replaces the arbitrary mixed-unit score in the uploaded newer `hand_task.c` and performs all signal assessment and TX/RX switching from `_hand_ch101_handle_data_ready()` after every selected sensor has been read.

The checked-in GitHub `branch_v1` still uses an older combined CH101 data structure, while the uploaded source has separate simple, AMP, and IQ queues and protobuf paths. The GitHub branch therefore carries an exact patch/source payload instead of overwriting the incompatible older source file.

## Selected sensors

The processing mask is `0x0D`:

- `dev0`: included
- `dev1`: excluded from range weighting, confidence, and TX selection
- `dev2`: included
- `dev3`: included

`dev1` remains electrically connected so the existing group interrupt synchronization remains intact, but this processing path writes zero range, zero amplitude, and zero waveform count for it.

## Correct SonicLib range semantics

- `CH_MODE_TRIGGERED_TX_RX` and `CH_MODE_FREERUN` use `CH_RANGE_ECHO_ONE_WAY` for monostatic target distance.
- `CH_MODE_TRIGGERED_RX_ONLY` uses `CH_RANGE_ECHO_ROUND_TRIP` for the total bistatic path requested by this project:

\[
d_i=r_T+r_i,
\]

where \(r_T\) is transmitter-to-fingertip distance and \(r_i\) is fingertip-to-receiver distance.
- `CH_RANGE_DIRECT` is not used for target-echo ranging because it denotes the direct transmitter-to-receiver path.
- `ch_get_amplitude()` is called only when `ch_get_range()` reports a target; SonicLib does not refresh the simple target amplitude after `CH_NO_TARGET`.

## Part 1 — waveform signal/noise quality

For every raw IQ sample:

\[
A_i[n]=\sqrt{I_i[n]^2+Q_i[n]^2},\qquad
P_i[n]=I_i[n]^2+Q_i[n]^2.
\]

The HAND measurements show severe normal-mode near-field/ring-down instability below approximately 15 cm. The code therefore excludes the first 150 mm-equivalent samples from the waveform peak/noise estimator. This exclusion is used only for waveform quality estimation; it does not automatically delete every reported range below 150 mm.

The post-ring-down median power is used as a robust, in-cycle noise-power proxy:

\[
\widehat N_i=\operatorname{median}\{P_i[n]\}.
\]

For the DSP-selected target amplitude \(A_{\text{target},i}\), the noise-subtracted signal power and SNR estimate are

\[
\widehat S_i=\max\!\left(A_{\text{target},i}^{2}-\widehat N_i,0\right),
\qquad
\widehat{\mathrm{SNR}}_i=\frac{\widehat S_i}{\widehat N_i}.
\]

The bounded signal-quality score is the monotonic SNR transform

\[
q_{\mathrm{snr},i}
=\frac{\widehat S_i}{\widehat S_i+\widehat N_i}
=\frac{\widehat{\mathrm{SNR}}_i}
       {1+\widehat{\mathrm{SNR}}_i}.
\]

This avoids a fabricated fixed ADC noise floor and avoids adding amplitude, energy, and SNR values with incompatible units.

## Part 2 — path-compensated FoV/directivity quality

For one pitch-catch ping, the leading spherical-spreading term of received echo pressure is proportional to

\[
A_i\propto
\frac{|H_T(\theta_T)H_i(\theta_i)|}{r_T r_i},
\]

apart from target reflectivity, atmospheric absorption, and device calibration terms. For the current TX/RX device, \(r_i=r_T\), so the corresponding leading term is proportional to \(1/r_T^2\).

The code removes this leading distance dependence before comparing sensors:

\[
D_T=A_T r_T^2,
\qquad
D_i=A_i r_T r_i,
\qquad
r_i=d_i-r_T.
\]

Thus, for one synchronized ping, the ratio \(D_i/D_T\) approximates the receiving directivity ratio \(H_i/H_T\) under acoustic reciprocity. This is more appropriate for a far fingertip than comparing uncorrected amplitudes from unequal monostatic and bistatic paths.

The simultaneous normalized directivity level is

\[
D_{\max}=\max_j D_j,
\qquad
L_i=20\log_{10}\!\left(\frac{D_i}{D_{\max}}\right).
\]

The HAND FoV experiment normalized its strongest amplitude to 0 dB and used \(-6\,\mathrm{dB}\) as the half-power boundary. The code therefore defines

\[
q_{\mathrm{fov},i}=
\min\!\left(
1,
\frac{D_i/D_{\max}}{10^{-6/20}}
\right).
\]

When the current TX range is unavailable, the code can only use raw amplitude as a temporary reacquisition proxy; logs mark this case with `[RAW]`. When a valid TX range exists, an RX-only total path satisfying \(d_i\le r_T\) is physically inconsistent because it would imply a non-positive receiver leg, so that candidate receives zero directivity confidence.

This FoV term is not an online angle estimator. The measured HAND pattern contains side lobes that may themselves exceed the \(-6\,\mathrm{dB}\) contour; therefore the code does not claim that the threshold mathematically proves main-lobe reception. The destructive `HAND_CH101_REJECT_BELOW_HALF_POWER` gate remains disabled by default. The term is used for confidence weighting and TX-switch hysteresis.

## Temporal range-continuity factor

A local constant-velocity predictor produces \(\widehat r_i\) from the two preceding valid samples. The code then uses the dimensionless similarity

\[
q_{\mathrm{range},i}=
\frac{\min(r_i,\widehat r_i)}
     {\max(r_i,\widehat r_i)}.
\]

This is an explicit control metric, not a claimed acoustic law. It is bounded in \([0,1]\) and avoids introducing an unsupported variance scale or exponential tuning constant. History is reset whenever the sensor mode or active transmitter changes because monostatic and bistatic ranges have different meanings.

## Final confidence

\[
q_i=q_{\mathrm{snr},i}\,
    q_{\mathrm{fov},i}\,
    q_{\mathrm{range},i}.
\]

Every factor is dimensionless. A sensor without a valid DSP range cannot become the next transmitter even when raw IQ samples are present. This explicitly prevents the known `dev1` failure mode from contaminating the weight or switching decision.

## TX-switching state machine

1. Read the actual TX from `ch_get_mode()`; do not assume a fixed device index.
2. Calculate all three selected-sensor confidences after completing the current measurement reads.
3. Retain the current TX while its path-compensated level remains within \(-6\,\mathrm{dB}\) of the strongest simultaneous valid sensor.
4. Consider another sensor only when the current TX loses a valid target or falls below the measured half-power boundary and the candidate has a larger complete confidence score.
5. Require the same candidate for the configured hold count; the default is three completed cycles.
6. Demote every other connected sensor to `CH_MODE_TRIGGERED_RX_ONLY`, then promote the candidate to `CH_MODE_TRIGGERED_TX_RX`.
7. Read all modes back and verify exactly one TX/RX node.
8. Restore the entire previous mode vector if any mode write or verification fails.
9. Preserve `hand_global_ch101_active_dev_num` as the connected-device bit mask. Only `hand_global_ch101_triggered_dev_num` is recomputed after mode changes.

The hold count and optional confidence margin are control-policy parameters. They are deliberately not presented as CH101 physical constants and must be checked on the actual array.

## I/O and timing safeguards

SonicLib requires full waveform I/O to finish before another measurement is triggered. The implementation therefore stops the periodic trigger timer during blocking IQ reads and mode writes, then restarts it after processing.

By default, AMP samples are generated from the same IQ trace:

\[
A[n]=\sqrt{I[n]^2+Q[n]^2}.
\]

This guarantees sample alignment and avoids a second complete I2C waveform transfer. Setting `HAND_CH101_READ_AMP_TRACE_FROM_SENSOR=1` restores a separate firmware amplitude read, but only a measurement period long enough for both transfers is safe.

The data-ready task now checks the event-group wait result and skips a timed-out/incomplete cycle instead of reading stale data.

## Required initialization outside this patch

Dynamic switching requires one initial TX/RX node and the remaining connected nodes in triggered receive-only mode, for example:

```c
#define HAND_CH101_DEFAULT_MODE \
  {CH_MODE_TRIGGERED_RX_ONLY, CH_MODE_TRIGGERED_RX_ONLY, \
   CH_MODE_TRIGGERED_RX_ONLY, CH_MODE_TRIGGERED_TX_RX}
```

The application must continue calling `ch_group_trigger()`. SonicLib and the BSP already route that call to the hardware-trigger implementation. The internal `chdrv_group_hw_trigger()` routine must not be copied into `hand_global.c` or called directly by application code.

Receive pre-triggering is independent of TX selection. It improves very-short-distance receive-only capture but reduces RX-only maximum range by approximately 200 mm. For the stated far-fingertip priority, keep it disabled unless controlled short-range testing demonstrates a requirement.

## Artifacts

Local files:

- `hand_task_ch101_scored.c` — complete modified source
- `hand_task_ch101_scoring.patch` — unified patch against the uploaded newer source
- `CH101_scoring_switching_host_test.c` — standalone host harness
- `CH101_SCORING_SWITCHING_VALIDATION.md` — build/test result

The GitHub feature branch stores the exact source and patch as gzip-compressed Base64 payloads because `branch_v1` has the older incompatible data schema:

- `patches/hand_task_ch101_scoring.patch.gz.b64`
- `reference/hand_task_ch101_scored.c.gz.b64`

Decode with:

```bash
base64 -d patches/hand_task_ch101_scoring.patch.gz.b64 | gzip -dc \
  > hand_task_ch101_scoring.patch
base64 -d reference/hand_task_ch101_scored.c.gz.b64 | gzip -dc \
  > hand_task_ch101_scored.c
```
