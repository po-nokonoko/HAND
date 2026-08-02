# CH101 range-quality scoring and dynamic TX/RX switching

## Scope

This implementation modifies the firmware data-ready path only. It does **not** perform fingertip localization, nonlinear least-squares positioning, EKF updates, or Python visualization. Its output remains the existing CH101 simple-range, amplitude-trace, and IQ-trace streams. The purpose of this stage is to improve the physical credibility of those range observations and to keep exactly one CH101 in `CH_MODE_TRIGGERED_TX_RX` while the other connected sensors are in `CH_MODE_TRIGGERED_RX_ONLY`.

The implementation is in:

- `src/hand_modules/hand_task/hand_task.c`
- `include/hand_config.h`

The selected sensor mask is `0x0D`, so `dev0`, `dev2`, and `dev3` participate in scoring and TX selection. `dev1` stays connected to preserve the existing all-connected interrupt synchronization, but it has no range weight, no waveform read, and no TX eligibility in this path.

## Evidence used

The control logic follows these uploaded project sources:

1. The HAND dissertation and the published HAND article report three CH101 measurement periods: transmission, ring-down, and the usable measurement interval. Their normal-mode experiment reports severe range instability below approximately 15 cm.
2. The same HAND FoV experiment reports a 34.3-degree main lobe plus six side lobes. Its lobe plot normalizes the strongest amplitude to 0 dB and uses the -6 dB half-power contour.
3. The uploaded pitch-catch report specifies one synchronized Tx/Rx node and Rx-only nodes, and gives the leading bistatic intensity decay as proportional to `1/(r_T^2 r_i^2)`.
4. The uploaded scoring report connects IQ-derived electrical signal power to acoustic-pressure SNR. It does not provide calibrated values for target reflectivity, receive sensitivity, atmospheric absorption, per-device gain, acoustic-mask gain, or system noise power.
5. The repository SonicLib implementation documents `CH_RANGE_ECHO_ONE_WAY` for the Tx/Rx echo and `CH_RANGE_DIRECT` for an Rx-only pitch-catch node. In this GPR implementation only `CH_RANGE_ECHO_ONE_WAY` is divided by two; `CH_RANGE_DIRECT` preserves the undivided ToF length.

The code therefore uses only quantities measured in the current frame or experimentally supported boundaries. It removes the previous fixed coupling factor, fixed 100-LSB noise floor, fixed 6/20 dB thresholds, and mixed-unit weighted sum because those values were not calibrated for this hardware.

## Measurement acquisition

For each completed synchronized ping:

1. Stop the periodic trigger timer.
2. Discover the actual current transmitter from `ch_get_mode()`.
3. For each selected connected sensor:
   - Tx/Rx or free-running node: read `CH_RANGE_ECHO_ONE_WAY`.
   - Rx-only node: read `CH_RANGE_DIRECT`, which is the documented SonicLib pitch-catch selector.
   - Call `ch_get_amplitude()` only after a valid range. SonicLib does not refresh the simple target amplitude after `CH_NO_TARGET`.
   - Read one blocking IQ trace.
   - Derive the AMP trace from the same IQ trace with `ch_iq_to_amplitude()`.
4. Compute every sensor score after all selected sensors have been acquired.
5. Evaluate and, when required, apply a transactional mode switch.
6. Restart the periodic trigger before software queueing.

Deriving the AMP trace from IQ is exactly what the current GPR Open `ch_get_amplitude_data()` path does internally. Reusing the IQ trace guarantees AMP/IQ sample alignment and avoids a second complete I2C waveform transfer.

## Part 1: IQ-derived signal/noise quality

For each sample:

\[
A_i[n]=\sqrt{I_i[n]^2+Q_i[n]^2},
\qquad
P_i[n]=I_i[n]^2+Q_i[n]^2.
\]

The first 150 mm-equivalent portion is excluded from noise estimation because the HAND normal-mode data show severe near-field/ring-down instability below approximately 15 cm. A range below 150 mm is still preserved in the outgoing raw simple stream, but it receives zero control quality and cannot drive a TX switch.

The robust in-cycle noise-power proxy is the median post-ring-down IQ power:

\[
\widehat N_i=\operatorname{median}\{P_i[n]\}.
\]

For the CH101 DSP-selected target amplitude \(A_{t,i}\):

\[
\widehat S_i=\max(A_{t,i}^2-\widehat N_i,0),
\qquad
\widehat{\mathrm{SNR}}_i=\frac{\widehat S_i}{\widehat N_i}.
\]

The bounded signal-quality factor is:

\[
q_{\mathrm{snr},i}
=\frac{\widehat S_i}{\widehat S_i+\widehat N_i}
=\frac{\widehat{\mathrm{SNR}}_i}{1+\widehat{\mathrm{SNR}}_i}.
\]

This is dimensionless and monotonic in SNR. A one-count squared-power floor is used only to prevent division by zero; it is not presented as a measured noise floor.

## Part 2: relative FoV/directivity quality

For one synchronized pitch-catch ping, the uploaded report gives leading echo intensity proportional to:

\[
I_i\propto\frac{1}{r_T^2r_i^2}.
\]

The corresponding amplitude spreading term is proportional to `1/(r_T r_i)`. The code removes that leading path-length dependence before comparing sensors:

\[
D_T=A_T r_T^2
\]

for the current monostatic Tx/Rx node, and

\[
r_i=d_i-r_T,
\qquad
D_i=A_i r_T r_i
\]

for an Rx-only node whose undivided path is \(d_i\). A candidate with \(d_i\le r_T\) is inconsistent with a positive target-to-receiver leg and is rejected. This also rejects the short direct inter-sensor arrival in the stated far-fingertip operating region.

The normalized simultaneous level is:

\[
D_{\max}=\max_j D_j,
\qquad
L_i=20\log_{10}\left(\frac{D_i}{D_{\max}}\right).
\]

Using the HAND experiment's -6 dB half-power contour:

\[
q_{\mathrm{fov},i}=
\begin{cases}
1, & L_i\ge -6\ \mathrm{dB},\\[4pt]
\dfrac{D_i/D_{\max}}{10^{-6/20}}, & L_i<-6\ \mathrm{dB}.
\end{cases}
\]

Only a candidate at or above the relative -6 dB contour may become the next Tx/Rx node.

### Important limitation

This calculation is a relative directivity confidence, not an online angle estimator. The measured HAND pattern contains side lobes that also cross -6 dB. Therefore neither this code nor the source experiment can mathematically label every single echo as “main lobe” or “side lobe” from amplitude alone. The destructive output-range gate remains disabled by default. The score is used to suppress influence in the control decision and to move the active transmitter toward the strongest reciprocal acoustic path.

The path compensation removes only the leading spherical-spreading term. It intentionally does not invent values for atmospheric absorption, target reflectivity, acoustic-mask gain, or sensor-to-sensor gain. Per-device calibration can be introduced later from measured data.

## Temporal range consistency

A local constant-velocity prediction is formed from the preceding two valid same-mode measurements:

\[
\widehat r_i[k]
=r_i[k-1]+\bigl(r_i[k-1]-r_i[k-2]\bigr).
\]

The scale-free continuity factor is:

\[
q_{\mathrm{range},i}
=\frac{\min(r_i[k],\widehat r_i[k])}
       {\max(r_i[k],\widehat r_i[k])}.
\]

This is an explicit controller similarity metric in `[0,1]`, not a claimed acoustic law. History is reset after a mode/TX change because a monostatic one-way range and an Rx-only undivided path have different meanings.

## Final quality

\[
q_i=q_{\mathrm{snr},i}\,q_{\mathrm{fov},i}\,q_{\mathrm{range},i}.
\]

Every factor is dimensionless. A sensor without all of the following cannot become Tx/Rx:

- selected by mask `0x0D`;
- connected;
- DSP-valid range;
- range at least 150 mm for quality control;
- successful IQ read;
- physically consistent bistatic path when a Tx range is available;
- relative directivity level at or above -6 dB;
- positive complete quality.

## Switching state machine

1. Discover the actual Tx/Rx node from the current modes.
2. Retain the current Tx while it stays within -6 dB of the strongest path-compensated selected sensor.
3. Consider another sensor only when the current Tx is invalid or below the relative half-power contour and the candidate has a larger complete quality.
4. Require the same candidate for three completed cycles by default.
5. Demote every other connected node to `CH_MODE_TRIGGERED_RX_ONLY` before promoting the candidate.
6. Read the mode vector back and verify exactly one `CH_MODE_TRIGGERED_TX_RX` node.
7. Restore the entire previous mode vector if any write or verification fails.
8. Preserve `hand_global_ch101_active_dev_num` as the connected-device bit mask. It is never overwritten with a device index.

A zero- or multiple-transmitter mode vector is repaired immediately. The persistence count and optional score margin are controller parameters that require hardware validation; they are not CH101 physical constants.

## RX pre-trigger and trigger driver

`HAND_CH101_RX_PRETRIGGER_ENABLE` is set to zero for the current far-fingertip priority because SonicLib states that RX pre-trigger reduces the Rx-only maximum range by approximately 200 mm. The task applies this setting before the first measurement.

Application code continues to call `ch_group_trigger()`. The internal `chdrv_group_hw_trigger()` implementation remains in SonicLib/driver code and is not copied into `hand_global.c`.

## Deferred work

This change intentionally does not modify `tools/tcp_server/localization_test.py`. Dynamic TX identity must eventually be carried to the host or inferred from an explicit protocol field before a localization/EKF implementation interprets monostatic and bistatic measurements. That protocol/localization work belongs to the next stage, after hardware validation of the range-quality and switching behavior.
