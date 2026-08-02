# CH101 scoring/switching host validation

## Build command

The standalone harness contains SonicLib, FreeRTOS queue, timer, and CH101 mode/range stubs around the production scoring/switching block.

```bash
gcc -std=c11 -Wall -Wextra -Werror \
  CH101_scoring_switching_host_test.c -lm \
  -o ch101_scoring_unit_test
./ch101_scoring_unit_test
```

## Result

```text
CH101 scoring/switching host tests passed (mode writes=5).
```

## Exercised behavior

- A stronger valid candidate does not switch immediately.
- The same candidate must remain selected for the configured three completed cycles.
- `dev1` cannot become TX even when its synthetic amplitude dominates, because the processing mask is `0x0D`.
- A sensor with `CH_NO_TARGET` cannot become TX solely because raw IQ samples exist.
- A failed mode write restores the original mode vector.
- A successful transition leaves exactly one `CH_MODE_TRIGGERED_TX_RX` sensor.
- The periodic trigger timer is restarted after waveform processing and mode updates.

## Artifact integrity

After decoding the payload files documented in `CH101_SCORING_SWITCHING_IMPLEMENTATION.md`, the expected SHA-256 values are:

```text
dfda41e37f8f5628d4be3c1a499134140bc3290c7a423675ab06f874536b8a36  hand_task_ch101_scored.c
0793882d5ecfbf44dd703c56767e1ec8151a35dd5dff157b18181dce1c11d5a2  hand_task_ch101_scoring.patch
```

These are host-level logic tests. They do not replace on-board validation of I2C transfer duration, firmware-specific target-window alignment, or empirical tuning of the hold-cycle and optional score-margin policy parameters.
