# CH101 scoring/switching host validation

## GitHub harness payload

The exact host harness is stored on `ch101-scoring-switching-v2` as:

```text
tests/host/CH101_scoring_switching_host_test.c.gz.b64
```

Decode it before running the build command:

```bash
base64 -d tests/host/CH101_scoring_switching_host_test.c.gz.b64 | gzip -dc \
  > CH101_scoring_switching_host_test.c
```

## Build command

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

- A path-compensated stronger candidate does not switch immediately.
- The same candidate must remain selected for the configured three completed cycles.
- `dev1` cannot become TX even when its synthetic amplitude dominates, because the processing mask is `0x0D`.
- A sensor with `CH_NO_TARGET` cannot become TX solely because raw IQ samples exist.
- RX-only total paths are interpreted as `d_i = r_T + r_i`; an impossible non-positive receiver leg receives zero directivity confidence.
- A failed mode write restores the original mode vector.
- A successful transition leaves exactly one `CH_MODE_TRIGGERED_TX_RX` sensor.
- The periodic trigger timer is restarted after waveform processing and mode updates.

## Patch verification

The unified patch was applied in a temporary Git repository to an LF-normalized copy of the uploaded `hand_task.c`. The patched result was byte-for-byte identical to `hand_task_ch101_scored.c`, and `git diff --check` reported no whitespace errors.

## Artifact integrity

```text
512cd2079927b18f39a38030f32787e3f084e410c7fbaa4ea49159d662c4e9cb  hand_task_ch101_scored.c
2254ba3ed7a024714bd522973108daea377fb9e8ff1701f6bbc43c6742bbef59  hand_task_ch101_scoring.patch
5f59dddf68af55f074881360383d985da4716b5b4077de3b564b4af6f22e04a2  CH101_scoring_switching_host_test.c
```

These are host-level logic tests. They do not replace on-board validation of I2C transfer duration, firmware-specific target-window alignment, per-device amplitude calibration, or empirical tuning of the hold-cycle and optional score-margin policy parameters.
