# CH101 scoring/switching validation

## Source reviewed

The implementation was rebuilt against the current `branch_v1` production layout, which has separate CH101 simple, AMP, and IQ queues. The prior encoded reference-only integration package is no longer the authoritative implementation; the production source itself is changed in this branch.

## Host build

The host harness embeds the exact production quality/switching helper block and `_hand_ch101_handle_data_ready()` body. A synchronization checker fails if the harness drifts from production.

```bash
python3 tests/host/verify_ch101_scoring_test_sync.py

gcc -std=c11 \
  -Wall -Wextra -Werror -Wformat=2 \
  -Wshadow -Wconversion -Wsign-conversion \
  tests/host/CH101_scoring_switching_host_test.c \
  -lm -o /tmp/ch101_scoring_switching_test

/tmp/ch101_scoring_switching_test
```

Observed result:

```text
CH101 host harness matches the production scoring/switching block.
CH101 production scoring/switching host tests passed (mode writes=1).
```

The reported final mode-write count belongs to the last isolated test case; each test resets the harness state.

## Exercised behavior

- Correct range selector: Tx/Rx uses `CH_RANGE_ECHO_ONE_WAY`; Rx-only uses SonicLib's documented `CH_RANGE_DIRECT` selector.
- The same stronger candidate must persist for three completed frames before switching.
- The current Tx is retained while it remains inside the relative -6 dB half-power contour.
- `dev1` cannot become Tx even with a dominant synthetic simple amplitude.
- `CH_NO_TARGET` cannot be promoted through stale amplitude or raw IQ activity.
- A non-positive bistatic receiver leg (`d_i <= r_T`) is rejected.
- A near-field/direct inter-sensor path below the 150 mm quality region cannot drive switching.
- A failed promotion restores the previous full mode vector.
- A successful transition leaves exactly one `CH_MODE_TRIGGERED_TX_RX` node.
- A zero-Tx mode vector is repaired.
- A reported port count larger than the four HAND ports is capped before array access.
- The periodic trigger is stopped during blocking waveform I/O and restarted after hardware processing.
- `dev1` leaves the quality path with `range=NAN` and `sample_num=0`.

## Static checks

```text
git diff --check: passed
```

Artifact SHA-256 values used for this validation run:

```text
1c92eaf81c2b66b883d6210096552268713e8ee97deef4083a1b207e5ae36123  src/hand_modules/hand_task/hand_task.c
e74d96da58102290e0574ec51c62094811c40bf8a70cb5a8e6a31ba4c76948b3  include/hand_config.h
e107c3ae94d2cff86e131c74aa4782152e860007a94e5f742219b1934c925d21  tests/host/CH101_scoring_switching_host_test.c
8369dee7cbe57e872f9fbd7504f403531297efacaaba05d8664099172a60d67d  tests/host/verify_ch101_scoring_test_sync.py
```

## Validation boundary

The host harness validates C control logic, state transitions, range semantics, score gating, rollback, and timer ordering. It does not emulate the CH101 ASIC, acoustic mask, FPC structural coupling, I2C transfer duration, or actual fingertip scattering.

Before merging for deployment, hardware tests must record at least:

- post-ring-down noise distributions for dev0/dev2/dev3;
- simple-amplitude and IQ alignment at the DSP-selected target sample;
- per-device gain differences with the installed acoustic masks;
- direct-path leakage in Rx-only traces;
- switch latency and false-switch rate during controlled angular sweeps;
- performance at the 150 mm quality boundary and at the configured maximum range;
- whether a three-frame hold and zero score margin are adequate.

No absolute main-lobe classifier is claimed. The HAND lobe plot itself shows side lobes that can cross the -6 dB contour, so the current implementation can reduce side-lobe influence and choose a stronger reciprocal path, but only an angularly controlled calibration can quantify its main-lobe classification error.
