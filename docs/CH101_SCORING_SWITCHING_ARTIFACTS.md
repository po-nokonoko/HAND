# CH101 scoring/switching artifact index

The `branch_v1` source tree uses an older combined CH101 data schema, while the uploaded integration target has separate simple, AMP, and IQ structures. To avoid a false merge, the exact newer-source artifacts are stored as gzip-compressed Base64 payloads on `ch101-scoring-switching-v2`.

## Payloads

- Complete modified source: `reference/hand_task_ch101_scored.c.gz.b64`
- Unified patch against the uploaded newer source: `patches/hand_task_ch101_scoring.patch.gz.b64`
- Standalone host validation harness: `tests/host/CH101_scoring_switching_host_test.c.gz.b64`

## Decode

```bash
base64 -d reference/hand_task_ch101_scored.c.gz.b64 | gzip -dc \
  > hand_task_ch101_scored.c
base64 -d patches/hand_task_ch101_scoring.patch.gz.b64 | gzip -dc \
  > hand_task_ch101_scoring.patch
base64 -d tests/host/CH101_scoring_switching_host_test.c.gz.b64 | gzip -dc \
  > CH101_scoring_switching_host_test.c
```

## Integrity

```text
512cd2079927b18f39a38030f32787e3f084e410c7fbaa4ea49159d662c4e9cb  hand_task_ch101_scored.c
2254ba3ed7a024714bd522973108daea377fb9e8ff1701f6bbc43c6742bbef59  hand_task_ch101_scoring.patch
5f59dddf68af55f074881360383d985da4716b5b4077de3b564b4af6f22e04a2  CH101_scoring_switching_host_test.c
```

See `CH101_SCORING_SWITCHING_IMPLEMENTATION.md` for the equations, physical assumptions, limits of the FoV score, and switching state machine. See `CH101_SCORING_SWITCHING_VALIDATION.md` for the host build and behavioral test result.
