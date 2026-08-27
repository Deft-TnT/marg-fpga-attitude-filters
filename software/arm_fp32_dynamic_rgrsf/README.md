# ARM FP32 software baselines: dynamic beta v1 and RGRSF

This package implements the two proposed SAAM-family estimators as an independent C99/FP32 baseline for the Zynq-7000 Cortex-A9 processing system:

- `dynamic_beta_v1_fp32`: SAAM plus four-level confidence-gated beta correction;
- `rgrsf_fp32`: Reliability-Gated Reference-Selective Fusion with MARG, IMU-tilt, and gyro-only reference use.

It is deliberately **not bit-accurate RTL emulation**. The code uses IEEE-754 `float` and standard square-root normalization so that it answers the software-side comparison question. The fixed-point RTL remains the hardware implementation under test.

## Frozen comparison contract

- Input: the same 20,000-frame, 1 kHz nine-axis CSV used by the RTL experiments.
- Coordinate adapter: measured acceleration is negated, magnetometer and gyro are unchanged, and truth is mapped to `[qx_true, -qw_true, qz_true, -qy_true]`.
- Parameters: `beta0=0.0075`, the independently frozen gate thresholds, one bad frame for downgrade, and five clean frames per recovery step.
- Metrics: quaternion geodesic RMSE, maximum error, clean/disturbed partitions, disturbed P95, gate/mode occupancy, and compute-only throughput.

The benchmark reads the CSV once into memory, then repeats estimator computation without file I/O. This measures algorithm computation rather than storage throughput.

## Build

The supplied PowerShell script builds a host functional-check executable and a
statically linked Zynq Cortex-A9 ELF. It expects a C99 compiler on `PATH` and
an `arm-linux-gnueabihf-gcc` cross compiler on `PATH` (or passed explicitly).

```powershell
Set-Location software\arm_fp32_dynamic_rgrsf
.\scripts\build.ps1
```

Use `-HostOnly` or `-ArmOnly` to build a single variant. The ARM output is
`build/arm_marg_baseline_zynq_arm`; it is a statically linked `ELF32`,
little-endian ARM, hard-float binary compiled with
`-mcpu=cortex-a9 -mfpu=neon-vfpv3`. Static linking prevents a routine libc
version mismatch with the board image; it does not alter the FP32 algorithm.

For a non-default toolchain location:

```powershell
.\scripts\build.ps1 -ArmOnly -ArmCompiler 'C:\toolchains\bin\arm-linux-gnueabihf-gcc.exe'
```

## Host smoke run

```powershell
$data = '<path-to-your-calibrated-nine-axis-input.csv>'
.\build\arm_marg_baseline_host.exe --input $data --method both --repeats 200 --output .\reports\host_fp32_20k.json
```

## Board run

```text
scp build/arm_marg_baseline_zynq_arm root@<BOARD-IP>:/root/
scp <chosen-input.csv> root@<BOARD-IP>:/root/
ssh root@<BOARD-IP> 'chmod +x /root/arm_marg_baseline_zynq_arm && /root/arm_marg_baseline_zynq_arm --input /root/<chosen-input.csv> --method both --repeats 200 --output /root/arm_fp32_result.json'
```

`--method` accepts `dynamic`, `rgrsf`, or `both`.

## Evidence boundary

The source can be cross-compiled now, but an actual ARM baseline requires copying the binary and the chosen input CSV to AC7020C Linux, running it there, and retaining the emitted JSON together with the CPU governor, Linux image, compiler version, clock/CPU-load conditions, and run repetitions. Host timing is only a functional smoke check and must not be reported as Cortex-A9 execution time or energy.

## Not included

This directory deliberately excludes input data, expected-output files, test
vectors, generated JSON reports, binaries, bitstreams, board packaging, and
power measurements. Those are platform- and experiment-specific artefacts.
