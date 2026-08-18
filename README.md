# FPGA MARG Attitude-Fusion RTL

Synthesizable SystemVerilog implementations of six fixed-point MARG attitude-estimation architectures for FPGA comparison:

- Fixed-β SAAM fusion
- Confidence-gated dynamic-β SAAM fusion
- **RGRSF** (Reliability-Gated Reference-Selective Fusion)
- Mahony MARG observer
- Madgwick MARG filter
- Diagonal-covariance MEKF research prototype

## Main SAAM-family variants

### Fixed β

The baseline combines a SAAM reference quaternion with gyroscope propagation using a user-supplied constant fusion gain. It is the smallest SAAM-family reference implementation in this repository.

### Dynamic β

This version retains the SAAM estimator but changes the correction weight according to acceleration consistency, magnetic-field consistency, magnetic observability, and quaternion innovation. The available gain levels are `0`, `β₀/4`, `β₀/2`, and `β₀`, with hysteretic frame-based gating.

### RGRSF

RGRSF means **Reliability-Gated Reference-Selective Fusion**. It is the selective-reference extension of dynamic fusion. According to the reference-observation condition, it uses MARG correction, an IMU tilt correction, or gyro-only propagation, so an unreliable magnetic reference can be isolated without automatically discarding a usable gravity reference.

## Conventional comparison implementations

The following three implementations are included as independent comparison paths. They are not SAAM/RGRSF variants and are intended to make same-device, fixed-point FPGA comparisons possible.

### Mahony MARG observer

Mahony uses proportional-integral feedback derived from gravity and magnetic-reference errors to correct gyroscope propagation. It is a conventional lightweight nonlinear complementary observer used here as a reference baseline.

### Madgwick MARG filter

Madgwick computes a normalized gradient-descent correction from the inertial and magnetic measurement residuals, then combines it with the gyroscope update. It is included as a widely used gradient-based MARG comparison baseline.

### Diagonal-covariance MEKF prototype

This implementation is a resource-bounded multiplicative error-state Kalman-filter prototype with a diagonal covariance approximation. It is included for exploratory comparison only; it is not a full covariance MEKF implementation.

## Target and scope

- Target synthesis device: **AMD/Xilinx Zynq-7000 XC7Z020-CLG400-2**.
- The supplied XDC is a **generic 50 MHz timing constraint** only.
- Inputs use the shared 11-word streaming frame: `ax ay az mx my mz wx wy wz dt beta/config`.
- Outputs use five words: `qw qx qy qz status`.

This repository intentionally publishes the algorithm RTL, shared arithmetic blocks, and a generic Vivado project-generation script only. It does **not** include board-specific pin assignments, packaged IP, PS/DMA designs, sensor drivers, input datasets, simulation test vectors, generated reports, checkpoints, or bitstreams. Integrators should add their own board wrapper, physical interface, calibration/configuration path, and verification data.

## Directory layout

```text
rtl/common/           shared fixed-point SAAM arithmetic
rtl/fixed_beta/       fixed-beta SAAM top and core
rtl/dynamic_beta/     confidence-gated dynamic-beta top and core
rtl/rgrsf/            RGRSF top and reference-selective core
rtl/mahony_madgwick/  conventional Mahony/Madgwick implementations
rtl/mekf_diag/        diagonal-covariance MEKF research prototype
constraints/          generic 50 MHz timing constraint
scripts/              Vivado project generator
```

## Creating a Vivado project

Run the following from a Vivado command prompt, replacing `rgrsf` with one of `fixed_beta`, `dynamic_beta`, `mahony`, `madgwick`, or `mekf_diag` as needed:

```text
vivado -mode batch -source scripts/create_project.tcl -tclargs rgrsf
```

The project is created under `vivado_<design>/`. The selected top module is reported by the script.

## Notes

- The Mahony and Madgwick paths are conventional comparison implementations and do not use the SAAM reference quaternion or rational normalizer.
- Fixed-point formats, limits, and parameters are defined in the source. Use calibrated sensor inputs and validate parameter values for the intended sampling rate before hardware deployment.

## License

Released under the [MIT License](LICENSE).
