# Excessive Motion controller: Yaskawa incremental encoder support

> **This is a fork** of [ExcessiveMotion/controller-software](https://github.com/ExcessiveMotion/controller-software),
> the open-source universal machine controller. It adds support for Yaskawa 17-bit
> **incremental** serial encoders, such as the UTSIH-B17CK fitted to SGMGH servo motors, with a
> CNC spindle as the first use. The work is on the
> [`yaskawa-incremental`](https://github.com/devttys1/controller-software/tree/yaskawa-incremental)
> branch; the rest of the code follows upstream.

Full details, status and bench results:
**[Yaskawa incremental encoder support](Documentation/yaskawa-incremental-encoder.md)**.

## What this fork adds

The controller already read Yaskawa **absolute** encoders. Incremental encoders use the same link —
4 Mbit/s Manchester over half-duplex RS-485, HDLC framing with bit stuffing, CRC-16/X.25 — but pack
different data after payload bit 77, so decoding them in absolute mode made the turn count jump by
4096 turns whenever the encoder's coarse position field stepped.

- **FPGA module** ([`yaskawa_encoders.py`](controller-firmware/python/src/yaskawa_encoders.py),
  Amaranth HDL on the Zynq-7020): a per-port `17bit_incremental` mode that exposes the hall
  sensors and the coarse turn counters, a response timeout for silent encoders, a configurable
  request preamble, and fixes to the existing receiver's bit-stuffing and CRC-valid handling.
- **C++ driver** ([`yaskawa_encoders.cpp`](controller-software/core/controller/fpga_module_drivers/src/yaskawa_encoders.cpp)):
  a hall-sensor angle reference at power-up, index detection that gives the true shaft angle
  within one turn, revolution tracking, and speed from the encoder's step timing.
- **Spindle orient (M19)** ([`spindle_orient.h`](controller-software/core/controller/nodes/generic_nodes/inc/spindle_orient.h)):
  a controller node that stops the spindle at a commanded angle.
- **Tests:** an FPGA simulation test bench driven by frames captured from a real encoder
  ([`controller-firmware/python/sim`](controller-firmware/python/sim/README.md)), and host-side
  C++ tests for the driver, index, velocity and spindle-orient logic
  ([`run_host_tests.sh`](controller-software/core/tests/run_host_tests.sh)).
- **Tooling:** containerised and remote Vivado bitstream builds
  ([`vivado-runtime`](controller-firmware/vivado-runtime)), a `spindle_bench` FPGA configuration,
  and a live monitor page for encoder telemetry.

Tested on the bench on a Z-Turn (Zynq-7020) board: decode, index, power cycles, speed and M19.
Still open: where spindle commutation runs, the spindle speed loop, and integration with the drive.

### Related projects

- [yaskawa-odrive](https://github.com/devttys1/yaskawa-odrive): the protocol was first
  reverse-engineered and proven on an STM32; its README is the protocol reference.
- [OpenFFBoard, `f767zi-yaskawa` branch](https://github.com/devttys1/OpenFFBoard/tree/f767zi-yaskawa):
  the same motor and encoder driven by OpenFFBoard.

## Repository layout

Like upstream, the repository has three parts:

| Directory | Contents |
| --- | --- |
| [`controller-firmware`](controller-firmware/README.md) | FPGA HDL generation (Python and Amaranth) and the Vivado project |
| [`controller-software`](controller-software/core/build_and_zynq_setup.md) | The core programs that run on the controller; start with the [software setup docs](controller-software/core/build_and_zynq_setup.md) |
| [`em-os`](em-os/project-spec/meta-user/README) | PetaLinux project for the base Linux OS the controller runs |

See the README in each directory for details.
