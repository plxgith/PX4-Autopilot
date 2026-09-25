

# Build Instructions

This document describes how to build the **Vesta Gimbal Circuit Status Combined** configuration of this PX4-Autopilot fork.

## Repository

Clone the repository with its submodules:

```bash
git clone --recursive https://github.com/plxgith/PX4-Autopilot.git
```

Enter the repository:

```bash
cd PX4-Autopilot
```

## Select the Branch

Checkout the branch containing the combined Vesta Gimbal and Circuit Status changes:

```bash
git checkout Vesta_Gimbal_Circuit_Status_Combined
```

## Initialize and Update Submodules

Synchronize the submodule configuration and update all submodules:

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

## Build

Build the PX4 firmware for the **Cube Orange**:

```bash
make cubepilot_cubeorange_default
```

After a successful build, the firmware files will be available in:

```text
build/cubepilot_cubeorange_default/
```

## Complete Build Procedure

For convenience, the complete procedure can be run as:

```bash
git clone --recursive https://github.com/plxgith/PX4-Autopilot.git
cd PX4-Autopilot
git checkout Vesta_Gimbal_Circuit_Status_Combined
git submodule sync --recursive
git submodule update --init --recursive
make cubepilot_cubeorange_default
```
