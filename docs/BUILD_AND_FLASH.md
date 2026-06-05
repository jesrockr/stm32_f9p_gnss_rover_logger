# Build and Flash

## Prerequisites

Install:

- STM32CubeIDE
- STM32CubeMX
- STM32CubeProgrammer
- u-blox u-center

STM32 tools:

```text
https://www.st.com/en/development-tools/stm32-software-development-tools.html
```

## Import

1. Open STM32CubeIDE.
2. Select a workspace.
3. Go to `File` -> `Import`.
4. Choose `Existing Projects into Workspace`.
5. Select this repository folder.
6. Import the detected project.

## Build

In STM32CubeIDE:

1. Select the project.
2. Click the hammer icon, or use `Project` -> `Build Project`.
3. Confirm the build finishes without errors.

## Flash

Use STM32CubeIDE run/debug, or STM32CubeProgrammer with the generated `.elf`, `.hex`, or `.bin`.

## SD Card Setup

Place `ROD.TXT` in the SD card root:

```text
2.000
```

The value is meters.

The firmware creates:

```text
GNSSxxx.UBX
Pxxx.UBX
POINTxxx.CSV
```

## First Boot Check

Expected behavior:

1. OLED boot messages appear.
2. SD mounts successfully.
3. Rod-height reminder appears.
4. Logging screen starts.
5. SW Maps receives NMEA over HC-05.
6. Button press starts point collection.
7. Second button press stores the point to CSV.

