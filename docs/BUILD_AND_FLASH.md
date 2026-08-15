# Build and Flash

## Prerequisites

Install:

- STM32CubeProgrammer
- u-blox u-center

STM32 tools:

```text
https://www.st.com/en/development-tools/stm32-software-development-tools.html
```



*IMPORTANT: the `BOOT0` jumper pad on the STM32 board must be bridged in order for .ELF firmware build to be flashed via USB-C in STM32CubeProgrammer. It must be opened to run program on STM32. Recommend use of ST-LINK V2 to flash firmware.


 Flash With STM32CubeProgrammer:

- Open STM32CubeProgrammer.
- Connect STM32 board using USB-C or ST-LINK V2.
- Select USB or ST LINK and hit ⟳ then hit Connect.
- NOTE: USB only available when BOOT0 jumper pad is bridged.
- Find/select the .ELF filepath from the Debug/ folder.
- Click Start Programming to flash the STM32.
- Unplug USB



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

