# STM32 F9P RTK Rover Logger

Low-cost STM32 + ZED-F9P rover firmware for RTK field point collection, backup UBX logging, Bluetooth NMEA pass-through, and CSV point averaging.

This project is the rover companion to the STM32 F9P base logger. The base provides RTCM corrections; this rover receives corrected GNSS data, forwards NMEA to SW Maps over Bluetooth, logs raw UBX data to SD, and stores averaged points with rod-height correction.

## What It Does

- Logs the incoming ZED-F9P UART stream to SD card as `.UBX`
- Passes NMEA from the F9P through STM32 `USART3` to an HC-05 Bluetooth module
- Lets SW Maps receive live rover position over Bluetooth
- Uses a momentary button to start/stop point collection
- Creates per-point UBX files such as `P001.UBX`
- Creates one CSV point file per boot session such as `POINT001.CSV`
- Averages `NAV-HPPOSLLH` position samples when available
- Falls back to `NAV-PVT` position data if high-precision messages are not enabled
- Applies rod-height correction from `ROD.TXT`
- Displays status on a 128x64 SSD1306 OLED

## Companion Base Firmware

This rover is designed to work with the matching STM32 F9P base logger:

```text
Base F9P -> RTCM over SiK radio -> Rover F9P -> STM32 rover logger
```

Add the base repository link here after publishing:

```text
Companion base project: https://github.com/YOUR_USERNAME/YOUR_BASE_REPO
```

## Hardware

- STM32F407ZGT6 development board
- u-blox ZED-F9P GNSS receiver
- SSD1306 128x64 I2C OLED
- HC-05 Bluetooth serial module
- SD card using STM32 SDIO
- Momentary pushbutton
- RTK correction link, such as SiK radio

## Main Wiring

| Function | STM32 | External device |
|---|---:|---|
| F9P UART TX | `PA10 / USART1 RX` | ZED-F9P UART TX |
| F9P UART RX | `PA9 / USART1 TX` | ZED-F9P UART RX |
| OLED SCL | `PB6 / I2C1 SCL` | OLED SCL/SCK |
| OLED SDA | `PB7 / I2C1 SDA` | OLED SDA |
| HC-05 RX | `PB10 / USART3 TX` | HC-05 RXD |
| HC-05 TX | `PB11 / USART3 RX` | HC-05 TXD |
| Point button | `PE5` | Momentary switch to GND |

The switch is wired between `PE5` and `GND`. The firmware uses the STM32 internal pull-up.

## Rod Height

Create `ROD.TXT` in the SD card root:

```text
2.000
```

The value is meters. On boot, the OLED reminds the user to check the configured rod height.

If `ROD.TXT` is missing or invalid, firmware defaults to `2.000 m`.

## CSV Output

Each boot creates a new point session CSV:

```text
POINT001.CSV
POINT002.CSV
POINT003.CSV
```

Each stored point gets a matching UBX file:

```text
P001.UBX
P002.UBX
P003.UBX
```

CSV columns include:

```text
point,ubx_file,start_utc,end_utc,samples,lat_deg,lon_deg,
antenna_height_m,antenna_hmsl_m,rod_height_m,
point_height_m,point_hmsl_m,fix,carrier,sats,hacc_m,vacc_m
```

`point_height_m` and `point_hmsl_m` are corrected by subtracting rod height from the antenna position.

## Expected OLED Flow

Boot:

```text
BOOT
UARTS OK
SDIO CFG
FATFS OK
MNT=OK SD=OK
...
CHECK ROD HEIGHT
= 2M
```

Logging:

```text
SD WRITE 55KB OK
GNSS012.UBX
LOGGING ** 00:42
SAT=18
3D        UTC=18:24:11
```

Point collection:

```text
START POINT
P001.UBX
P001 12 00:12
```

Press the button again to store:

```text
STORE POINT
```

## RTK Note

The averaging and CSV storage work without RTK fix, but real survey use should require `carrier=2` RTK fixed.

Current CSV fields:

- `fix=3` means 3D GNSS fix
- `carrier=0` means no RTK carrier solution
- `carrier=1` means RTK float
- `carrier=2` means RTK fixed

For survey-grade collection, treat `carrier=2` as the target state.

## Required Tools

- STM32CubeIDE
- STM32CubeMX
- STM32CubeProgrammer
- u-blox u-center
- SW Maps on Android

## Documentation

- [Hardware wiring](docs/HARDWARE.md)
- [F9P rover configuration](docs/F9P_ROVER_CONFIGURATION.md)
- [Build and flash](docs/BUILD_AND_FLASH.md)
- [Relationship to base logger](docs/BASE_AND_ROVER.md)

