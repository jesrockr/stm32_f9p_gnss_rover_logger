# ZED-F9P Rover Configuration

## UART to STM32

The STM32 expects the F9P UART connected to `USART1` to run at:

```text
460800 baud
```

Required output on this UART:

- UBX raw/logging stream
- `UBX-NAV-PVT`
- `UBX-NAV-HPPOSLLH`
- Lean NMEA for SW Maps pass-through

Recommended NMEA:

- `GGA`
- `RMC`

Disable high-volume NMEA such as `GSV` unless you know the Bluetooth link can handle it.

## HC-05 / SW Maps

The STM32 forwards NMEA sentences to HC-05 over:

```text
USART3, PB10/PB11, 115200 baud
```

SW Maps should connect to the HC-05 Bluetooth serial device and receive NMEA.

## RTK Status

The CSV `carrier` field comes from `NAV-PVT`.

```text
carrier=0  no RTK carrier solution
carrier=1  RTK float
carrier=2  RTK fixed
```

For real survey point collection, aim for `carrier=2`.

## Correction Input

The rover F9P should receive RTCM corrections from the base over a separate correction link, such as SiK radio.

Typical base RTCM messages:

- `1005` or `1006`
- `1074`
- `1094`
- `1084` plus `1230` if GLONASS is used

Keep correction bandwidth lean.

