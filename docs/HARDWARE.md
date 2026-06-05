# Hardware

## Parts

- STM32F407ZGT6 development board
- u-blox ZED-F9P receiver
- SSD1306 128x64 I2C OLED
- HC-05 Bluetooth serial module
- microSD card
- Momentary pushbutton
- RTK correction radio/link

## Wiring

| Function | STM32 | External device |
|---|---:|---|
| F9P UART TX | `PA10 / USART1 RX` | ZED-F9P UART TX |
| F9P UART RX | `PA9 / USART1 TX` | ZED-F9P UART RX |
| OLED SCL | `PB6 / I2C1 SCL` | OLED SCL/SCK |
| OLED SDA | `PB7 / I2C1 SDA` | OLED SDA |
| HC-05 RX | `PB10 / USART3 TX` | HC-05 RXD |
| HC-05 TX | `PB11 / USART3 RX` | HC-05 TXD |
| Point button | `PE5` | Momentary switch to GND |

## Power Notes

- OLED module may require `5V` on `VDD`, depending on the breakout.
- HC-05 breakout `VCC` should usually be powered from `5V` if it includes an onboard regulator.
- UART logic is 3.3V.
- Share ground between STM32, F9P, OLED, HC-05, and correction radio.
- Avoid tying separate 3.3V regulator outputs together unless you know the board power design.

## Button

Use a passive momentary switch.

Wiring:

```text
PE5 ---- switch ---- GND
```

The firmware enables the STM32 internal pull-up.

