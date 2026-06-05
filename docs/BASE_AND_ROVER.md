# Base and Rover Relationship

This rover firmware is intended to be used with a separate STM32 F9P base logger project.

Recommended GitHub layout:

```text
stm32-f9p-base-logger
stm32-f9p-rover-logger
```

Keep them as separate repositories because they are different firmware builds with different field behavior:

- Base firmware handles survey-in, cold-start-on-boot, RTCM/base behavior, and raw base logging.
- Rover firmware handles Bluetooth NMEA pass-through, point collection, CSV storage, and rod-height correction.

Link the two projects from each README.

Suggested base README link text:

```markdown
Companion rover project: https://github.com/YOUR_USERNAME/stm32-f9p-rover-logger
```

Suggested rover README link text:

```markdown
Companion base project: https://github.com/YOUR_USERNAME/stm32-f9p-base-logger
```

The base and rover can be developed independently, but they should use compatible F9P UART rates and RTCM message settings.

