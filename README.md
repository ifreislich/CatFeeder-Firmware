# CatFeeder-Firmware

This is the firmware to run the CatFeeder project. For completion you will need both
[CatFeeder Electronics](https://github.com/ifreislich/CatFeeder-Hardware)
and
[Catfeeder Enclosure](https://github.com/ifreislich/CatFeeder-3D-Model).

![CatFeeder-Firmware](https://github.com/ifreislich/CatFeeder-Firmware/blob/master/images/root.png)

1. Download dygraph.min.js and dygraph.css from [dygraphs.com](https://dygraphs.com/) and place them in src.
2. Open the project in Platform.io to build.

## Features
- Measured feed dispensing with daily weight quota.
- 10 individually configurable dispensing schedules.
- RFID access control to food and intruder lockout.
- Web interface for configuration.
- Push notifications using [ntfy.sh](https://ntfy.sh) wsupporting either the free or self-hosted service.
  - Visitation.
  - Intruder alerts.
  - Dispensing weighed kibbles.
  - Dispensing issues.
- Rescue menu on USB or BLE UART.
- SNMP agent for monitoring.
- 365 day feeding history with an interactive graph (uses [dygraphs.com](https://dygraphs.com/)).
