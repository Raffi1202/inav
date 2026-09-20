# INAV GitHub Issue: u-blox ZED-X20P auto-configuration

Repository: https://github.com/iNavFlight/inav/issues/new/choose

## Title

ZED-X20P remains GPS=UNAVAILABLE with auto-config enabled, works when auto-config and auto-baud are disabled

## Issue body

### Current behavior

A u-blox ZED-X20P connected to INAV 9.1.0 is reported as unavailable when the default automatic configuration is enabled:

```text
Sensor status: ... GPS=UNAVAILABLE
GPS: HW Version: Unknown Proto: 0.00 Baud: 115200 (UBLOX Proto >= 15.0 required)
  SATS: 0
```

The receiver is electrically connected correctly and already outputs valid UBX-NAV-PVT data at 115200 baud. After disabling both automatic configuration and automatic baud detection and rebooting the FC, INAV immediately reports:

```text
Sensor status: ... GPS=OK
GPS: HW Version: Unknown Proto: 0.00 Baud: 115200 (UBLOX Proto >= 15.0 required)
```

The test was performed indoors without GNSS reception, so zero satellites/no fix is expected and unrelated. The important difference is `GPS=UNAVAILABLE` versus `GPS=OK`.

### Hardware and software

- FC target: `IFLIGHT_BLITZ_ATF435`
- INAV: `9.1.0 Sep 9 2026 / 19:29:13 (1c89adbc)`
- INAV Configurator: 9.1.1
- Receiver: u-blox ZED-X20P
- Receiver firmware: EXT HPG 2.11 (c97ee1)
- Connection: UBX over a hardware serial port at 115200 baud

### Relevant INAV configuration

```text
serial 1 2 115200 115200 0 115200

gps_provider = UBLOX
gps_auto_config = ON
gps_auto_baud = ON
gps_auto_baud_max_supported = 230400
gps_ublox_nav_hz = 8
gps_ublox_use_galileo = ON
gps_ublox_use_beidou = ON
gps_ublox_use_glonass = OFF
```

### Relevant ZED-X20P UART configuration

Read back over the receiver's USB interface:

```text
UART baud: 115200
UART input UBX: enabled
UART output UBX: enabled
UART output NMEA: disabled
UBX-NAV-PVT output rate: 1
UBX-NAV-SIG output rate: 1
```

Both receiver UARTs had the same baud rate and PVT output settings during the test.

### Steps to reproduce

1. Connect a ZED-X20P that outputs UBX-NAV-PVT at 115200 baud.
2. Configure the corresponding INAV serial port for GPS at 115200 baud.
3. Use `gps_provider = UBLOX`, `gps_auto_config = ON`, and `gps_auto_baud = ON`.
4. Save and reboot.
5. Run `status`: GPS is reported as unavailable, hardware unknown, protocol 0.00.
6. Set:

   ```text
   set gps_auto_config = OFF
   set gps_auto_baud = OFF
   save
   ```

7. Run `status` after reboot: GPS is now reported as OK and the UBX data is processed.

### Expected behavior

INAV should detect and configure the ZED-X20P, or at least continue using its valid existing UBX-NAV-PVT output without requiring both automatic settings to be disabled manually.

### Possible cause

This is an inference from the current source: `gpsDecodeHardwareVersion()` appears to recognize hardware versions only through u-blox M10 and returns `UBX_HW_VERSION_UNKNOWN` for other values. Protocol-version parsing is then guarded by a hardware-version check. The X20P therefore remains `Unknown / Proto 0.00`, which may cause the automatic configuration sequence to use the wrong path or fail to complete.

I can provide additional UBX configuration values or test a development build if needed.

## Confirmed workaround

These settings were saved successfully on the FC and changed the status from `GPS=UNAVAILABLE` to `GPS=OK`:

```text
set gps_auto_config = OFF
set gps_auto_baud = OFF
save
```

The `HW Version: Unknown / Proto: 0.00` display remains because INAV 9.1.0 does not identify the X20P hardware version, but UBX position data is accepted.
