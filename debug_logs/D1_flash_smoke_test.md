# D1 - Flash + Smoke Test

Date: 2026-07-16

## Status: BLOCKED — no hardware connected

Checked for a connected probe:

```
STM32_Programmer_CLI.exe -l
```

Result:

```
===== J-Link Interface =====
No J-Link/flasher probe detected.
===== STLink Interface =====
No ST-Link detected!
```

No ST-Link (or J-Link) is attached to this machine right now, and no serial
console for the board was found among the enumerated COM ports (`COM3`/`COM4`
are Bluetooth-over-serial, unrelated). Flashing and the boot/link-status smoke
test cannot be performed from here without the physical board connected.

## What's ready to go

- `uwb_sw/Debug/uwb_sw.elf` — built, 0 errors (see `D1_cubeide_build.md`).
- Flash command, once a probe is connected:
  ```
  STM32_Programmer_CLI.exe -c port=SWD -w "uwb_sw/Debug/uwb_sw.elf" -v -rst
  ```
  (`STM32_Programmer_CLI.exe` is at
  `C:\ST\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.400.202601091506\tools\bin\STM32_Programmer_CLI.exe`.)

## Action needed from the user

1. Connect the NXP/green board's ST-Link (or external probe) via USB.
2. Confirm the PC's NIC connected to the board's Ethernet port is configured as
   `192.168.1.102` — the firmware has this hardcoded as its UDP destination
   (`destination_ip_addr.addr = 0x6601A8C0` in `udp_server.c:57`, see
   `D0_firmware_inventory.md`). If the PC uses a different IP on that link,
   the board will send CIR/ACK/ERROR/SYSTEM_STATE datagrams into the void no
   matter how correct `new_uwb`'s decoding is.
3. Re-run `STM32_Programmer_CLI.exe -l` to confirm the probe is visible, then
   flash with the command above.
4. After flashing, either open a serial/RTT console if available, or proceed
   straight to D2 (`tools/uwb_cmd_debug.py`) and watch for any UDP traffic at
   all on port 20000 as the first sign of life.

## Gate

**Not evaluated — blocked on hardware.** Do not treat this as a failure; the
build-side gate (D1 build) already passed. This gate needs the user to attach
the board.
