# D1 - CubeIDE Build

Date: 2026-07-16

## What was done

1. Fixed the `CMD_SYSTEM_STATE` dead-send bug identified in the addendum: added
   `case CMD_SYSTEM_STATE:` to the case group with `CMD_UDP_ETH_ACK`/
   `CMD_ERROR_REPORT` in `uwb_sw/Core/Src/udp_server.c` (`nucleo_udp_send()`), so
   the two existing `main.c` call sites (idle-state "system OK" and
   post-radar-deinit "session done") actually transmit instead of silently
   no-op'ing.
2. Created a real STM32CubeIDE/Eclipse-CDT project for `uwb_sw/`:
   `uwb_sw/.project`, `uwb_sw/.cproject`. Modeled on `boshUWBSTM32`'s project
   (same MCU `STM32H743ZITx`, same board `NUCLEO-H743ZI2`, same toolchain
   `arm-none-eabi-`, same include paths + `USE_PWR_LDO_SUPPLY`/`USE_HAL_DRIVER`/
   `STM32H743xx` defines, same `Core`/`Drivers`/`LWIP`/`Middlewares` source
   entries), with two differences:
   - Source layout kept as `Core/Src`/`Core/Inc` (uwb_sw's native layout, not
     reordered to `boshUWBSTM32`'s `Drivers/Src`), matching the addendum's
     Decision B.
   - `Core/Src/uwb_udp_protocol.c` is excluded from the Debug configuration's
     source entries (`excluding="Src/uwb_udp_protocol.c"` on the `Core`
     sourcePath), because it does not compile against the current
     `nucleo_udp_send()` signature (see `NEW_UWB_PLAN_ADDENDUM.md`, Section B).
   - Only a Debug configuration was created (no Release) — matches what's
     needed to flash/debug for bring-up.
3. Built headlessly (no GUI) via STM32CubeIDE 2.1.1's Eclipse CDT headless
   builder, found at
   `C:\ST\STM32CubeIDE_2.1.1\STM32CubeIDE\stm32cubeidec.exe`, using its bundled
   ARM GCC toolchain (`gnu-tools-for-stm32.14.3.rel1`) — no manual toolchain
   install needed, it ships with CubeIDE.

Command used (note: `-import` must be a `file:/` URI on Windows, or Eclipse
misparses the drive letter as a URI scheme and fails with
`No file system is defined for scheme: C` — hit this on the first attempt,
fixed on the second):

```bash
stm32cubeidec.exe --launcher.suppressErrors -nosplash \
  -application org.eclipse.cdt.managedbuilder.core.headlessbuild \
  -data <scratch_workspace> \
  -import "file:/C:/Users/Usuario/Documents/summer/bosch_UWB/uwb_sw" \
  -cleanBuild "uwb_sw/Debug"
```

## Result

**Build succeeded: 0 errors, 3 warnings.** Full compiler invocation log kept in
`debug_logs/D1_cubeide_build_raw.txt`.

```
   text	   data	    bss	    dec	    hex	filename
 108152	    160	 167347	 275659	  434cb	uwb_sw.elf
23:17:21 Build Finished. 0 errors, 3 warnings. (took 1m:9s.66ms)
```

Artifact produced: `uwb_sw/Debug/uwb_sw.elf` (2,592,196 bytes, Debug build with
symbols), `uwb_sw/Debug/uwb_sw.map`.

The 3 warnings are pre-existing in `uwb_sw` source, not introduced by the two
changes above, and are not blocking:

- `Core/Src/uci/uci_core.c:255` — unused variable `p_ctr`.
- `Core/Src/uci/uci_sr250.c:78` — unused variable `payload`.
- `Core/Src/spi.c:168` — `nucleo_spi_master_transfer_blocking` has a code path
  that reaches the end of a non-`void` function without an explicit `return`
  (real latent bug — the caller may read a garbage return value on that path —
  but out of scope for this pass; flagging for a future firmware cleanup pass).

## Gate

**Passed**: build produces an `.elf`. Debug level `g3`, so the `.elf` is
flash/debug-ready via `STM32_Programmer_CLI` or CubeIDE's own debugger once
hardware is connected.
