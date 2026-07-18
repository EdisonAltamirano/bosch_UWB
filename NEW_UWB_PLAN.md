# Plan: firmware + recepción ROS2 para la placa NXP (`uwb_sw` → `new_uwb`)

Objetivo general: replicar para `uwb_sw` (placa NXP) el mismo flujo que hoy funciona
para `legacy_tlv` (`boshUWBSTM32` + paquete ROS2 `sensors`), terminando en rosbags de
`/uwb/frame_raw` consumibles sin cambios por `uwb_processing`.

Dos objetivos, en orden (el 2 depende de que el 1 esté compilando y flasheado):

1. **Firmware**: convertir `uwb_sw/` en un proyecto STM32CubeIDE que compile y flashee,
   igual que `boshUWBSTM32/`.
2. **Software de recepción `new_uwb`**: nuevo paquete ROS2 (nodos, decoding, get/set,
   grabación de rosbags) equivalente a `legacy_tlv` pero hablando el protocolo real de
   `uwb_sw`.

---

## Estado actual (verificado en el repo, 2026-07-17)

### `boshUWBSTM32` (referencia — legacy_tlv, funciona)
- Proyecto STM32CubeIDE completo: `.project`, `.cproject`, `.mxproject`, `.settings/`,
  `boschUWBSTM32.ioc`, `boschUWBSTM32 Debug.launch`, carpeta `Debug/` con artefactos de
  build.
- Fuente de aplicación en `Drivers/Src/` (incl. `Drivers/Src/uci/*`), no en `Core/Src/`.
- Protocolo UDP propio (`Drivers/Inc/uwb_udp_protocol.h`): magic `0x55 0x57`, versión
  `0x01`, opcodes `MSG_SET_CONFIG_FULL(0x01) / SET_PARAMS_PARTIAL(0x02) /
  START_RADAR(0x03) / STOP_RADAR(0x04) / ACK(0x05) / ERROR(0x06) / RADAR_FRAME(0x07) /
  RADAR_FRAME_CHUNK(0x08)`, config vía TLVs (`UWB_TAG_*`), reensamblado de frames
  grandes vía chunking (`UWB_MAX_CHUNK_DATA`, `UWB_RADAR_RAW_MAX_LEN`).
- Lado PC: paquete ROS2 `sensors` con `protocol_mode=legacy_tlv` (`uwb_node.py`,
  `uwb_udp_frame_publisher.py`, `sr250_protocol.py`) — probado y en uso (ver bags en
  `uwb_rosbags/`, README raíz).

### `uwb_sw` (placa NXP — punto de partida, incompleto)
- **Sin proyecto CubeIDE**: no hay `.project`, `.cproject`, `.mxproject`, `.ioc` ni
  `.settings/`. Solo hay código fuente suelto (`Core/`, `Drivers/`, `LWIP/`,
  `Middlewares/`, dos `.ld`) más un binario ya compilado
  `Binary/sample_eth_code_3.elf` y `sample_eth_code_3.launch` (config de debug de
  otro entorno, probablemente MCUXpresso/otro IDE, no CubeIDE).
- La fuente de aplicación vive en `Core/Src/` (no `Drivers/Src/` como en
  `boshUWBSTM32`), con los mismos nombres de archivo (`main.c`, `udp_server.c`,
  `uwb_udp_protocol.c`, `uci/uci_core.c`, `uci/uci_commands.c`, `uci/uci_radar.c`,
  `uci/uci_sr250.c`, `uci/uci_transport.c`). `main.c` y `uwb_udp_protocol.c` difieren
  bastante en contenido de `boshUWBSTM32` (no es una copia idéntica).
- **Hallazgo importante**: `uwb_sw/Core/Inc/uwb_udp_protocol.h` define **el mismo**
  magic/versión/opcodes/tags que `legacy_tlv` (`MSG_SET_CONFIG_FULL...RADAR_FRAME`,
  mismos `UWB_TAG_*`), pero **le faltan** `RADAR_FRAME_CHUNK`, `UWB_ACK_REPLY_PORT`,
  `UWB_MAX_CHUNK_DATA` y `UWB_RADAR_RAW_MAX_LEN`. Es decir: el envelope es
  compatible/heredado de legacy_tlv, pero sin soporte de fragmentación de frames
  grandes. `uwb_udp_protocol_send_radar_frame()` manda el payload crudo en un solo
  datagrama.
- El control interno del radar (`uci_commands.c`) sí implementa el set completo de
  comandos UCI de NXP SR250 (`CORE_GET/SET_CONFIG`, `SESSION_GET/SET_APP_CONFIG`,
  `GET_DEVICE_INFO`, `GET_CAPS_INFO`, `GET_STATE`, `GET_COUNT`,
  `GET_RANGING_COUNT`, reset, etc. — ver
  `uwb_docs/NXP_SR250_UCI_Specification_v2.0.23.pdf`), más rico que lo que expone
  hacia el PC por UDP.

### Paquete ROS2 `sensors` — soporte `uwb_sw` ya existente, pero probablemente obsoleto
- `uwb_udp_frame_publisher.py`, `uwb_node.py` y `sr250_protocol.py` ya tienen un
  parámetro `protocol_mode ∈ {legacy_tlv, uwb_sw}` con una rama `uwb_sw` parcial.
- **Esa rama `uwb_sw` usa un wire format distinto** al que de verdad implementa
  `uwb_sw/Core/Src/uwb_udp_protocol.c`: comandos de 1 byte
  `CMD_START_SESSION(0x01)/CMD_CIR_REPORT(0x02)/CMD_UDP_ETH_ACK(0x03)/
  CMD_ERROR_REPORT(0x04)` con fragmentación por `last_fragment`, en vez del envelope
  `MAGIC + version + msg_type + seq + len` con TLVs que el firmware realmente manda.
  Todo indica que este código se escribió contra una revisión anterior/distinta del
  firmware (posiblemente la del `sample_eth_code_3.elf` precompilado) y **nunca se
  validó contra el `uwb_sw/Core/Src` actual**.
- Control (`uwb_node.py`): en modo `uwb_sw` solo existe `send_start_session()`
  (setting_idx + duración). No hay equivalente a `send_full_config` /
  `send_partial_update` de legacy_tlv — no manda TLVs de configuración.
- Conclusión: **no asumir que la rama `uwb_sw` actual sirve de base**; hay que
  decidir en Fase 0 si se reescribe contra el envelope real del firmware (que
  aparenta ser ~compatible con `sr250_protocol.py` de legacy_tlv) o si el firmware
  se ajusta al esquema `CMD_*` ya codificado en Python. Ver "Decisiones abiertas".

### Piezas reutilizables tal cual (no protocolo-específicas)
- `sensors_interfaces/msg/UwbFrame.msg` — mensaje genérico, ya usado por legacy_tlv;
  sirve para `new_uwb` sin cambios.
- `sensors/sensors/uwb_rosbag_recorder_node.py` — graba por nombre de tópico, no
  conoce el protocolo. Reutilizable apuntando a `/uwb/frame_raw` sin duplicar.
- `sensors/sensors/uwb_frame_parser_node.py`, `uwb_cir_inspector.py` — idem, actúan
  sobre `UwbFrame` ya decodificado.
- `uwb_processing/uwb_processing/loaders.py` — consume rosbags de `/uwb/frame_raw`
  reutilizando el parser CIR de `sr250_protocol.py`; si `new_uwb` publica el mismo
  `UwbFrame` con `radar_data_type=0x00` (CIR) en el mismo formato de payload, esto
  funciona sin tocarlo.
- Workspace Docker (`Makefile`, `compose*.yaml`, contenedor `uwb_nxp`) — ya nombrado
  para NXP; el build de firmware es aparte (STM32CubeIDE en el host, fuera del
  contenedor), igual que hoy con `boshUWBSTM32`.

### Recursos de referencia disponibles
- `uwb_docs/NXP_SR250_UCI_Specification_v2.0.23.pdf` — spec UCI del chip.
- `uwb_docs/Ethernet_Output_Structure_Dump.txt` — captura real de tráfico Ethernet
  (útil para verificar el wire format real contra lo que dice el código).
- `uwb_docs/Bosch_UWB_Measurement_Setup.pdf`, `UWB_Radar_Fundamentals.pdf`.
- `boshUWBSTM32/Drivers/Src/sr250_radar_code_guide.tex` (y su copia en `uwb_sw`) —
  guía interna del código de radar.

---

## Objetivo 1 — Firmware `new_uwb` compilando sobre `uwb_sw`

Meta: un proyecto STM32CubeIDE, análogo a `boshUWBSTM32`, que compile limpio y
flashee la placa NXP, arrancando desde el código de `uwb_sw`.

1. **Confirmar identidad de hardware**: MCU (STM32H743ZITx, según los `.ld`),
   módulo/placa NXP UWB conectado (¿shield SR250 sobre Nucleo-H743ZI2?), y método de
   programación/depuración (ST-Link igual que `boshUWBSTM32`, u otro).
2. **Reconciliar layout de fuente**: `uwb_sw` tiene la app en `Core/Src` /
   `Core/Inc` mientras `boshUWBSTM32` la tiene en `Drivers/Src` / `Drivers/Inc`.
   Decidir si se generan `.project`/`.cproject`/`.ioc` nuevos apuntando al layout de
   `uwb_sw` tal cual, o si se reordena `uwb_sw` para calcar la estructura de
   `boshUWBSTM32` (recomendado si se quiere diffear ambos firmwares fácilmente más
   adelante).
3. **Crear el proyecto CubeIDE** (`.ioc`, `.project`, `.cproject`, `.mxproject`,
   `.settings/`) usando `boschUWBSTM32.ioc` como plantilla de arranque (mismo MCU,
   clocks, periféricos SPI/ETH/TIM base) y ajustando lo que sea específico de la
   placa NXP (pines del módulo UWB, si difieren).
4. **Verificar linker/memoria**: comparar `uwb_sw/STM32H743ZITX_FLASH.ld` /
   `_RAM.ld` contra los de `boshUWBSTM32` — deberían coincidir si es el mismo MCU;
   si no, entender por qué.
5. **Build limpio**: compilar en CubeIDE, resolver includes/paths rotos que
   aparezcan por el layout distinto, dejar un build `Debug` (y `Release` si aplica)
   sin warnings nuevos respecto a `boshUWBSTM32`.
6. **Flash + smoke test**: flashear la placa NXP, confirmar que:
   - Levanta el link Ethernet/LWIP (igual que `boshUWBSTM32`).
   - Responde a un `START_RADAR`/`CMD_START_SESSION` (el que corresponda tras la
     Fase 0 del Objetivo 2) con al menos un frame CIR o un ACK visible por UDP
     (`nc -u` / Wireshark) antes de escribir nada de ROS2.
7. Actualizar/crear el `.tex` de guía si el pinout o el flujo de arranque cambia
   respecto al documentado para legacy_tlv.

**Definición de "listo" para el Objetivo 1**: build reproducible sin errores, `.elf`
flasheable, y al menos un paquete UDP real observado saliendo de la placa hacia el
PC con el protocolo que se decida en la Fase 0 del Objetivo 2.

---

## Objetivo 2 — Paquete ROS2 `new_uwb` (recepción, get/set, decoding, rosbag)

Meta: nodos ROS2 nuevos, nombrados `new_uwb`, que hablen el protocolo real de
`uwb_sw`, publiquen `UwbFrame` en `/uwb/frame_raw` y permitan grabar rosbags
consumibles por `uwb_processing` sin tocarlo.

### Fase 0 — Reconciliar el protocolo (bloqueante, hacer primero)
- Capturar tráfico real UDP de la placa `uwb_sw` ya flasheada (Objetivo 1) con
  Wireshark/tcpdump y compararlo contra:
  - El envelope de `uwb_sw/Core/Src/uwb_udp_protocol.c` (magic `0x55 0x57` + TLVs,
    compatible con `sr250_protocol.py` de legacy_tlv).
  - La rama `CMD_*` ya escrita en `sensors/sensors/sr250_protocol.py` /
    `uwb_udp_frame_publisher.py` / `uwb_node.py`.
  - `uwb_docs/Ethernet_Output_Structure_Dump.txt` como tercer punto de referencia.
- Decidir con base en la captura real cuál protocolo está vivo. Si es el envelope
  tipo legacy_tlv (lo más probable dado el `.h` actual), **la rama `uwb_sw` actual
  en `sensors/` debe tratarse como código muerto/incorrecto** — no extenderla, y
  documentar por qué se descarta.
- Confirmar específicamente el tamaño real de un frame CIR con el preset activo
  (128 muestras, hasta 3 antenas RX según `RADAR_PRESETS` en
  `sensors/launch/sensors.launch.py`) contra el límite de datagrama único que
  `uwb_sw` puede mandar hoy (sin `RADAR_FRAME_CHUNK`). Si el frame no cabe en un
  datagrama UDP seguro (~1400 bytes), el firmware de `uwb_sw` necesita el mismo
  mecanismo de chunking que `boshUWBSTM32` antes de que `new_uwb` pueda recibir
  frames completos — esto puede rebotar al Objetivo 1.

### Fase 1 — Esqueleto del paquete
- Nuevo paquete ROS2 `new_uwb/` (hermano de `sensors/`), calcando
  `sensors/setup.py` y `sensors/package.xml`: `ament_python`, depende de `rclpy` y
  `sensors_interfaces` (reutilizar `UwbFrame.msg`, no duplicarlo), `resource/`,
  `launch/`, módulo Python `new_uwb/`.
- Entry points espejo de `sensors`: `new_uwb_udp_frame_publisher`, `new_uwb_node`
  (control/get-set), y reutilizar tal cual `uwb_rosbag_recorder_node`,
  `uwb_frame_parser_node`, `uwb_cir_inspector` desde `sensors` (o exponerlos también
  como entry points de `new_uwb` si se prefiere que el paquete sea autocontenido) en
  vez de reescribirlos.

### Fase 2 — Decoding (`new_uwb/sr250_protocol_uwb_sw.py` o similar)
- Si la Fase 0 confirma el envelope tipo legacy_tlv: portar/adaptar directamente
  `unpack_envelope`, `unpack_tlvs`, `parse_radar_frame_payload`,
  `parse_cir_udp_payload` de `sensors/sensors/sr250_protocol.py` (son genéricos,
  ya parametrizados por `radar_data_type`), quitando/ajustando solo lo que no
  exista en `uwb_sw` (p.ej. chunking, si no se implementó en el Objetivo 1).
- Si la Fase 0 confirma el esquema `CMD_*`: corregir/terminar los parsers ya
  presentes en `sr250_protocol.py` (`parse_uwb_sw_ack_packet`,
  `parse_uwb_sw_error_packet`, `parse_uwb_sw_cir_fragment_packet`) contra el
  firmware real, no contra suposiciones.
- En cualquier caso, el resultado final debe llenar el mismo `UwbFrame.msg` que usa
  legacy_tlv (mismos campos: `seq`, `msg_type`, `radar_data_type`,
  `session_handle`, `status`, `num_samples`, `block_size`, `bytes_per_tap`,
  `raw_payload`) para que `uwb_processing` no necesite saber qué protocolo generó
  el bag.

### Fase 3 — `new_uwb_udp_frame_publisher` (equivalente a `uwb_udp_frame_publisher.py`)
- Nodo dedicado (no una rama `if protocol_mode==...` dentro del nodo de legacy_tlv,
  tal como pidió el usuario): escucha UDP, decodifica con el módulo de la Fase 2,
  publica `UwbFrame` en `/uwb/frame_raw` (mismo tópico, mismo tipo de mensaje →
  intercambiable con legacy_tlv a nivel de bag).
- Mantener el mismo patrón de parámetros (`listen_ip`, `listen_port`,
  `topic_name`) para que el launch file sea análogo.

### Fase 4 — `new_uwb_node` (control, get/set — equivalente a `uwb_node.py`)
- Mandar `START_RADAR`/`STOP_RADAR` (o `CMD_START_SESSION`, según Fase 0).
- Si el envelope soporta TLVs (`SET_CONFIG_FULL` / `SET_PARAMS_PARTIAL`), exponer
  `send_full_config` / `send_partial_config` igual que legacy_tlv, reutilizando
  `build_radar_config_tlvs` / `build_partial_radar_config_tlvs` si el set de tags
  es el mismo (`UWB_TAG_*` coincide entre ambos `.h` hoy).
  - Nota: si se prefiere aprovechar el UCI completo del SR250 (`GET_CONFIG`,
    `GET_CAPS_INFO`, `GET_STATE`, etc. — más rico que legacy_tlv), habría que
    exponer opcodes UDP adicionales en el firmware primero; por defecto, igualar
    el subset que ya expone `uwb_udp_protocol.h` de `uwb_sw`.
- Recibe ACK/ERROR (puerto de ack, igual patrón que `PC_LISTEN_ACK_PORT` en
  legacy_tlv) y loguea igual que `uwb_node.py`.

### Fase 5 — Launch + grabación de rosbag
- `new_uwb/launch/new_uwb.launch.py`, calcando la estructura de
  `sensors/launch/sensors.launch.py` (presets, `STM32_IP`/puertos, nodos
  encadenados con `TimerAction`), pero apuntando a `new_uwb_udp_frame_publisher` /
  `new_uwb_node`.
- Reutilizar `uwb_rosbag_recorder_node` sin cambios, apuntado a
  `/uwb/frame_raw`, escribiendo a `uwb_rosbags/<bag_name>/` — mismo layout que hoy
  usa `uwb_processing`.
- Alternativa más simple si se prefiere no duplicar todo el launch file: mantener
  `sensors.launch.py` como único launch y agregar `new_uwb_udp_frame_publisher` /
  `new_uwb_node` como nodos seleccionables vía el mismo `PROTOCOL_MODE`, en vez de
  la rama `if protocol_mode==` actual dentro de los nodos de `sensors`. Definir con
  el usuario cuál de las dos organizaciones prefiere (ver "Decisiones abiertas").

### Fase 6 — Validación end-to-end
- Grabar una sesión corta con `new_uwb` contra la placa NXP real.
- Correr `uwb_processing.run_session` (`python -m uwb_processing.run_session
  --input uwb_rosbags/<bag_name>`) **sin modificarlo** y confirmar que produce
  `range_time.png`, `summary.json`, etc., igual que con una sesión legacy_tlv.
- Si algo en `loaders.py` necesita un branch de protocolo, es una señal de que la
  Fase 2 no dejó el `UwbFrame` en un formato realmente equivalente — corregir ahí,
  no en `uwb_processing`.

---

## Decisiones abiertas (para el usuario, antes de implementar)

1. **¿Paquete ROS2 separado `new_uwb` o extender `sensors` con más nodos
   `protocol_mode`?** El código parcial que ya existe usa la segunda estrategia
   (rama dentro de los mismos archivos). El usuario pidió explícitamente algo
   "que se llame new_uwb", lo que sugiere paquete/nodo dedicado. Recomendado:
   paquete `new_uwb` separado, y evaluar deprecar la rama `uwb_sw` actual dentro de
   `sensors/` una vez que `new_uwb` funcione, para no mantener dos implementaciones
   del mismo protocolo divergentes.
2. **Confirmar qué protocolo UDP habla realmente `uwb_sw` hoy** (Fase 0 del
   Objetivo 2) — es el mayor riesgo del plan; todo lo demás en el Objetivo 2 se
   deriva de esa respuesta.
3. **Chunking de frames grandes**: si el CIR con el preset activo no cabe en un
   datagrama, decidir si se porta `RADAR_FRAME_CHUNK` al firmware de `uwb_sw`
   (Objetivo 1) antes de continuar con el Objetivo 2.
4. **Alcance del control (`get/set`)**: replicar solo el subset que legacy_tlv ya
   expone (`SET_CONFIG_FULL`/`SET_PARAMS_PARTIAL`/`START`/`STOP`), o aprovechar que
   `uwb_sw` internamente soporta el UCI completo del SR250 (`GET_CONFIG`,
   `GET_CAPS_INFO`, `GET_STATE`, etc.) y exponer más comandos por UDP de los que
   legacy_tlv tiene hoy.
5. **Layout de fuente del firmware**: ¿reordenar `uwb_sw` para que coincida
   exactamente con `boshUWBSTM32` (`Drivers/Src/...`), o dejarlo en `Core/Src/...`
   y adaptar el proyecto CubeIDE a ese layout?
