# Cómo correr `new_uwb` y generar los logs

Guía operativa para lo que ya está construido (ver `NEW_UWB_PLAN.md`,
`NEW_UWB_PLAN_ADDENDUM.md` y `debug_logs/` para el contexto y lo ya
verificado). Todos los comandos de esta guía fueron probados de verdad
dentro del contenedor `uwb_nxp` — no son solo la teoría del plan.

## Mapa de piezas

| Pieza | Dónde vive | Qué es |
|---|---|---|
| Firmware | `uwb_sw/` | Proyecto STM32CubeIDE. Ya compila (`uwb_sw/Debug/uwb_sw.elf`), falta flashear. |
| Debug sin ROS2 | `tools/uwb_cmd_debug.py` | Script standalone (sin ROS2) para ver tráfico UDP crudo de la placa. |
| Paquete ROS2 | `new_uwb/` | `new_uwb_udp_frame_publisher`, `new_uwb_node`, `new_uwb_test_sender` (sintético, sin hardware). |
| Evidencia ya generada | `debug_logs/` | Un `.md` por fase (D0-D6) con lo que se probó y su resultado. |

## 0. Requisitos previos

- [ ] Docker Desktop corriendo y el contenedor `uwb_nxp` levantado:
  ```powershell
  cd bosch_UWB
  make uwb.up
  ```
- [ ] Para pasos con hardware real: placa NXP conectada por USB (ST-Link) **y**
  por Ethernet, con el PC configurado en la subred de la placa.
- [ ] La IP `192.168.1.102` asignada a la NIC del PC conectada a la placa —
  está hardcodeada en el firmware (`udp_server.c:57`) como destino de todo
  lo que la placa manda. Si el PC tiene otra IP en esa interfaz, no vas a
  recibir nada sin importar qué tan bien esté el resto.

Todo lo que sigue asume que entraste al contenedor con:

```powershell
make uwb.shell
```

y una vez dentro:

```bash
cd /home/ws          # el README raíz dice "/ws"; el mount real es /home/ws
source /opt/ros/humble/setup.bash
source install/setup.bash    # solo si ya compilaste antes; si no, ver paso 2
```

(Ninguna de las dos se sourcea sola al entrar al shell — hay que hacerlo en
cada sesión nueva.)

## 1. Flashear el firmware (necesita hardware)

```powershell
# Confirmar que el ST-Link está conectado:
& "C:\ST\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.400.202601091506\tools\bin\STM32_Programmer_CLI.exe" -l

# Si aparece un ST-Link, flashear:
& "C:\ST\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.400.202601091506\tools\bin\STM32_Programmer_CLI.exe" -c port=SWD -w "uwb_sw\Debug\uwb_sw.elf" -v -rst
```

Si cambiaste algo en `uwb_sw/Core/`, recompilá antes de flashear (headless,
sin abrir la IDE):

```powershell
& "C:\ST\STM32CubeIDE_2.1.1\STM32CubeIDE\stm32cubeidec.exe" --launcher.suppressErrors -nosplash `
  -application org.eclipse.cdt.managedbuilder.core.headlessbuild `
  -data "$env:TEMP\cubeide_ws" `
  -import "file:/C:/Users/Usuario/Documents/summer/bosch_UWB/uwb_sw" `
  -cleanBuild "uwb_sw/Debug"
```

Revisá `uwb_sw/Debug/uwb_sw.elf` y el resumen `0 errors, N warnings` al final
de la salida.

## 2. Compilar `new_uwb` (dentro del contenedor)

```bash
cd /home/ws
source /opt/ros/humble/setup.bash
colcon build --packages-select new_uwb --symlink-install
source install/setup.bash
```

Prueba rápida sin nada de hardware ni red — corre los 25 tests offline del
parser del protocolo:

```bash
cd /home/ws/src/new_uwb
python3 -m unittest test.test_protocol -v
```

Debería terminar en `OK` (25 tests). Si algo de `uwb_sw/Core/Src/udp_server.c`
o del protocolo cambia, este es el primer lugar para volver a correr.

## 3. Probar sin ROS2 primero — `tools/uwb_cmd_debug.py`

Este paso es el más simple para confirmar que la placa manda algo, **antes**
de meter ROS2 en la ecuación. Correlo **dentro del contenedor**, no en el
host de Windows — el contenedor publica el puerto UDP 20000 hacia el host y
si corrés el script también en Windows quedan dos cosas potencialmente
escuchando el mismo puerto sin que sea obvio cuál recibe el tráfico real.

```bash
# dentro de make uwb.shell
cd /home/ws/src
python3 tools/uwb_cmd_debug.py --board-ip 192.168.1.10 --setting-idx 0 \
  --duration-ms 10000 --listen-seconds 30
```

Qué mirar:
- Línea `Sent CMD_START_SESSION ...` — confirma que el envío salió.
- Por cada datagrama que llegue: `cmd=ACK`, `cmd=CIR_REPORT`, etc. Si ves
  `cmd=UNKNOWN_0xXX` seguido, algo no matchea lo documentado en
  `debug_logs/D0_firmware_inventory.md` — es señal de que el protocolo real
  cambió y hay que revisar `new_uwb/new_uwb/protocol.py` contra el firmware
  actual.
- Si a los 30s dice `0 datagramas recibidos`, el script mismo imprime una
  lista de qué revisar (IP del PC, firewall, si el puerto ya está tomado por
  otra cosa).

El log completo queda en `debug_logs/D2_udp_smoke_test.jsonl` (se va
agregando, no se borra entre corridas — bórralo a mano si querés arrancar
limpio).

## 4A. Probar el pipeline ROS2 completo SIN hardware (sintético)

Útil para verificar que nada se rompió en el código, sin depender de la
placa. `new_uwb_test_sender` genera tráfico `CMD_CIR_REPORT` sintético con
el mismo framing que usa el firmware real.

Terminal 1 (dentro del contenedor):
```bash
source /opt/ros/humble/setup.bash && source /home/ws/install/setup.bash
ros2 run new_uwb new_uwb_udp_frame_publisher
```

Terminal 2 (otra sesión `make uwb.shell`):
```bash
source /opt/ros/humble/setup.bash && source /home/ws/install/setup.bash
ros2 run new_uwb new_uwb_test_sender --ros-args -p rate_hz:=10.0
```

Terminal 3 — verificar:
```bash
ros2 topic hz /uwb/frame_raw          # debería estabilizarse cerca de 10 Hz
ros2 topic echo /uwb/frame_raw --once # debería mostrar num_samples/block_size/bytes_per_tap coherentes
```

`Ctrl+C` en las tres terminales para parar. **No dejes corriendo el sender
sintético junto con la placa real** — comentario ya en el propio
`new_uwb.launch.py` y en el docstring del nodo: van a mezclarse los paquetes
sintéticos con los reales en el mismo puerto.

## 4B. Correr con la placa real

Editá primero `new_uwb/launch/new_uwb.launch.py` (las mismas variables que
`sensors/launch/sensors.launch.py` para legacy_tlv):

```python
RECORDING_DURATION_MS = 10_000
BAG_NAME              = ""       # "" -> nombre con timestamp automático
SETTING_IDX           = 0        # preset on-device, no hay TLV config para uwb_sw
BOARD_IP              = "192.168.1.10"
```

Luego:
```bash
source /opt/ros/humble/setup.bash && source /home/ws/install/setup.bash
ros2 launch new_uwb new_uwb.launch.py
```

Esto levanta: `unix_timestamp`, `new_uwb_udp_frame_publisher`,
`uwb_rosbag_recorder_node` (reusado de `sensors`, sin cambios), y
`new_uwb_node` (que manda `CMD_START_SESSION` 2s después de arrancar).

En otra terminal, para ver qué está pasando en vivo:
```bash
ros2 topic echo /uwb/new_uwb_control_status   # ACK/ERROR/SYSTEM_STATE de la placa
ros2 topic hz /uwb/frame_raw                  # frames CIR llegando
```

### ⚠️ Si `ros-humble-rosbag2-storage-mcap` no está instalado

`uwb_rosbag_recorder_node` va a fallar con `Failed to open rosbag` **por
cada frame que llegue** — y como reintenta en cada frame, en pocos segundos
crea decenas de carpetas basura bajo `uwb_rosbags/` (nos pasó probando esto:
más de 250 carpetas en menos de 10 minutos a 15 Hz). Antes de dejarlo
corriendo más de unos segundos:

```bash
apt-get update && apt-get install -y ros-humble-rosbag2-storage-mcap
```

Si el paquete no aparece (nos pasó — no está en el índice apt de esta
imagen, confirmado con `apt-cache search mcap` vacío incluso después de
`apt-get update`), avisale a quien mantiene el `Dockerfile`: hay que
agregarlo ahí para que quede permanente. Mientras tanto, si accidentalmente
lo dejaste corriendo y generó carpetas basura:

```bash
rm -rf /home/ws/src/uwb_rosbags/new_uwb_session_*
```

(el prefijo es `new_uwb_session_` por el `bag_prefix` del launch file —
ajustalo ahí si lo cambiaste).

## 5. Correr `uwb_processing` sobre la grabación

Una vez que exista un bag real en `uwb_rosbags/<nombre>/metadata.yaml`:

```bash
cd /home/ws/src
python -m uwb_processing.run_session --input uwb_rosbags/<nombre>
```

### ⚠️ Si esto falla con un error de scipy/numpy (`_ARRAY_API not found` o similar)

Es un problema de versiones preexistente en este contenedor (numpy 2.x vs.
un scipy de apt compilado contra numpy 1.x), no algo de `new_uwb`. Bloquea
`run_session.py` para legacy_tlv también. Se puede confirmar así:

```bash
python3 -c "import uwb_processing"
```

Si tira ese error, hay que fijar versiones compatibles de numpy/scipy en el
`Dockerfile` (o reinstalar scipy contra la versión actual de numpy) antes de
poder correr `run_session.py` para cualquier protocolo. Mientras tanto, la
compatibilidad de `new_uwb` con `uwb_processing.loaders` ya se verificó por
otra vía (ver `debug_logs/D6_processing_report.md`) sin necesitar arreglar
esto.

## 6. Qué log corresponde a qué, y qué falta generar

| Log | Estado | Cómo generarlo/actualizarlo |
|---|---|---|
| `debug_logs/D0_firmware_inventory.md` | Hecho | No hace falta repetir salvo que cambie el firmware. |
| `debug_logs/D1_cubeide_build.md` | Hecho | Paso 1 de esta guía (rebuild). |
| `debug_logs/D1_flash_smoke_test.md` | **Pendiente — necesita hardware** | Paso 1, con la placa conectada. |
| `debug_logs/D2_udp_smoke_test.md` / `.jsonl` | Script listo, **captura real pendiente** | Paso 3, con la placa conectada. |
| `debug_logs/D3_parser_report.md` | Hecho | `python3 -m unittest test.test_protocol -v` (paso 2). |
| `debug_logs/D4_ros2_publisher.md` | Hecho con datos sintéticos | Paso 4A confirma que sigue funcionando; con placa real, repetir con datos reales y anotar diferencias si las hay. |
| `debug_logs/D5_control_node.md` | Hecho con datos sintéticos | Igual que D4. |
| `debug_logs/D6_rosbag_record.md` / `D6_processing_report.md` | Parcial — bloqueado por mcap y numpy/scipy | Pasos 4B/5, una vez resueltos esos dos gaps de entorno. |

Lo único que realmente falta para cerrar el ciclo completo es **acceso
físico a la placa** (pasos 1 y 3) y, por separado, **arreglar el
`Dockerfile`** (plugin mcap + versiones numpy/scipy) para que el paso 5
corra sin rodeos.

## 7. Troubleshooting rápido

- **No llega nada en absoluto** (ni con `tools/uwb_cmd_debug.py` ni con
  `ros2 topic hz`): revisá en este orden — ¿la placa está flasheada y
  encendida? ¿el link Ethernet está up? ¿la NIC del PC es
  `192.168.1.102`? ¿hay firewall bloqueando UDP entrante? ¿corriste el
  script en el host de Windows en vez de dentro del contenedor?
- **`new_uwb_udp_frame_publisher` no arranca / error de bind**: algo más ya
  tiene el puerto 20000. Buscá procesos viejos:
  ```bash
  ps aux | grep new_uwb
  kill -9 <pid>
  ```
  (nos pasó durante las pruebas — un `ros2 launch` en background que no se
  mató bien dejó publishers viejos corriendo y acumulando tráfico fantasma.)
- **`uwb_rosbags/` se llena de carpetas `new_uwb_session_*` vacías/rotas**:
  ver la advertencia de mcap en el paso 4B.
- **Los tests de `test_protocol.py` fallan después de tocar
  `uwb_sw/Core/Src/udp_server.c`**: significa que el wire format cambió;
  actualizá `new_uwb/new_uwb/protocol.py` para que coincida antes de
  confiar en cualquier otro resultado.
