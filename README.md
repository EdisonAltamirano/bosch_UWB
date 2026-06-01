# Bosch UWB

## Container and ROS workspace setup

```bash
cd bosch_UWB
make uwb.up
make uwb.shell
```

Stop the container without deleting it:

```bash
make uwb.down
```

On the first container boot, build the ROS 2 workspace once:

```bash
cd /ws
colcon build
source install/setup.bash
```

This repository is mounted into the container at `/home/ws/src`, so edits on the host appear immediately in the container and vice versa.

## Acquisition pipeline

Run the full acquisition stack with:

```bash
ros2 launch sensors sensors.launch.py
```

The launch file `sensors/launch/sensors.launch.py` is the main place to change per-session acquisition settings:

- `PROTOCOL_MODE`: choose the on-wire protocol, currently `"legacy_tlv"` or `"uwb_sw"`. 
- `RECORDING_DURATION_MS`: how long the board streams and how long the rosbag recorder runs. (Change this)
- `BAG_NAME`: output bag directory name under `uwb_rosbags/`. (Change this)
- `ACTIVE_PRESET`: which preset from `RADAR_PRESETS` to send to the board.
- `RADAR_PRESETS[...]`: radar configuration fields mirrored into `uwb_node`.
- `STM32_IP`, `STM32_PORT`, `PC_LISTEN_ACK_PORT`, `PC_LISTEN_FRAME_PORT`: transport addresses and ports.

What the launch file starts:

1. `unix_timestamp`: publishes timestamps for alignment.
2. `uwb_udp_frame_publisher`: listens for UDP packets and publishes `/uwb/frame_raw`.
3. `uwb_cir_inspector`: decodes received CIR frames and writes `/tmp/session_summary.txt`.
4. `uwb_frame_parser_node`: decoding/parsing hook for `.npz` export; the save call in `sensors/sensors/uwb_frame_parser_node.py` is currently commented out.
5. `uwb_rosbag_recorder_node`: records `/uwb/frame_raw` to `uwb_rosbags/<bag_name>/`.
6. `uwb_node`: sends the radar configuration and starts the session on the STM32.

The default recorded topic is `/uwb/frame_raw`. The rosbag under `uwb_rosbags/<bag_name>/` is the main offline input for signal processing.

## Offline signal processing

The main offline entry point is `uwb_processing/uwb_processing/run_session.py`.

Quick-run flow:

1. Change `BAG_NAME` near the top of `run_session.py` if you want the script default to point at a different session.
2. Run one session:

```bash
cd /home/ws/src
python -m uwb_processing.run_session
```

Equivalent console entry point:

```bash
cd /home/ws/src
uwb_run_session
```

More explicit form:

```bash
cd /home/ws/src
python -m uwb_processing.run_session \
  --input uwb_rosbags/test1 \
  --annotation ground_truth/annotations/test1.yaml
```

Outputs are written to `uwb_rosbags/<bag_name>/analysis/` by default:

- `range_time.png`
- `peak_tracking.png`
- `range_doppler.png`
- one subdirectory per annotation window with `doppler.png` and `microdoppler.png`
- `summary.json`
- `detections.csv`

If no annotation file is provided, `run_session.py` tries `ground_truth/annotations/<bag_name>.yaml`. If that file does not exist, it auto-creates one full-session `"unknown"` window.

### What `run_session.py` actually does

`run_session.py` orchestrates the offline pipeline but calls the processing stages in other files:

- `uwb_processing/uwb_processing/loaders.py`
  Loads either a rosbag directory or an `.npz` directory and converts it into a `RadarSession`.
- `uwb_processing/uwb_processing/preprocessing.py`
  Runs the core preprocessing:
  global-mean clutter removal, optional offline RDS high-pass filtering, range gating, path selection, and dominant-tap selection.
- `uwb_processing/uwb_processing/detection.py`
  Computes window-level detection scores, Doppler, and micro-Doppler products.
- `uwb_processing/uwb_processing/plotting.py`
  Writes the plots saved under the analysis directory.

Key command-line tuning knobs in `run_session.py`:

- `--default-range-min-m`, `--default-range-max-m`: default ROI if an annotation window does not specify one.
- `--wall-clip-m`: suppresses very near bins before plotting/detection.
- `--motion-threshold`: decision threshold for presence detection.
- `--enable-offline-rds`, `--rds-cutoff-hz`: enable extra offline drift suppression.
- `--carrier-frequency-hz`: used to convert Doppler frequency into radial velocity.
- `--doppler-window-s`, `--microdoppler-window-s`, `--stft-overlap`: spectrogram settings.
- `--topic`: rosbag topic to load if it is not `/uwb/frame_raw`.

## Annotations and batch evaluation

Per-session annotation files live in `ground_truth/annotations/`. The format is documented in `ground_truth/annotations/README.md`.

To run a manifest-based batch evaluation:

```bash
cd /home/ws/src
uwb_batch_eval --manifest ground_truth/annotations/manifest.yaml
```

The batch entry point is `uwb_processing/uwb_processing/batch_eval.py`.

## If you are capturing or decoding something new

Use this map to decide what to modify.

### Only changing recording settings or radar presets

Edit `sensors/launch/sensors.launch.py`:

- session name and duration
- selected preset
- board IP/ports
- which nodes are launched

If the STM32 command/config fields themselves changed, also update `sensors/sensors/uwb_node.py` and `sensors/sensors/sr250_protocol.py`.

### New UDP packet format or new decode logic on the live ROS side

Edit these files:

- `sensors/sensors/uwb_udp_frame_publisher.py`
  This is the live UDP ingest point. Add a new `protocol_mode` branch or change reassembly here.
- `sensors/sensors/sr250_protocol.py`
  This is the main parser/encoder file. Add new packet parsers, field decoding, TLVs, or tap parsing rules here.
- `sensors/launch/sensors.launch.py`
  Wire the new mode or node parameters into the launch path.

If you also want the inspector summary to understand the new payload, update `sensors/sensors/uwb_cir_inspector.py`.

If you want parsed `.npz` archival for the new payload, update `sensors/sensors/uwb_frame_parser_node.py`. That is where raw ROS frames are converted into structured arrays for offline use.

### New offline file format or new offline decode behavior

Edit these files:

- `uwb_processing/uwb_processing/run_session.py`
  Change defaults, CLI options, and orchestration of the offline pipeline.
- `uwb_processing/uwb_processing/loaders.py`
  This is the first file to change if the offline input structure changes. Add a new loader or adapt rosbag/npz decoding here.
- `sensors/sensors/sr250_protocol.py`
  `loaders.py` reuses the same CIR parser for rosbag playback, so payload-format changes usually need to land here too.

### New signal-processing algorithm

Edit these files depending on the change:

- `uwb_processing/uwb_processing/preprocessing.py`
  For clutter suppression, range gating, path selection, and dominant-tap logic.
- `uwb_processing/uwb_processing/detection.py`
  For detection score logic, window evaluation, Doppler, and micro-Doppler generation.
- `uwb_processing/uwb_processing/plotting.py`
  For plots and report outputs.




## Post Processing

If you also want the rosbag for two person walking opposite directions

python3 -m uwb_processing.run_session --input uwb_rosbags/shubo_edison_two_static_walking_medium_2ms --cfar-mode range_doppler --use-2d-ekf --use-group-association --use-dbscan-init --animate

If you also want the rosbag for single person walking

python3 -m uwb_processing.run_session --input uwb_rosbags/shubo_edison_two_static_walking_medium_2ms --cfar-mode range_doppler --use-2d-ekf --use-group-association --use-dbscan-init --animate