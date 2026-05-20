# AoA Multi-Target MP4 Design

Date: 2026-05-20

## Goal

Produce an offline MP4 that shows multiple UWB targets using real AoA estimated from paired RX CIR data.

The output video will contain two synchronized panels:

- left: polar top-view of the radar field of view with tracks in `(angle, range)`
- right: range-angle heatmap with the same tracks overlaid

The design must work with the existing offline `uwb_processing` pipeline and reuse the current CFAR detection, tracking, and animation flow where possible.

## Constraints And Assumptions

- The recorded data contains paired CIR measurements from two RX antennas for the same TX event.
- The RX antenna spacing is approximately `0.10 m`.
- `0 deg` is defined as the direction normal to the front of the board.
- The current dataset contains enough metadata to pair RX samples using `tx_timestamp` and/or `cir_counter`.
- AoA will be estimated offline from CIR phase differences, not from SR250 on-chip AoA notifications.
- The initial implementation targets offline rendering to MP4, not an interactive viewer.

## Key Technical Risk

At `fc ~= 6.5 GHz`, the wavelength is approximately `0.046 m`, while the antenna spacing is `0.10 m`. This exceeds `lambda/2`, so raw phase-difference AoA is angularly ambiguous.

Because of that, the design must not expose a naive unrestricted AoA estimator. The implementation must explicitly include:

- a configurable FoV limit
- temporal continuity constraints per track
- branch selection that prefers consistency with recent track history
- clear documentation that the range-angle heatmap is an approximation in the first version

## Proposed Architecture

### Module Boundaries

- `uwb_processing/preprocessing.py`
  - keep current clutter removal, ROI logic, and per-frame preprocessing
  - do not embed multi-track AoA estimation here

- `uwb_processing/aoa.py` (new)
  - group simultaneous RX observations
  - build paired complex signals for `RX1/RX2`
  - estimate AoA for detections and tracks
  - build framewise range-angle grids for visualization

- `uwb_processing/plotting.py`
  - reusable static helpers for polar and range-angle rendering if needed

- `uwb_processing/animate_cir.py` or new `uwb_processing/animate_aoa.py`
  - render the MP4 with the two-panel synchronized view

- `uwb_processing/run_session.py`
  - expose a new CLI option to generate the AoA animation offline

### Data Flow

1. Load session using the current loader.
2. Preprocess the CIR using the current clutter-removal pipeline.
3. Group CIR records into same-event RX pairs using metadata.
4. Run existing CFAR detection and tracker logic.
5. For each detection or confirmed track, estimate AoA from paired RX data at the associated range bin or a small neighborhood.
6. Smooth AoA over time per track.
7. Build a per-frame range-angle visualization grid.
8. Render a two-panel MP4 and write optional AoA summary artifacts.

## RX Pairing Design

### Pairing Rule

The AoA pipeline will pair CIR observations primarily by:

- same `tx_timestamp`
- same `cir_counter` when needed as a fallback or verification

Within each paired event:

- the TX antenna ID must match
- the RX antenna IDs must form the expected AoA pair
- mismatched or incomplete groups are dropped from AoA estimation

### Output Of Pairing Stage

The pairing stage will produce a framewise structure containing:

- logical event identifier
- timestamp for the paired event
- complex CIR for `RX1`
- complex CIR for `RX2`
- shared TX metadata
- RX IDs and any path identifiers used for consistency checks

This stage is separate because it is the foundation for both track AoA and the range-angle heatmap.

## AoA Estimation Design

### Per-Track AoA

For each detection or track at a selected range tap:

1. Select the tap associated with the detection, or a small local tap window centered on it.
2. Extract complex samples from `RX1` and `RX2`.
3. Form the phase-difference quantity `RX2 * conj(RX1)`.
4. Average locally in range if needed to stabilize the estimate.
5. Compute phase difference `dphi`.
6. Convert `dphi` into candidate angles using the known wavelength and antenna spacing.
7. Select the branch most consistent with:
   - configured FoV
   - previous AoA for the same track
   - continuity constraints between adjacent frames

### Smoothing

Per-track AoA smoothing will be applied after branch selection. The first version should use a simple temporal smoother consistent with the current pragmatic style of the project:

- small-window EMA or Kalman-like scalar smoothing on angle
- hard rejection of implausible angle jumps

The goal is stable visualization, not final scientific AoA calibration.

### Ambiguity Handling

The implementation must treat angular ambiguity as a first-class part of the estimator, not a post-hoc cosmetic fix.

The first version will:

- limit the valid angular search region to a configurable FoV
- keep only the candidate angle branch that maintains track continuity
- optionally mark AoA as invalid for frames where branch choice is too uncertain

## Range-Angle Heatmap Design

### First-Version Strategy

The first version will generate an efficient approximate range-angle map rather than full beamforming.

For each range bin:

- compute paired-RX angular evidence from the same-event RX pair
- assign energy to one or a small number of candidate angle bins
- accumulate into a 2D grid of `angle x range`
- apply light smoothing for readability

This yields a visually meaningful range-angle panel while staying compatible with the available two-channel data.

### Intended Use

The heatmap is primarily a visualization aid for the MP4. The track AoA estimate remains the main interpretable output.

## MP4 Output Design

### Left Panel: Polar Top-View

Display:

- radar origin at the bottom center or center depending on visual composition
- configurable FoV sector
- range rings
- track markers and short history tails
- labels with `track_id`, range, and angle

This panel should communicate the spatial interpretation intuitively.

### Right Panel: Range-Angle Map

Display:

- x-axis: angle
- y-axis: range
- color: returned energy or angular evidence
- overlay of current track positions

This panel should communicate how the spatial interpretation relates to the underlying signal structure.

### Rendering Format

The output will be an offline MP4 written alongside the existing analysis artifacts, for example:

- `aoa_tracks.mp4`

Optional future side artifacts:

- `aoa_summary.json`
- per-track AoA CSV

## CLI And Integration

`run_session.py` should expose a dedicated flag for this output, for example:

- `--animate-aoa`

Associated arguments may include:

- `--aoa-fov-deg`
- `--aoa-rx-spacing-m`
- `--aoa-max-angle-jump-deg`
- `--aoa-grid-angle-bins`

The AoA animation should be optional and should not affect existing non-AoA processing unless explicitly requested.

## Testing Strategy

### Unit Tests

- RX pairing test:
  - verifies that `RX1/RX2` samples are correctly grouped by `tx_timestamp` or `cir_counter`

- synthetic AoA test:
  - generate two-channel complex signals with controlled phase offset
  - verify that recovered AoA follows the expected trend within tolerance

- branch continuity test:
  - verify that the selected AoA branch stays temporally consistent for a moving synthetic target

### Integration Tests

- smoke test for AoA video generation:
  - run the pipeline on a small sample session
  - verify that the MP4 file is created

- artifact consistency test:
  - verify that both panels render without missing-track failures when multiple peaks exist

## Out Of Scope For First Version

- exact SR250 OCPD-style live GUI behavior
- full calibrated beamforming
- absolute-angle scientific validation against a motion-capture ground truth
- interactive browser viewer

## Recommendation

Build the first version around:

- CFAR multi-target detections
- per-track AoA from paired RX CIR
- ambiguity-resolved temporal smoothing
- two-panel offline MP4 output

This is the highest-value design that is still compatible with the data and codebase structure already in place.
