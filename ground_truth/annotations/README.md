# Annotation Format

Each session annotation file is YAML with this shape:

```yaml
session_id: walking
source: uwb_rosbags/walking
wall_description: 28 cm drywall
notes: Full-session walking example behind a wall.
windows:
  - label: walking_human
    start_s: 0.0
    end_s: 39.3
    wall_present: true
    expected_range_m: [0.5, 4.0]
    notes: Single-person motion through the full bag.
```

`label` drives evaluation defaults:

- labels like `empty`, `absence`, `no_human`, and `no_presence` are treated as expected negatives
- all other labels are treated as expected positives

Use one file per session and reference those files from `manifest.yaml`.
