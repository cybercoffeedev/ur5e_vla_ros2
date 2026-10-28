# UR5e model (MuJoCo Menagerie)

Third-party MJCF description of the UR5e arm, used by `vla_sim` for M1. It is not authored by this
project and must not be modified here.

## Provenance

| Field | Value |
|---|---|
| Source | <https://github.com/google-deepmind/mujoco_menagerie> |
| Directory | `universal_robots_ur5e/` |
| Commit | `71f066ad0be9cd271f7ed58c030243ef157af9f4` (2026-07-04) |
| License | BSD-3-Clause, see `LICENSE` |
| Copyright | 2018 ROS Industrial Consortium |
| Model version | requires MuJoCo 2.3.3 or later; this project pins 3.12.0 |

The model is a simplified UR5e description derived from the ROS Industrial URDF. Its joints are
`shoulder_pan`, `shoulder_lift`, `elbow`, `wrist_1`, `wrist_2`, `wrist_3`, and the `home` keyframe
is `-1.5708 -1.5708 1.5708 -1.5708 -1.5708 0` rad.

## What is tracked

| Path | Tracked | Notes |
|---|---|---|
| `SHA256SUMS` | yes | Expected digest for every fetched file |
| `LICENSE` | yes | Upstream licence text, retained per BSD-3-Clause |
| `README.md` | yes | This file |
| `ur5e.xml`, `scene.xml`, `assets/*.obj` | no | Fetched, ignored by Git |

## Fetching

```bash
python3 scripts/fetch_mujoco_ur5e.py
```

The script downloads each file listed in `SHA256SUMS` from the pinned commit, verifies its digest,
writes it atomically, and skips files whose local digest already matches. `--force` re-downloads
everything. It uses only the Python standard library.

A digest mismatch is an error, not a warning: the model must be byte-identical to the pinned
commit before it is used as a simulation plant.

## Size

22 files, about 30 MB, almost entirely visual meshes. They are text OBJ files and are therefore not
stored in Git; `SHA256SUMS` is the reproducible record of what belongs here.
