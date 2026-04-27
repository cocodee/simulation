# Repository Guidelines

## Project Structure & Module Organization

This repository contains Isaac Sim workflows for DKK robot scene simulation and video capture.

- `scripts/`: Bash launchers and Python implementations for simulation, rendering, recording, and video encoding.
- `docs/`: Isaac Sim notes, rendering guides, and DKK2 workflow documentation.
- `outputs/`: generated frames, videos, manifests, and run logs. This path is ignored by Git and should stay uncommitted.

Scene assets are expected outside this repository, commonly under `/root/data1/kdi/workspace/supre_robot_assets/scenes/`.

## Build, Test, and Development Commands

There is no package build step. Run workflows from the repository root.

- `bash scripts/run_dkk2_local_isaac.sh`: opens and simulates the DKK2 scene in local Isaac Sim.
- `bash scripts/run_dkk2_motion_video.sh`: records a DKK2 motion video.
- `bash scripts/run_dkk2_replicator_video.sh`: records the DKK2 Replicator workflow.
- `bash scripts/run_dkk5_diff_motion_video.sh`: records DKK5 differential-drive motion.
- `bash scripts/run_dkk5_diff_carry_video.sh`: records the DKK5 carry scenario.

Override defaults with environment variables such as `ISAAC_SIM_DIR`, `SCENE_PATH`, `RUN_DIR`, `ACTIVE_GPU`, `WIDTH`, `HEIGHT`, and `FPS`.

# Codex Project Memory

Before starting any task:
- Read `.codex/TASK_HISTORY.md`
- Read `.codex/DECISIONS.md`

After finishing any task:
- Append a short summary to `.codex/TASK_HISTORY.md`
- Include changed files, commands run, results, unresolved issues, and next steps
- Append architectural decisions to `.codex/DECISIONS.md`

## Coding Style & Naming Conventions

Python uses 4-space indentation, `argparse` for command-line options, `pathlib.Path` for filesystem paths, and JSON manifests for run metadata. Keep Isaac Sim imports that require `SimulationApp` after app initialization, using `# noqa: E402` where needed.

Bash scripts should start with `#!/usr/bin/env bash` and `set -euo pipefail`. Use uppercase environment variables for configurable settings. Follow existing naming patterns: `run_<scene>_<task>.sh`, `record_<scene>_<task>.py`, and `render_<scene>_<mode>.py`.

## Testing Guidelines

No standalone test framework is configured. Validate changes by running the smallest relevant workflow and checking its manifest, frame output, and video output under `outputs/`. For Python-only utilities, prefer fast local checks such as:

```bash
python3 scripts/encode_video.py --help
```

When adding new workflows, include clear validation in the generated manifest so failures can be diagnosed without inspecting every frame.

## Commit & Pull Request Guidelines

Recent commits use short, direct summaries such as `carry`, `modify params`, and `dkk5 and diff move`. Keep commits focused on one workflow or behavior change.

Pull requests should include the workflow changed, commands run, scene paths used, output manifest locations, and screenshots or video samples for rendering changes. Do not commit `outputs/`, `__pycache__/`, local virtual environments, or machine-specific paths unless they are documented defaults.

## Security & Configuration Tips

Do not add secrets or credentials to scripts. Prefer CLI flags or environment variables for machine-specific Isaac Sim and asset paths. Validate external scene paths before launching long-running simulations.
