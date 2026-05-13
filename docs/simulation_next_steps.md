# Simulation Next Steps

Date: 2026-05-13

This note records the recommended next simulation work after the current DKK2/DKK5 workflows reached the baseline state of opening scenes, running Isaac Sim, recording videos, validating camera views, and writing JSON manifests.

## Current Baseline

- DKK2 has local Isaac Sim, Replicator video, orbit preview, and manifest workflows.
- DKK5 has differential-drive motion and carry video workflows.
- DKK5 has authored head, left-wrist, and right-wrist camera sensor prims in a lightweight USD layer.
- Existing runs have validated non-empty RGB frame output and encoded MP4 videos.
- Known source-scene material and conveyor texture warnings remain, but they have not blocked rendering or encoding.

## Recommended Priorities

1. Validate physics fidelity.
   - Check actual DKK5 displacement, turn radius, wheel direction signs, and wheel slip.
   - Check arm joint limits, gripper behavior, contact stability, and whether carry scenarios are physically valid rather than only visually plausible.

2. Add objective task metrics to manifests.
   - Record robot start and final poses.
   - Record actual traveled distance and heading change.
   - Record end-effector trajectories.
   - Record target object pose before and after manipulation.
   - Record collision, penetration, joint-limit, and contact status where available.
   - Add an explicit success or failure field for task workflows.

3. Expand the scenario matrix.
   - Straight-line base motion.
   - In-place rotation.
   - Curved differential-drive motion.
   - Approach-to-object behavior.
   - Dual-arm carry pose.
   - Move-while-carrying behavior.
   - Synchronized global, head, and wrist camera recording.

4. Add a small regression smoke workflow.
   - Use short duration, low resolution, and low frame count.
   - Verify manifest creation.
   - Verify expected RGB frame count.
   - Verify MP4 readability with `ffprobe`.
   - Verify frames are not blank.
   - Verify required manifest fields are present.

5. Clean up asset warnings when visual quality matters.
   - Resolve missing conveyor and material texture references if the output is intended for demos, visual inspection, or synthetic-data generation.
   - Keep the warning list documented if the assets cannot be corrected immediately.

6. Choose the next project direction explicitly.
   - For demo videos, prioritize camera framing, lighting, motion trajectories, and visual quality.
   - For robot algorithm validation, prioritize physics parameters, sensors, task metrics, and repeatable tests.
   - For synthetic data, prioritize RGB/depth/segmentation/pose annotation and batch generation across scenario variants.

## Suggested Immediate Next Task

Enhance the DKK5 carry workflow with task-success metrics in the manifest. This is the highest-leverage step because it turns the current video workflow into a measurable simulation experiment.
