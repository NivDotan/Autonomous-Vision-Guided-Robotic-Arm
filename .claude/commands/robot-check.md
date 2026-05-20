# /robot-check — Audit the robot_sam2_app_v2 codebase for common bugs

Run all checks below and report findings grouped by severity: ERROR (will break), WARNING (likely bug), INFO (style/minor).

## Check 1 — State flag resets

Every flag in `state.py` that gets set to True during operation must be reset in ALL of these places:
- `_start_retreat()` in app.py
- `elif key == ord("a"):` handler (approach toggle off branch) in app.py
- `elif key == ord("r"):` handler in app.py
- `elif key == ord("m"):` handler (mode toggle) in app.py

Flags to verify: `approach_mode`, `object_reached`, `is_frozen`, `gripper_closed`, `returning_home`, `pre_grasp_palm`, `arm_locked`, `grip_local_retry`, `retreat_mode`, `trajectory_active`.

Report any flag that is set True somewhere but not reset in one of the above handlers.

## Check 2 — Tick limit compliance

Every write to `state.target["shoulder"]`, `state.target["elbow"]`, or `state.target["palm"]` must go through `clamp()` with the correct limits from config:
- shoulder: `cfg.SH_MIN` to `cfg.SH_MAX`
- elbow: `cfg.EL_MIN` to `cfg.EL_MAX`
- palm: `cfg.PALM_MIN` to `cfg.PALM_MAX`
- base: 1000 to 3000

Report any assignment that skips `clamp()`.

## Check 3 — Config constant sanity

Read config.py and verify these relationships:
- `VL53_GRIP_DIST_MM < VL53_LOCK_DIST_MM` (grip threshold must be closer than lock threshold)
- `GRIPPER_CLOSE < GRIPPER_OPEN` (closing = lower ticks)
- `GRIP_LOAD_MIN_FRAMES < GRIP_CHECK_FRAMES` (must wait before checking load, but not longer than miss window)
- `SH_MIN < SH_MAX`, `EL_MIN < EL_MAX`, `PALM_MIN < PALM_MAX`
- `GRIP_UP_MAX_TRIES >= 1`

Report any violated relationship.

## Check 4 — Direct hardware writes without state sync

Any call to `hardware.write_ticks()` outside of `_step_proportional()` must be followed by `state.set_curr_and_target()` or manually updating both `state.curr` and `state.target`. Otherwise the proportional stepper will overwrite the hardware position on the next frame.

Search for `write_ticks(` calls and verify each one is followed by a state sync.

## Check 5 — VL53 guard

Any code path that reads `state.vl53_dist_mm` for control decisions must handle the `None` case (sensor not connected or no reading yet).

## Output format

For each check, print:
- CHECK N: PASSED — or —
- CHECK N: N issues found
  - [ERROR/WARNING] description + file:line
