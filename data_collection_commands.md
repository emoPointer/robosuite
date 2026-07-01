# Data Collection Commands

Run all commands from `/home/emopointer/robosuite`.

For visual debugging, keep `--workers 1`, change `--num_demos` to `1`, and append `--render`.

Boolean flags that change the task behavior, such as `--no_video`, `--no_camera_obs`, `--disable_attach_*`, and
`--disable_handle_check`, are intentionally not included in the normal collection commands.

## Lemon

```bash
python -u robosuite/scripts/arx_lemon_eef_pose_collect.py \
    --num_demos 10 \
    --workers 1 \
    --max_steps 500 \
    --max_episodes 500 \
    --record_freq 20 \
    --control_freq 20 \
    --seed 0 \
    --eef_kp 150.0 \
    --motion_speed 0.12 \
    --grasp_yaw_offset_deg 90 \
    --lift_yaw_deg 0 \
    --lift_height 0.12 \
    --place_hover_height 0.16 \
    --success_stable_steps 20 \
    --save_dir lemon_eef_pose_demonstrations
```

Output directory:

```text
/home/emopointer/robosuite/lemon_eef_pose_demonstrations/
```

## BoxInBin

```bash
python -u robosuite/scripts/arx_box_in_bin_eef_pose_collect.py \
    --num_demos 50 \
    --workers 1 \
    --max_steps 1000 \
    --max_episodes 500 \
    --record_freq 20 \
    --control_freq 20 \
    --seed 0 \
    --eef_kp 150.0 \
    --motion_speed 0.12 \
    --grasp_yaw_offset_deg 0 \
    --approach_height 0.18 \
    --lift_height 0.12 \
    --place_hover_height 0.16 \
    --box_release_height 0.075 \
    --retreat_height 0.12 \
    --success_stable_steps 20 \
    --save_dir box_in_bin_eef_pose_demonstrations
```

Output directory:

```text
/home/emopointer/robosuite/box_in_bin_eef_pose_demonstrations/
```

## MugHang

```bash
python -u robosuite/scripts/arx_mug_hang_eef_pose_collect.py \
    --num_demos 50 \
    --workers 1 \
    --max_steps 1600 \
    --max_episodes 500 \
    --record_freq 20 \
    --control_freq 20 \
    --seed 0 \
    --eef_kp 150.0 \
    --motion_speed 0.12 \
    --shape_id b4ae56d6 \
    --mug_scale 1.0 \
    --grasp_yaw_offset_deg -90 \
    --grasp_roll_offset_deg 90 \
    --grasp_offset_x 0.0 \
    --grasp_offset_y -0.05 \
    --grasp_offset_z 0.03 \
    --grasp_edge_fraction 0.95 \
    --grasp_height_fraction 0.70 \
    --grasp_z_offset 0.0 \
    --approach_height 0.08 \
    --lift_height 0.12 \
    --transit_clearance 0.05 \
    --pre_insert_clearance 0.05 \
    --insert_depth 0.03 \
    --target_center_y 0.0 \
    --target_center_z 0.055 \
    --mug_hang_local_x 0.0 \
    --mug_hang_local_y 0.03 \
    --mug_hang_local_z 0.0 \
    --mug_handle_axis x \
    --hang_settle_time 1.2 \
    --release_pause 1.0 \
    --retreat_distance 0.08 \
    --retreat_height 0.08 \
    --debug_hang_alignment \
    --alignment_log_interval 10 \
    --alignment_error_warn 0.015 \
    --alignment_correction_gain 2.0 \
    --alignment_correction_max 0.06 \
    --handle_anchor_tolerance 0.018 \
    --handle_axis_alignment_min 0.75 \
    --handle_segment_margin 0.005 \
    --post_task_wait_steps 300 \
    --success_stable_steps 20 \
    --save_dir mug_hang_eef_pose_demonstrations
```

Output directory:

```text
/home/emopointer/robosuite/mug_hang_eef_pose_demonstrations/
```

## Square

```bash
python -u robosuite/scripts/arx_square_eef_pose_collect.py \
    --num_demos 50 \
    --workers 1 \
    --max_steps 1400 \
    --max_episodes 500 \
    --record_freq 20 \
    --control_freq 20 \
    --seed 0 \
    --eef_kp 150.0 \
    --motion_speed 0.12 \
    --grasp_yaw_offset_deg 0 \
    --grasp_local_x 0.064 \
    --grasp_local_y 0.0 \
    --grasp_local_z 0.012 \
    --grasp_z_offset 0.0 \
    --approach_height 0.14 \
    --lift_height 0.12 \
    --place_hover_height 0.14 \
    --pre_insert_height 0.06 \
    --nut_center_z_offset_from_peg -0.02 \
    --release_pin_pause 1.5 \
    --retreat_handle_clearance -0.10 \
    --retreat_height 0.02 \
    --post_task_wait_steps 120 \
    --success_stable_steps 20 \
    --save_dir square_eef_pose_demonstrations
```

Output directory:

```text
/home/emopointer/robosuite/square_eef_pose_demonstrations/
```

Recorded Square demos used `release_pin_pause=1.5` and `align_nut_rotation_to_peg=False`. Do not append
`--align_nut_rotation_to_peg` unless deliberately changing the behavior.

## Drawer

```bash
python -u robosuite/scripts/arx_drawer_eef_pose_collect.py \
    --num_demos 50 \
    --workers 1 \
    --max_steps 1200 \
    --max_episodes 500 \
    --record_freq 20 \
    --control_freq 20 \
    --seed 0 \
    --eef_kp 150.0 \
    --motion_speed 0.10 \
    --contact_local_x 0.14 \
    --contact_local_y 0.0 \
    --contact_local_z 0.04 \
    --pull_distance 0.03 \
    --drawer_qpos_start 0.06 \
    --drawer_qpos_end 0.09 \
    --drawer_qpos_closed 0.025 \
    --approach_height 0.10 \
    --post_open_lift_height 0.12 \
    --contact_pause 0.5 \
    --pull_duration 1.5 \
    --post_pull_pause 1.0 \
    --pod_grasp_z_offset 0.0 \
    --pod_lift_height 0.12 \
    --pod_place_front_local_x 0.13 \
    --pod_place_local_x 0.145 \
    --pod_place_local_y 0.0 \
    --pod_place_local_z 0.035 \
    --pod_place_hover_height 0.12 \
    --pod_release_pause 1.0 \
    --pod_retreat_local_x 0.06 \
    --pod_retreat_height 0.12 \
    --close_outside_offset 0.035 \
    --close_contact_pause 0.4 \
    --close_duration 1.8 \
    --post_task_wait_steps 60 \
    --drawer_qpos_tolerance 0.003 \
    --success_stable_steps 20 \
    --save_dir drawer_eef_pose_demonstrations
```

Output directory:

```text
/home/emopointer/robosuite/drawer_eef_pose_demonstrations/
```

Recorded Drawer demos rely on the default scripted helpers being enabled:
`drive_drawer_qpos=True`, `attach_pod_after_grasp=True`, `snap_pod_to_grasp_frame_on_attach=True`,
`carry_pod_with_drawer_during_close=True`, and `snap_pod_to_drawer_place_on_release=True`. Do not append the matching
`--disable_*` flags for normal collection.
