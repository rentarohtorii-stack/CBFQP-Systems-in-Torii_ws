# CBF OctoMap snapshot: mapping PC -> AMIRPC

Both PCs need the updated `amir_driver` package in `Torii_ws`. These files have
been prepared locally; they have not been installed on a remote AMIRPC.

## AMIRPC (start first)

Run the external AMIR communication bridge and the RealSense driver first.
The bridge must publish real encoder data on `/encoder_pub` (`AmirSensor`) and
consume `/motor_sub` (`AmirCmd`). This repository's hardware plugin connects to
those topics; these launches do not start the external bridge or camera driver.

Build once:

```bash
cd /home/dars-note-12/Torii_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mecanumrover_description amir_description amir_moveit_config amir_driver amir_qp_control --symlink-install
source install/setup.bash
```

Start the hardware and snapshot receiver first:

```bash
ros2 launch amir_driver amir_bringup.launch.py
```

This starts the real hardware `ros2_control_node`, robot_state_publisher,
joint_state_broadcaster, arm_controller, and gripper_controller, plus camera mount TF
and the snapshot receiver. Do not run another controller manager for this robot.
It uses the driver's `amir_ros2_controllers.yaml`. `forward_position_controller` and `forward_velocity_controller`
are loaded and configured as inactive for later activation.
The former camera perception launch is preserved as `realsense_perception.launch.py`.

After initialization has finished using the trajectory controller, switch to QP
position/velocity control (do not run QP during the initial trajectory):

```bash
ros2 control switch_controllers --deactivate arm_controller --activate forward_position_controller forward_velocity_controller
ros2 control list_controllers
```

Expected states: arm_controller inactive, both forward controllers active,
joint_state_broadcaster active, gripper_controller active.
Then in a second sourced terminal:

```bash
ros2 launch amir_driver amir_cbf_snapshot_qp.launch.py
```

This starts CBF-QP, the RViz target server, and RViz. `AmirKinematics` from
`kinematics.py` is instantiated inside QP, not started as a separate ROS node.
QP requires the frozen map, measured Joint_1..Joint_5 and a target. It computes
Joint_1..Joint_4 velocity, integrates it into position commands at 50 Hz, and
holds the initially measured Joint_5 position. The selected velocities and matching
position increments are sent through the two forward controllers.

RViz shows the robot and the transformed frozen obstacle spheres. Select
**Interact** and drag the **EE Target** X/Y/Z arrows for continuous target updates
at up to 30 Hz. The release sends the final position; QP keeps tracking after
arrival. The initial marker position is display-only until operated.
**Publish Point** also sends a target. Marker coordinates use `kinematics_frame`.

QP launch arguments: `target_initial_x/y/z`, `target_drag_update_rate`,
`use_rviz_target` (disable for external target publishers), `use_rviz`,
`max_cartesian_velocity` and `max_joint_velocity` (inspect current defaults with
`ros2 launch amir_driver amir_cbf_snapshot_qp.launch.py --show-args`).
Use matching `kinematics_frame` and `frozen_cbf_topic` in both launches.

The driver passes arm controller velocities through to `AmirCmd.vel` (rad/s to
mrad/s), without the legacy arm minimum-speed correction. Only the gripper
retains its legacy 300 mrad/s fallback. Verify the external bridge's angle/velocity
semantics, encoder offsets, joint signs and physical FK. These semantics remain
unverified because the firmware source is unavailable.
The driver now waits for an actual finite encoder sample before initializing
commands or publishing motor commands. Physical calibration, stale-encoder
behavior and emergency-stop behavior still require real hardware validation.

The mount copies `amir_mecanum3_sim.xacro`: parent `link1_1`, translation
`0 0.07 0.05` m, rotation RPY `0 0 1.57` rad. The RealSense driver's optical TF
is used as well: Gazebo's camera body axes must not be treated as optical axes.
Set `camera_frame` if the RealSense body frame name differs. If an existing
robot TF already attaches the camera to the arm, use
`publish_camera_mount_tf:=false` to avoid two parents for the same frame.

The receiver accepts nonempty sphere snapshots while waiting for TF. A newer
snapshot replaces a pending one. After successful transformation it unsubscribes. It
waits until *all* snapshot markers can be transformed at their capture timestamps
into `link0_1`, applies both marker pose and TF, and publishes the frozen result on
`/cbf/frozen_obstacle_spheres`. Failed TF does not publish a partial map or allow QP
to run without its map. The process stays alive to retain the map with transient-local
DDS durability and republishes the identical map every second for restarted
subscribers. The sender also republishes its identical captured snapshot every
second. Later snapshots after a successful freeze are ignored; restart `amir_bringup.launch.py` to capture
again. The frozen map is fixed relative to the arm base and does not follow later
camera/joint motion. Reacquire it if the robot base or environment moves.

Start `amir_bringup.launch.py` before mapping, so its TF buffer covers the capture timestamp.
Synchronize the PCs' clocks for acquisition-time TF. The receiver may receive a
retained sample when started later, but historical dynamic TF may no longer exist.

## Mapping PC

```bash
cd /home/dars-note-12/Torii_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch amir_driver depth_octomap_pc.launch.py snapshot_delay:=10.0
```

The live map, temporal cloud filtering and live CBF visualization continue as
before. Ten wall-clock seconds after the **first nonempty occupied map converted
to CBF spheres**, the sender publishes the latest nonempty map once on
`/cbf/octomap_snapshot`. It waits for fresh data if the stream stops (latest received
sample must be no more than one second old); an empty current map is never replaced
by an older occupied map. `snapshot_delay` changes the delay and
`send_cbf_snapshot:=false` disables sending. Keep this launch alive for retained
DDS delivery. `use_cbf_spheres` must remain true when snapshot sending is enabled.

The transferred data is `visualization_msgs/MarkerArray` containing voxel-enclosing
spheres with sizes and frame IDs, rather than a serialized `.bt` OctoMap tree.
QP already consumes this representation. Both PCs must share `ROS_DOMAIN_ID` and
have DDS discovery/data communication enabled on their network. No host IP is
hard-coded; sending/receiving occurs through ROS 2 topics.

## Verification

Local tests cover delayed one-shot state, marker pose + TF transformation, scale
preservation, invalid/empty maps, and temporal point cloud gating. The launch files
can be inspected without starting hardware using `ros2 launch ... --show-args`.
Actual cross-PC delivery, camera calibration and real arm motion need to be checked
on the two PCs.

## QP ON/OFF

Start disabled while retaining RViz and the stop publisher:

```bash
ros2 launch amir_driver amir_cbf_snapshot_qp.launch.py qp_enabled:=false
```

Switch without restarting the launch:

```bash
ros2 service call /qp_tracking_controller/set_enabled std_srvs/srv/SetBool '{data: false}'
ros2 service call /qp_tracking_controller/set_enabled std_srvs/srv/SetBool '{data: true}'
```

OFF bypasses QP and publishes measured-position holds plus `/amir/qp_stop=true`
at 50 Hz, even if the obstacle map or target is absent. The hardware plugin
receives this stop flag and overrides all six motor angle fields with measured
encoder angles and all velocity fields with zero each hardware write cycle,
bypassing the 300 mrad/s fallback. ON clears the stop flag and requires a newly
operated target; old targets are discarded on each switch. The OFF publisher
must stay running for continuous stop requests. This is a software stop request,
not verification that the physical joints have stopped; the external bridge and
motor firmware must honor these fields. It is not an emergency-stop mechanism.
Do not run a second QP instance or publish conflicting stop/position commands.

The command path is QP -> `/forward_position_controller/commands` ->
JointGroupPositionController plus `/forward_velocity_controller/commands` ->
JointGroupVelocityController -> AmirHardwareInterfaces -> `/motor_sub` ->
external AMIR bridge -> motors. Launch files create these nodes; they do not
forward commands themselves. Measured state returns on `/encoder_pub` ->
AmirHardwareInterfaces -> joint_state_broadcaster -> `/joint_states` -> QP.

## Real hardware URDF

The bringup model `amir_moveit_config/config/amir_mecanum3.urdf.xacro` disables
`include_rover_simulation` and `include_builtin_arm_control`. It contains exactly
one ros2_control system, `amir_mecanum3_ros2`, covering Joint_1..Joint_5 and Gripper
in that order. Each joint exposes position/velocity commands and states.
`hardware_type:=amir_hardware` selects AmirHardwareInterfaces;
`hardware_type:=mock_components` selects GenericSystem instead. The rover's
links and joints remain available for visualization and TF, but this launch does
not control the rover wheels. The standalone rover xacro retains simulation as
its default. Rebuild all three description packages on the robot after updating.
Bringup starts arm_controller and gripper_controller active for initialization,
and both forward controllers inactive for a later manual switch. To return
to trajectory control after using QP, stop the QP launch and restart bringup.
The driver otherwise retains its QP OFF/heartbeat-timeout hold. A running QP OFF
publisher also overrides initial-position motion, so it must be stopped first.


Camera TF must be enabled at camera startup (`publish_tf:=true`). Check
`tf2_echo link0_1 camera_depth_optical_frame` before mapping. Do not replace
capture-time transforms with latest transforms to hide missing historical TF.
For a new acquisition, stop the mapping sender, restart only the receiver, then
start mapping again; keep hardware bringup and camera running. A receiver started
after the capture may lack historical joint transforms even with retained DDS data.


## Per-joint speed magnitudes with QP direction

The QP launch defaults to `joint_speed_mode:=qp` to preserve the original joint
velocity ratios. Use `joint_speed_mode:=fixed` explicitly to select the speed profile. Edit the source file
`amir_qp_control/config/joint_speeds.yaml` to set `joint_speeds` in rad/s, in
Joint_1..Joint_4 order. Joint_5 remains held at zero speed. Values must be finite
and nonnegative; zero disables motion of that joint in fixed mode. The file is
loaded on startup. Rebuild/source after source edits and restart QP, or specify
an external file with `joint_speed_config:=/absolute/path/joint_speeds.yaml`.
Write every entry with a decimal point, e.g. `[0.2, 2.0, 2.0, 0.2]`. ROS rejects
mixed integer/double arrays such as `[0.2, 2, 2, 0.2]` before the QP node can start.

The separate `joint_speed_profile.py` module takes the original tracking-QP
solution and uses its per-joint signs as allowed directions. It chooses velocities
closest to the configured magnitudes, without exceeding those magnitudes. A
second QP retains the original CBF, global joint velocity, and next-position
limits. A nominally stopped joint remains stopped. If these requirements are
infeasible or the solve fails, the controller publishes measured-position holds
and zero velocity; this is not a proof of physical CBF safety when no feasible
command exists. Configured magnitudes are preferences/caps, not unconditional
motor commands. The profile can change the resulting Cartesian motion because
joint velocities are coupled. `max_cartesian_velocity` shapes the nominal tracking
request, as before; it is not a hard bound on the selected Cartesian velocity.

Both position increments and velocity messages use the selected velocity.
Position targets use bounded integration by default (see the next section);
`position_command_mode:=measured` restores `target_q = clip(measured_q + dt * selected_qdot)`. Positions go to
`/forward_position_controller/commands`; five signed velocities go to
`/forward_velocity_controller/commands` (rad/s). OFF, waiting for target/map/TF,
and invalid commands send holds/zero velocity. Status logs show `nominal_qdot`
(the original solution) and `qdot` (the selected speed profile).

Use `joint_speed_mode:=qp` to use the original tracking solution directly, without
the speed profile. Axis tracking weights affect the nominal solution; fixed mode
subsequently adjusts its speed magnitudes. The two topic messages are independent,
not an atomic combined command. Only QP publishes the velocity topic; do not run
a competing velocity publisher.

The hardware driver tracks velocity-interface ownership on switches, avoiding
stale or NaN values on unclaimed interfaces. `arm_controller` retains its own
position/velocity interfaces for initial positioning. After receiving the first
QP stop heartbeat, the driver holds if heartbeats cease for 250 ms or OFF is
requested. Restart bringup before returning to trajectory control after ending
QP; the watchdog otherwise continues to hold.

A gripper position of NaN before the first gripper goal holds only the gripper
at its measured encoder angle with zero speed, bypassing the gripper fallback.
It no longer blocks valid arm commands. Non-finite arm commands, non-finite active
velocity commands, and infinite gripper positions still cause all-joint holds.
`motor_sub HOLD` logs identify explicit stop, heartbeat timeout or invalid-command
holds; individual invalid command logs identify the affected joint.

Copy the updated QP and driver packages to rover, then rebuild/source:

```bash
cd ~/Torii_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select amir_driver amir_qp_control --symlink-install
source install/setup.bash
```

Restart bringup and complete initialization. Then activate both forward
controllers and start QP (the configuration file defaults to 0.2 rad/s per joint):

```bash
ros2 control switch_controllers --deactivate arm_controller --activate forward_position_controller forward_velocity_controller
ros2 launch amir_driver amir_cbf_snapshot_qp.launch.py joint_speed_mode:=qp position_command_mode:=integrated max_joint_velocity:=0.2
```

Inspect the ROS velocity command with:

```bash
ros2 topic echo /forward_velocity_controller/commands --field data
ros2 topic echo /motor_sub --field vel
```

The first uses rad/s, the second mrad/s. Joint_5 is zero; the gripper retains its
separate controller and fallback.


## Bounded position command integration

`position_command_mode:=integrated` is the default. A separate
`position_reference.py` module advances the previous position command by
`dt * selected_qdot`, rather than restarting from measured angle every cycle.
The target stays within absolute joint limits and within `max_position_lead`
(default 0.02 rad, about 1.15 degrees) of each measured angle. Saturation is
stored in the reference, so there is no hidden unbounded integral. The QP also
limits its single-step increment to this lead bound.

When a joint reverses direction, its old lead is discarded before the new step.
A nominally stopped joint targets its measured angle. Holds, missing map/TF/target,
QP OFF and enable/disable switches clear the stored reference. Integration
continues when CBF constraints are active. Before publishing, the controller checks
the sent velocity against the current CBF rows `A*v >= b` and checks the accumulated
endpoint against the same local model. With `b=-alpha*h`, a currently safe pair
must satisfy `A*(target-measured) >= -h`. An already negative h is allowed to
recover at least as much as the original one-step bound `dt*b`.

Only an accumulated endpoint incompatible with these checks is discarded; the
controller tries a fresh measured-position step and resumes integration on later
cycles. If even the fresh command is incompatible, it sends a hold and clears the
reference. This is a local linearized consistency check, not a proof that the
motor controller follows a collision-free path. Physical CBF safety and the
firmware's angle/velocity semantics remain unverified.

At the lead cap the position target stops advancing until feedback catches up,
but the velocity command remains nonzero toward the target. It never exceeds the
selected QP speed or the remaining angle divided by dt. Absolute joint limit
clipping and lead caps can reduce the sent velocity. If no valid position exists
inside both bounds, hold at measurement instead of jumping outside the lead cap.
Logs include `sent_qdot` (published velocity) and `command_lead` (target minus
measurement) so the actual command can be compared to the selected solver result.

Start with explicit velocity bounds:

```bash
ros2 launch amir_driver amir_cbf_snapshot_qp.launch.py joint_speed_mode:=qp position_command_mode:=integrated max_position_lead:=0.02 max_joint_velocity:=0.2
```

For comparison, `position_command_mode:=measured` restores the old position-step
calculation. Mode and lead are read on startup, requiring a QP restart after changes.


## Foreground CBF rendering

`amir_cbf_rviz/ForegroundMarkerArray` draws the CBF displays after normal scene
geometry, with depth checking/writing disabled. Marker alpha is preserved (CBF
sphere publishers use 0.7). OctoMap remains visible through the translucent CBF,
but no longer occludes it; robot geometry is also behind the CBF. Point-cloud
SPHERE_LIST renderables are explicitly redirected to the foreground queue.
Only displays using this plugin receive the foreground behavior. Marker topics,
TF handling, namespaces and deletion/lifetimes use the standard RViz display.

Copy the new `amir_cbf_rviz` package as well as the updated driver/QP sources to
any PC running these RViz configs. Build and source before restarting RViz:

```bash
cd ~/Torii_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select amir_cbf_rviz amir_driver amir_qp_control --symlink-install
source install/setup.bash
```

Regenerate old frozen snapshots to receive the updated alpha. Existing saved RViz
configs can select the new display using Add -> By display type -> amir_cbf_rviz
-> ForegroundMarkerArray, with the same marker-array topic and QoS as before.

## Cartesian axis tracking weights

The QP accepts `tracking_weight_x`, `tracking_weight_y`, and `tracking_weight_z`
(default 1.0 each) at startup. These refer to axes of `kinematics_frame`, not
individual joints. All weights must be finite and nonnegative, with at least
one positive weight. Zero excludes that axis from the tracking cost.

Position errors are `e = target - FK(q)`. The desired Cartesian velocity is
`kp * e`, scaled to the existing Cartesian speed limit. The objective is
`0.5 * sum_i(w_i * ((J*qdot)_i - desired_velocity_i)^2)` plus the existing
joint-velocity regularization. Higher weights prioritize the selected axis when
joint coupling or constraints prevent following all axes equally; they do not
multiply the speed limit or remove CBF/joint limits. All-one weights reproduce
the former objective. The completion tolerance still uses the full 3D distance.
The status log now includes signed `error_xyz` in meters as well as total error.

For example, to prioritize vertical tracking without changing velocity limits:

```bash
ros2 launch amir_driver amir_cbf_snapshot_qp.launch.py tracking_weight_x:=1.0 tracking_weight_y:=1.0 tracking_weight_z:=5.0
```

Copy the changed QP script and launch file to rover, rebuild `amir_qp_control`
and `amir_driver`, source the workspace, and restart the QP launch. Weights are
read on startup; changing parameters on a running node does not update the cost.
