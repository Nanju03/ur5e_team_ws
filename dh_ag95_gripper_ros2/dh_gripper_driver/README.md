# AG95 feedback and force setting

The standalone launch now defaults to **20% force setting**. The previous raw
xacro parameter of `0.5` represents 60%; the older driver instead overwrote the
register with 100% on every position command. That overwrite has been removed.
An explicit `force_percent:=20..100` launch argument selects the initial setting.
The launch still initializes the gripper and its fingers may move.

The force controller accepts a one-element `Float64MultiArray` on
`/gripper/force_forward_controller/commands`. Its normalized value `u` must be
between 0 and 1; the requested register setting is `20 + 80*u` percent. For
example, 30% corresponds to `u=0.125`. Confirm the read-back setting before
sending a position goal. This setting is a percentage, not a measured force.

The driver exports the following `left_outer_knuckle_joint` interfaces through
`/gripper/dynamic_joint_states`:

| Interface | Meaning |
|---|---|
| `position` | `0.93*(1000-raw_position)/1000`, in model radians |
| `velocity` | Change of measured position divided by successful-read steady time |
| `grip_state` | 0 moving, 1 empty target reached, 2 object held, 3 object dropped |
| `raw_position` | Actual position feedback, 0 closed through 1000 open |
| `force_percent` | Force-setting register read-back, 20 through 100 percent |
| `communication_age` | Steady-clock seconds since the last fully validated sample |
| `sample_sequence` | Increases only for a newly validated polling sample |

Velocity is NaN on the first sample, after a long polling gap, and once the
sample becomes stale (default 0.5 seconds). Position and other feedback fields
retain the last measured values during an error. Consumers must check both
`communication_age` and advancing `sample_sequence`; a new ROS message header
does not prove that a new serial measurement arrived.

Polling reads documented initialization `0x0200`, grip state `0x0201`, actual
position `0x0202`, and force setting `0x0101`. Writes use `0x0101` for force and
`0x0103` for target position. Force read-back must match the requested setting
before a new position command is sent. CRC, response size, slave, function,
byte count, and write echo are validated. A failed cycle does not refresh the
sample. No independent speed or measured-current register is assumed.

Protocol source: [DH AG Modbus manual V2.5, pp.14–16](https://www.dh-robotics.com/wp-content/uploads/2022/12/AG-%E7%B3%BB%E5%88%97%E6%93%8D%E4%BD%9C%E6%89%8B%E5%86%8C_V2.5MODBUS-%E7%89%88-.pdf).
