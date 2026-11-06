"""Physical motion policy shared by launch, GUI and recipe validation.

Scaling factors multiply the model's joint limits and stay within (0, 1].
Change the stored recipe policies when changing this timing/clearance contract.
"""

CONTACT_CLEARANCE = .020
APPROACH_CLEARANCE = .020
ARM_SPEEDS = {'approach': .10, 'contact': .10, 'descent_fast': .50,
              'lift': 1., 'retreat': 1., 'transfer': 1.}

# One-block mode: each pair is (velocity scaling, acceleration scaling).
# Layer numbers include the foundation block. Changes invalidate old recipes.
FAST_LAYERS = 15
PICKUP_SPEED = (1., 1.)
LOW_TOWER_SPEED = (1., 1.)
HIGH_TOWER_SPEED = (.5, .3)
SURFACE_CONTACT_POLICY = 'bounded_depth_1e-5_v1'
HOLDING_FINGER_LOWER = .690
HOLDING_FINGER_UPPER = .733


def incremental_motion_profile():
    return dict(fast_layers=FAST_LAYERS, pickup=list(PICKUP_SPEED),
                low=list(LOW_TOWER_SPEED), high=list(HIGH_TOWER_SPEED),
                place=[ARM_SPEEDS['approach']]*2)


def incremental_arm_scaling(role, layer):
    if role == 'pickup':
        return PICKUP_SPEED
    if role == 'place':
        return (ARM_SPEEDS['approach'],)*2
    if role not in ('carry', 'retreat'):
        raise ValueError('Unknown continuous arm motion role')
    return LOW_TOWER_SPEED if layer <= FAST_LAYERS else HIGH_TOWER_SPEED


def incremental_motion_parameters():
    return dict(incremental_fast_layers=FAST_LAYERS,
                incremental_pickup_velocity_scaling=PICKUP_SPEED[0],
                incremental_pickup_acceleration_scaling=PICKUP_SPEED[1],
                incremental_low_velocity_scaling=LOW_TOWER_SPEED[0],
                incremental_low_acceleration_scaling=LOW_TOWER_SPEED[1],
                incremental_high_velocity_scaling=HIGH_TOWER_SPEED[0],
                incremental_high_acceleration_scaling=HIGH_TOWER_SPEED[1])


def motion_parameters():
    """Return fresh parameters so saved selections cannot change the policy."""
    return dict(
        split_stack_speeds=True,
        velocity_scaling=ARM_SPEEDS['transfer'], acceleration_scaling=ARM_SPEEDS['approach'],
        stack_acceleration_scaling=ARM_SPEEDS['approach'],
        approach_velocity_scaling=ARM_SPEEDS['approach'],
        approach_acceleration_scaling=ARM_SPEEDS['approach'],
        transfer_velocity_scaling=ARM_SPEEDS['transfer'],
        transfer_acceleration_scaling=ARM_SPEEDS['transfer'],
        descent_velocity_scaling=ARM_SPEEDS['descent_fast'],
        descent_acceleration_scaling=ARM_SPEEDS['descent_fast'],
        physical_contact_clearance=CONTACT_CLEARANCE,
        physical_approach_clearance=APPROACH_CLEARANCE,
        holding_finger_lower=HOLDING_FINGER_LOWER,
        holding_finger_upper=HOLDING_FINGER_UPPER)
