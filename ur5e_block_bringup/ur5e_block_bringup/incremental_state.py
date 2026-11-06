"""Pure state contracts for one-block physical planning. No ROS or hardware."""
import copy
from .physical_motion import incremental_motion_profile, SURFACE_CONTACT_POLICY

POLICY = 'physical_incremental_measured_fingers_v22'
ROUTING_POLICY = 'incremental_continuous_v3'
RETREAT_CLEARANCE = .050
PICKUP_APPROACH_CLEARANCE = .080
TRANSFER_CLEARANCE = .050
PICKUP_POLICY = 'continuous_pick_carry_place_v2'
PICKUP_ROUTE_POLICY = 'compact_above_obstacle_before_staging_v1'


class IncrementalFinished(Exception):
    def __init__(self, reason, state):
        super().__init__(reason)
        self.reason, self.state = reason, validate_state(state)


def validate_state(state):
    if not isinstance(state, dict) or type(state.get('schema')) is not int or state['schema'] != 1:
        raise ValueError('Missing incremental layout state')
    target = state.get('target_cell')
    if type(target) is not int or not 1 <= target <= 100:
        raise ValueError('Invalid incremental target')
    lists = {}
    for key in ('target_blocks', 'remaining_sources', 'obstacle_cells', 'empty_cells'):
        values = state.get(key)
        if (not isinstance(values, list) or any(type(n) is not int or not 1 <= n <= 100 for n in values)
                or len(set(values)) != len(values)):
            raise ValueError('Invalid incremental '+key)
        lists[key] = values
    if not lists['target_blocks'] or lists['target_blocks'][0] != target:
        raise ValueError('Incremental tower must retain its original foundation')
    flattened = [value for values in lists.values() for value in values]
    if len(set(flattened)) != len(flattened):
        raise ValueError('Incremental roles overlap')
    return copy.deepcopy(state)


def after_placement(state, block):
    result = validate_state(state)
    if type(block) is not int or block not in result['remaining_sources']:
        raise ValueError('Incremental source is missing or already stacked')
    result['remaining_sources'].remove(block)
    result['target_blocks'].append(block)
    return result


def recipe_layout(recipe):
    """Bind one complete pick/place to its exact before/after tower state."""
    before = validate_state(recipe.get('incremental_state_before'))
    after = validate_state(recipe.get('incremental_state_after'))
    source = recipe.get('source_cells')
    if (recipe.get('routing_policy') != ROUTING_POLICY or recipe.get('moves') != 1
            or recipe.get('transport_moves') != 1 or recipe.get('buffer_moves') != 0
            or not isinstance(source, list) or len(source) != 1
            or recipe.get('target_cell') != before['target_cell']
            or recipe.get('foundation_layers') != len(before['target_blocks'])
            or recipe.get('layers') != len(before['target_blocks'])+1
            or recipe.get('post_place_retreat_clearance') != RETREAT_CLEARANCE
            or recipe.get('incremental_pickup_approach_clearance') != PICKUP_APPROACH_CLEARANCE
            or recipe.get('incremental_transfer_clearance') != TRANSFER_CLEARANCE
            or recipe.get('incremental_pickup_policy') != PICKUP_POLICY
            or recipe.get('incremental_route_policy') != PICKUP_ROUTE_POLICY
            or recipe.get('incremental_motion_profile') != incremental_motion_profile()
            or recipe.get('surface_contact_policy') != SURFACE_CONTACT_POLICY
            or after != after_placement(before, source[0])):
        raise ValueError('Incremental recipe does not describe exactly one verified placement')
    steps = recipe.get('steps')
    if not isinstance(steps,list) or not all(isinstance(step,dict) for step in steps):
        raise ValueError('Missing incremental pickup sequence')
    grasps = [i for i,step in enumerate(steps) if step.get('phase') == 'grasp']
    places = [i for i,step in enumerate(steps) if step.get('phase') == 'place']
    if len(grasps)!=1 or len(places)!=1 or grasps[0]>=places[0]:
        raise ValueError('Incremental recipe needs one complete pickup/place sequence')
    arms = [step for step in steps if step.get('kind')=='arm']
    if ([step.get('motion_role') for step in arms] != ['pickup','carry','place','retreat']
            or [step.get('phase') for step in arms] != ['transfer','transfer','approach','retreat']
            or any(step.get('surface_contact_policy') != SURFACE_CONTACT_POLICY for step in arms)
            or [step.get('motion_role') for step in steps[:grasps[0]] if step.get('kind')=='arm'] != ['pickup']
            or [step.get('motion_role') for step in steps[grasps[0]+1:places[0]] if step.get('kind')=='arm'] != ['carry','place']):
        raise ValueError('Incremental motion requires one continuous pickup, carry, slow place and retreat')
    towers = {cell: [cell] for cell in before['remaining_sources']}
    towers[before['target_cell']] = before['target_blocks'][:]
    return towers, after


class SessionAuthorization:
    """One explicit operator consent, limited to one unchanged layout/context."""
    def __init__(self, context, initial_state):
        self.context = copy.deepcopy(context)
        self.expected = validate_state(initial_state)

    def check(self, context, recipe):
        recipe_layout(recipe)
        if context != self.context or recipe['incremental_state_before'] != self.expected:
            raise ValueError('Incremental session configuration or committed scene state changed')

    def advance(self, recipe):
        recipe_layout(recipe)
        if recipe['incremental_state_before'] != self.expected:
            raise ValueError('Incremental execution skipped or repeated a tower state')
        self.expected = copy.deepcopy(recipe['incremental_state_after'])
