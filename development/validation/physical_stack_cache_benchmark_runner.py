#!/usr/bin/python3
"""Run a preserved or current planner on mock domain103; no execution mode."""
import importlib.util
import hashlib
import json
import os
from pathlib import Path
import sys
import time

assert os.environ.get('ROS_DOMAIN_ID') == '103'
assert not any('physical_run' in argument for argument in sys.argv)
base = Path('/home/ksn/ur5e_ws')
variant = os.environ['PHYSICAL_STACK_BENCHMARK_VARIANT']
assert variant in ('before_cache', 'after_cache')
source = (base/'validation/physical_stack_before_cache.py' if variant == 'before_cache' else
          base/'src/ur5e_ros2/ur5e_block_bringup/ur5e_block_bringup/physical_stack.py')
spec = importlib.util.spec_from_file_location('ur5e_block_bringup._cache_benchmark', source)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)
original_run = module.PhysicalStackTask.run
def plan_only(task):
    assert task.mode == 'physical_plan' and task.mock
    return original_run(task)
module.PhysicalStackTask.run = plan_only
clients, timings = {}, {}
create_client = module.PhysicalStackTask.create_client
def counted_client(task, kind, service, *args, **kwargs):
    clients[service] = clients.get(service, 0)+1
    return create_client(task, kind, service, *args, **kwargs)
module.PhysicalStackTask.create_client = counted_client
for method_name in ('scene', 'apply_scene_message', 'validate_samples'):
    original = getattr(module.PhysicalStackTask, method_name)
    def timed(task, *args, _name=method_name, _method=original, **kwargs):
        began = time.monotonic()
        try:
            return _method(task, *args, **kwargs)
        finally:
            record = timings.setdefault(_name, dict(calls=0, seconds=0.))
            record['calls'] += 1
            record['seconds'] += time.monotonic()-began
    setattr(module.PhysicalStackTask, method_name, timed)
began = time.monotonic()
try:
    module.main()
finally:
    report = dict(variant=variant,elapsed_seconds=time.monotonic()-began,
                  source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  clients_created=clients,method_timings=timings,
                  domain=103,mode='physical_plan',movement_goals_sent=0)
    (base/f'validation/physical_stack_cache_benchmark_{variant}.timing.json').write_text(json.dumps(report,indent=2)+'\n')
    print('CACHE_BENCHMARK '+json.dumps(report),flush=True)
