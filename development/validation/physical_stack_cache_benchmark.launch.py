"""Use unchanged production task parameters, swapping only the Python module."""
import importlib.util
from pathlib import Path


def generate_launch_description():
    path = Path('/home/ksn/ur5e_ws/src/ur5e_ros2/ur5e_block_bringup/launch/block_task.launch.py')
    spec = importlib.util.spec_from_file_location('_cache_benchmark_task_launch', path)
    task_launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(task_launch)
    node = task_launch.Node
    def benchmark_node(*args, **kwargs):
        assert kwargs['executable'] == 'physical_stack'
        kwargs['executable'] = '/home/ksn/ur5e_ws/validation/physical_stack_cache_benchmark_runner.py'
        return node(*args, **kwargs)
    task_launch.Node = benchmark_node
    return task_launch.generate_launch_description()
