"""Exercise the production C++ settling budget; no ROS node or actuator."""
from pathlib import Path
import subprocess

import pytest


@pytest.fixture(scope='module')
def budget(tmp_path_factory):
    folder = tmp_path_factory.mktemp('gazebo_settle')
    header = Path(__file__).resolve().parents[2]/'ur5e_a3_grid/src/gazebo_settling.hpp'
    source = folder/'probe.cpp'
    source.write_text(r'''
#include "gazebo_settling.hpp"
#include <iostream>
#include <string>
int main(int argc,char** argv) {
  try {
    gazebo_settling::Budget budget(std::stod(argv[1]),std::stod(argv[2]),std::stod(argv[3]));
    std::string sim,wall,error;
    while(std::cin>>sim>>wall>>error) {
      const auto result=budget.observe(std::stod(sim),std::stod(wall),std::stod(error));
      const char* name="waiting";
      switch(result) {
        case gazebo_settling::Result::settled:name="settled";break;
        case gazebo_settling::Result::extended:name="extended";break;
        case gazebo_settling::Result::simulation_timeout:name="simulation_timeout";break;
        case gazebo_settling::Result::wall_timeout:name="wall_timeout";break;
        default:break;
      }
      std::cout<<name<<' '<<budget.deadline()<<'\n';
    }
  }catch(const std::exception& error){std::cerr<<error.what();return 3;}
}
''')
    binary = folder/'probe'
    subprocess.run(['g++', '-std=c++17', '-O2', '-I', str(header.parent), str(source), '-o', str(binary)], check=True)

    def run(samples, initial=15, maximum=45, wall=300, check=True):
        result = subprocess.run([str(binary), str(initial), str(maximum), str(wall)],
            input=''.join(f'{sim} {elapsed} {error}\n' for sim, elapsed, error in samples),
            text=True, capture_output=True, check=check)
        return result, [line.split() for line in result.stdout.splitlines()]
    return run


REPORTED = [(1.168, 5.019, .009112), (2.358, 10.036, .008688), (3.528, 15.051, .008291),
            (4.703, 20.068, .007911), (5.897, 25.084, .007543), (7.093, 30.102, .007192),
            (8.266, 35.118, .006862), (9.411, 40.135, .006556), (10.597, 45.150, .006252),
            (11.718, 50.168, .005979), (12.897, 55.185, .005704), (13.921, 60.204, .005476),
            (15.012, 65.190, .005244)]


def test_reported_failure_is_reproduced_and_only_the_converging_budget_extends(budget):
    _, original = budget(REPORTED, maximum=15)
    assert original[-1][0] == 'simulation_timeout'
    _, fixed = budget(REPORTED)
    assert fixed[-1][0] == 'extended' and float(fixed[-1][1]) > 20
    assert not any(row[0] == 'settled' for row in fixed)


def test_extension_still_requires_the_original_position_tolerance(budget):
    # Synthetic continuation tests the strict gate, not an observed robot trace.
    _, rows = budget(REPORTED+[(16, 70, .0051), (17, 75, .005001), (17.2, 76, .005)])
    assert [row[0] for row in rows[-3:]] == ['waiting', 'waiting', 'settled']


@pytest.mark.parametrize('mode', ['stalled', 'diverging', 'jitter'])
def test_nonconverging_error_does_not_receive_extra_time(budget, mode):
    samples = [(t, t*4, .008 if mode == 'stalled' else
                .008+t*.0001 if mode == 'diverging' else .008+(-1)**t*.00004) for t in range(16)]
    _, rows = budget(samples)
    assert rows[-1][0] == 'simulation_timeout'
    assert not any(row[0] in ('extended', 'settled') for row in rows)


def test_slow_progress_is_bounded_by_the_absolute_simulation_limit(budget):
    _, rows = budget([(t, t*4, .008-t*.00005) for t in range(46)])
    assert any(row[0] == 'extended' for row in rows)
    assert rows[-1][0] == 'simulation_timeout'
    assert all(float(row[1]) <= 45 for row in rows)


def test_paused_simulation_is_bounded_by_the_wall_watchdog(budget):
    _, rows = budget([(0, 0, .01), (3, 20, .009), (3, 299, .009), (3, 300, .009)])
    assert rows[-1][0] == 'wall_timeout'


@pytest.mark.parametrize('options', [dict(initial=0), dict(maximum=14), dict(wall=0),
                                    dict(maximum=float('nan')), dict(wall=float('inf'))])
def test_invalid_time_limits_are_rejected(budget, options):
    result, _ = budget([], check=False, **options)
    assert result.returncode == 3


@pytest.mark.parametrize('samples', [[(1, 1, .01), (.9, 2, .01)], [(0, 0, float('nan'))],
                                   [(0, 0, -.01)]])
def test_clock_reset_or_invalid_feedback_is_rejected(budget, samples):
    result, _ = budget(samples, check=False)
    assert result.returncode == 3
