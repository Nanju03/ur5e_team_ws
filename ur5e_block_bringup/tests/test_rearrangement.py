"""Exercise the production C++ scheduler without ROS or actuator commands."""
from pathlib import Path
import subprocess

import pytest


@pytest.fixture(scope='module')
def scheduler(tmp_path_factory):
    root = tmp_path_factory.mktemp('rearrangement')
    header = Path(__file__).resolve().parents[2]/'ur5e_a3_grid/src/rearrangement.hpp'
    code = root/'probe.cpp'
    code.write_text(r'''
#include "rearrangement.hpp"
#include <iostream>
int main(int argc,char** argv) {
  using namespace rearrangement;
  const int target=std::stoi(argv[1]),a=std::stoi(argv[2]),b=std::stoi(argv[3]),support=std::stoi(argv[4]);
  const std::string mode=argv[5];
  Layout layout;layout.target=target;
  for(int id:{target,a,b,support})layout.towers[id]={id};
  auto pick=[=](const Layout& current,int id) {
    const auto [cell,tier]=current.locate(id);
    if(current.towers.at(cell).back()!=id)return false;
    if(mode=="blocked"&&tier>0)return false;
    const bool constrained=id==b||(mode!="one_way"&&id==a);
    return mode=="ordinary"||!constrained||tier>0||current.towers.at(target).size()==1;
  };
  Search search({a,b,support},pick,[](const Layout&,const Move&){return true;},
    [=](int cell){return cell==support?100.:0.;});
  try {
    for(const auto& move:search.plan(layout)) {
      if(!pick(layout,move.block))return 4;
      layout=layout.moved(move);
      std::cout<<move.block<<' '<<move.from<<' '<<move.to<<' '<<move.temporary<<'\n';
    }
  }catch(const std::runtime_error& error){std::cerr<<error.what();return 2;}
}
''')
    binary = root/'probe'
    subprocess.run(['g++', '-std=c++17', '-O2', '-I', str(header.parent), str(code), '-o', str(binary)], check=True)
    return binary


@pytest.mark.parametrize('ids', [(14, 15, 10, 24), (7, 6, 11, 1), (100, 37, 22, 8)])
def test_mutually_obstructed_sources_are_parked_then_retrieved_from_current_support(scheduler, ids):
    target, first, endangered, support = ids
    result = subprocess.run([str(scheduler), *map(str, ids), 'cycle'], text=True, capture_output=True, check=True)
    moves = [tuple(map(int, line.split())) for line in result.stdout.splitlines()]
    assert moves[:3] == [(endangered, endangered, support, 1),
                         (first, first, target, 0), (endangered, support, target, 0)]
    assert moves[3] == (support, support, target, 0)
    assert sum(move[3] for move in moves) == 1


@pytest.mark.parametrize('mode', ['ordinary', 'one_way'])
def test_safe_order_needs_no_extra_transport(scheduler, mode):
    result = subprocess.run([str(scheduler), '14', '15', '10', '24', mode], text=True, capture_output=True, check=True)
    moves = [tuple(map(int, line.split())) for line in result.stdout.splitlines()]
    assert len(moves) == 3 and not any(move[3] for move in moves)
    if mode == 'one_way':
        assert moves[0][0] == 10


def test_no_recoverable_buffer_fails_before_execution(scheduler):
    result = subprocess.run([str(scheduler), '14', '15', '10', '24', 'blocked'], text=True, capture_output=True)
    assert result.returncode == 2
    assert not result.stdout and 'NOTHING executed' in result.stderr
