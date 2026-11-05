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
    [=](int cell){return cell==support?0.:100.;});
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


@pytest.fixture(scope='module')
def nearest_scheduler(tmp_path_factory):
    root = tmp_path_factory.mktemp('nearest_buffer')
    header = Path(__file__).resolve().parents[2]/'ur5e_a3_grid/src/rearrangement.hpp'
    code = root/'nearest.cpp'
    code.write_text(r'''
#include "rearrangement.hpp"
#include <cmath>
#include <iostream>
int main(int argc,char** argv) {
  using namespace rearrangement;
  const std::string mode=argv[1];
  Layout layout;layout.target=14;
  for(int id:{1,9,10,13,14,15,17})layout.towers[id]={id};
  auto pick=[=](const Layout& current,int id) {
    const auto [cell,tier]=current.locate(id);
    if(current.towers.at(cell).back()!=id)return false;
    if(tier>0&&(mode=="no_buffer"||(mode=="nearest_unrecoverable"&&cell==13)))return false;
    if(mode=="nearest_blocks_another"&&id==15&&current.towers.at(13).size()>1)return false;
    return (id!=10&&id!=15)||tier>0||current.towers.at(14).size()==1;
  };
  auto place=[=](const Layout& current,const Move& move) {
    if(mode=="two_buffers_required"&&!move.temporary&&current.parked_once.size()<2)return false;
    if(mode=="nearest_needs_extra"&&!move.temporary&&move.block==15&&
       current.locate(10).first==13&&current.parked_once.size()<2)return false;
    if(!move.temporary)return true;
    if(move.to!=1&&move.to!=9&&move.to!=13&&move.to!=17)return false;
    if((mode=="nearest_path_blocked"||mode=="tie")&&move.to==13)return false;
    return true;
  };
  auto distance=[=](int id) {
    if(mode=="tie"&&(id==9||id==17))return .08;
    const double x=((id-1)%4-1)*.0594,y=((id-1)/4-3)*.060;
    return std::hypot(x,y);
  };
  Search search({15,10,13,9,17,1},pick,place,distance);
  try {
    for(const auto& move:search.plan(layout)) {
      if(!pick(layout,move.block))return 4;
      layout=layout.moved(move);
      std::cout<<move.block<<' '<<move.from<<' '<<move.to<<' '<<move.temporary<<'\n';
    }
  }catch(const std::exception& error){std::cerr<<error.what();return 2;}
}
''')
    binary = root/'nearest'
    subprocess.run(['g++', '-std=c++17', '-O2', '-I', str(header.parent), str(code), '-o', str(binary)], check=True)
    return binary


@pytest.mark.parametrize('mode,destination', [
    ('nearest', 13), ('nearest_unrecoverable', 9),
    ('nearest_blocks_another', 9), ('nearest_path_blocked', 9), ('tie', 9),
    ('nearest_needs_extra', 9),
])
def test_nearest_feasible_target_support_is_used_without_losing_recovery(nearest_scheduler, mode, destination):
    result = subprocess.run([str(nearest_scheduler), mode], text=True, capture_output=True, check=True)
    moves = [tuple(map(int, line.split())) for line in result.stdout.splitlines()]
    assert moves[:3] == [(10, 10, destination, 1), (15, 15, 14, 0), (10, destination, 14, 0)]
    assert sum(move[3] for move in moves) == 1
    assert all(move[2] == 14 for move in moves[1:])


def test_nearby_candidates_are_not_used_when_none_can_be_retrieved(nearest_scheduler):
    result = subprocess.run([str(nearest_scheduler), 'no_buffer'], text=True, capture_output=True)
    assert result.returncode == 2
    assert not result.stdout and 'NOTHING executed' in result.stderr


def test_multiple_temporary_moves_remain_available_when_one_cannot_finish(nearest_scheduler):
    result = subprocess.run([str(nearest_scheduler), 'two_buffers_required'],
                            text=True, capture_output=True, check=True)
    moves = [tuple(map(int, line.split())) for line in result.stdout.splitlines()]
    assert sum(move[3] for move in moves) == 2
    first_final = next(i for i, move in enumerate(moves) if not move[3])
    assert sum(move[3] for move in moves[:first_final]) == 2


@pytest.fixture(scope='module')
def equivalent_target_scheduler(tmp_path_factory):
    root = tmp_path_factory.mktemp('equivalent_target')
    header = Path(__file__).resolve().parents[2]/'ur5e_a3_grid/src/rearrangement.hpp'
    code = root/'equivalent.cpp'
    code.write_text(r'''
#include "rearrangement.hpp"
#include <iostream>
int main(int argc,char** argv) {
  using namespace rearrangement;
  const std::string mode=argv[1];
  if(mode=="keys") {
    Layout first;first.target=14;first.towers={{14,{14,2,3}},{9,{9,10}},{13,{13}}};
    auto second=first;std::swap(second.towers[14][1],second.towers[14][2]);
    if(first.key()==second.key()||first.key(true)!=second.key(true))return 3;
    second=first;std::swap(second.towers[9][0],second.towers[9][1]);
    if(first.key(true)==second.key(true))return 4;
    second=first;second.parked_once.insert(10);
    if(first.key(true)==second.key(true))return 5;
    second=first;second.towers[14][1]=8;
    if(first.key(true)==second.key(true))return 6;
    return 0;
  }
  Layout layout;layout.target=14;
  for(int id:{14,10,9,13,1,2,3,4,5,6,7,8})layout.towers[id]={id};
  auto pick=[](const Layout& current,int id){auto [cell,tier]=current.locate(id);return current.towers.at(cell).back()==id;};
  auto place=[](const Layout&,const Move& move) {
    if(move.temporary)return move.block==10&&(move.to==9||move.to==13);
    return move.block!=10||(move.from!=10&&move.from!=13);
  };
  Search search({10,9,1,2,3,4,5,6,7,8,13},pick,place,
    [](int id){return id==13?.05:id==9?.20:1.;},
    [=](const Layout& current){return current.key(mode=="canonical");});
  try {
    const auto moves=search.plan(layout);int temporary=0;
    for(const auto& move:moves){if(!pick(layout,move.block))return 7;layout=layout.moved(move);temporary+=move.temporary;}
    if(layout.towers.at(14).size()!=12||temporary!=1)return 8;
    std::cout<<search.expanded()<<'\n';
  }catch(const std::runtime_error& error){std::cerr<<error.what();return 2;}
}
''')
    binary = root/'equivalent'
    subprocess.run(['g++', '-std=c++17', '-O2', '-I', str(header.parent), str(code), '-o', str(binary)], check=True)
    return binary


def test_target_geometry_key_ignores_only_completed_cube_order(equivalent_target_scheduler):
    subprocess.run([str(equivalent_target_scheduler), 'keys'], check=True)


def test_equivalent_target_geometry_avoids_exhausting_the_existing_state_limit(equivalent_target_scheduler):
    ordered = subprocess.run([str(equivalent_target_scheduler), 'ordered'], text=True, capture_output=True)
    assert ordered.returncode == 2 and 'search limit' in ordered.stderr
    canonical = subprocess.run([str(equivalent_target_scheduler), 'canonical'], text=True, capture_output=True, check=True)
    assert 0 < int(canonical.stdout.strip()) <= 5000
