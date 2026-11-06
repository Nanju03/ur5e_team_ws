"""Cube orientations produced by side placement, using the production C++ test."""
from itertools import permutations,product
from pathlib import Path
import subprocess
import numpy as np
import pytest


@pytest.fixture(scope='module')
def level_box(tmp_path_factory):
    folder=tmp_path_factory.mktemp('incremental_box')
    header=Path(__file__).resolve().parents[2]/'ur5e_a3_grid/src/incremental_geometry.hpp'
    code=folder/'probe.cpp'
    code.write_text('''
#include "incremental_geometry.hpp"
#include <iostream>
int main(int argc,char** argv) {
  Eigen::Vector3d size;Eigen::Matrix3d R;
  for(int i=0;i<3;++i)size[i]=std::stod(argv[i+1]);
  for(int r=0;r<3;++r)for(int c=0;c<3;++c)R(r,c)=std::stod(argv[4+r*3+c]);
  std::cout<<incremental_geometry::levelBox(R,size);
}
''')
    binary=folder/'probe'
    subprocess.run(['g++','-std=c++17','-O2','-I/usr/include/eigen3','-I',str(header.parent),str(code),'-o',str(binary)],check=True)
    def check(R,size=(.025,.025,.025)):
        result=subprocess.run([str(binary),*map(str,size),*map(str,np.asarray(R).flat)],text=True,capture_output=True,check=True)
        return result.stdout=='1'
    return check


def test_all_24_cube_axis_permutations_remain_stackable(level_box):
    count=0
    for axes in permutations(range(3)):
        for signs in product((-1,1),repeat=3):
            R=np.eye(3)[:,axes]@np.diag(signs)
            if np.linalg.det(R)<0:continue
            assert level_box(R)
            # A 25mm cube has the same occupied extents after every quarter turn.
            assert np.allclose(np.abs(R)@np.full(3,.025),np.full(3,.025))
            count+=1
    assert count==24


@pytest.mark.parametrize('angle',[10,25,45,70])
def test_tilted_cubes_and_horizontal_tall_obstacles_are_rejected(level_box,angle):
    a=np.deg2rad(angle)
    R=np.array([[1,0,0],[0,np.cos(a),-np.sin(a)],[0,np.sin(a),np.cos(a)]])
    assert not level_box(R)
    side=np.array([[1,0,0],[0,0,-1],[0,1,0]])
    assert not level_box(side,(.025,.025,.15))
    assert level_box(np.eye(3),(.025,.025,.15))
