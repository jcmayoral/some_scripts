import argparse
import sys
import os
from os.path import expanduser
from plyfile import PlyData, PlyElement
import io
import h5py
import threading
import time

from pointnet_pc_visualizer import showpoints, trigger_reset


def read_ply(filename):
    """ read XYZ point cloud from filename PLY file """
    """
    with io.open(filename, 'r', encoding="utf-8", errors= 'ignore') as f:
        plydata = PlyData.read(f)
    #plydata = PlyData.read(filename)
    pc = plydata['vertex'].data
    pc_array = np.array([[x, y, z] for x,y,z in pc])
    return pc_array
    """
    return  h5py.File(filename, 'r')


HOME_DIR = expanduser("~")
print HOME_DIR
model_file = "experiments_ws/pointnet2/data/modelnet40_ply_hdf5_2048/ply_data_test0.h5"

BASE_DIR=os.path.dirname(os.path.abspath(__file__))
datafile = os.path.join(HOME_DIR , model_file)


def thread1(name , data):
    showpoints(pc_data[c])

def thread2(name, seconds):
    time.sleep(seconds)
    trigger_reset()

print datafile
pc = read_ply(datafile)
print pc.keys()
pc_data = pc['data']
print type(pc_data)
print pc_data[0]
print pc['label'][0]

c = 0



while True:
    while pc['label'][c] != 25:
        c = c+1
        if c > len(pc['label']):
            break

    print "FOUND AT ", c
    thread1("aaa", pc_data[c])
    c = c+1



seconds = 100

#x = threading.Thread(target = thread1, args=("aa", pc_data[c]))
#y = threading.Thread(target = thread2, args=("bb", seconds))

#x.start()
#y.start()


thread1("aaa", pc_data[c])
