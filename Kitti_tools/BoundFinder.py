
# 全部顶点中的最大值 记录
max_x = 0
max_y = 0
max_z = 0

min_x = 1000
min_y = 1000
min_z = 1000
src_l = "/root/files/sjwang/YOLO3D_GREATCODE_0921/YOLO3D-Stone/dataset/kitti/training/velodyne/"

import os
import numpy as np
def load_PC(lidar_path):
    return np.fromfile(lidar_path, dtype=np.float32).reshape(-1, 4)
# Get lidar
# lidar_path = "/root/files/sjwang/YOLO3D_GREATCODE_0921/YOLO3D-Stone/dataset/kitti/training/velodyne/000000.bin"
# lidar_data = load_PC(lidar_path)
# print(np.max(lidar_data,axis=0))
def load_PC(lidar_file): # array of lidar
	return  np.fromfile(lidar_file, dtype=np.float32).reshape(-1, 4)
for root,dirs,files in os.walk(src_l):
    for lidar_file in files:        
        lidar_data = load_PC(os.path.join(root,lidar_file))

        # 找到 当前场景中对打的x|y|z
        cur_max_X,cur_max_Y,cur_max_Z ,_= np.max(lidar_data,axis=0)

        max_x = max(max_x,cur_max_X)
        max_y = max(max_y,cur_max_Y)
        max_z = max(max_z,cur_max_Z)

        cur_min_X,cur_min_Y,cur_min_Z ,_= np.max(lidar_data,axis=0)

        min_x = min(min_x,cur_min_X)
        min_y = min(min_y,cur_min_Y)
        min_z = min(min_z,cur_min_Z)

print(max_x)
print(max_y)
print(max_z)

print(min_x)
print(min_y)
print(min_z)