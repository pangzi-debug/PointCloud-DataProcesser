import numpy as np
from open3d import *
import fire



def convert_pcd(infile,outfile):
	cloud = open3d.io.read_point_cloud(infile)
	open3d.io.write_point_cloud(outfile,cloud,write_ascii=True) #http://www.open3d.org/docs/release/python_api/open3d.io.write_point_cloud.html



def convert(pcdfolder, binfolder):
    '''
        pcdfolder:pcd的文件夹
        binfolder:bin的文件夹
    '''
    current_path = os.getcwd()
    ori_path = os.path.join(current_path, pcdfolder)
    file_list = os.listdir(ori_path)
    des_path = os.path.join(current_path, binfolder)
    if os.path.exists(des_path):
        pass
    else:
        os.makedirs(des_path)
    for file in file_list: 
        (filename,extension) = os.path.splitext(file)
        velodyne_file = os.path.join(ori_path, filename) + '.pcd'
        velodyne_file_new = os.path.join(des_path, filename) + '.pcd'
        convert_pcd(velodyne_file,velodyne_file_new)
    
if __name__ == "__main__":
    fire.Fire()  