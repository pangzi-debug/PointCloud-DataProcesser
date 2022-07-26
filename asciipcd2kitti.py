import os
import numpy as np
import fire

def read_pcd(filepath):
    print(filepath)
    lidar = []
    with open(filepath,encoding='utf-8') as f:
        line = f.readline().strip()
        print(line)
        while line:
            linestr = line.split(" ")
            if len(linestr) == 3: 
                linestr_convert = list(map(float, linestr))
                #print(linestr_convert)
                linestr_convert.append(0)
                lidar.append(linestr_convert)
            line = f.readline().strip()
    return np.array(lidar)


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
        pl = read_pcd(velodyne_file)  
        pl = pl.reshape(-1, 4).astype(np.float32)
        newfilename = '%06d'%int(filename)
        velodyne_file_new = os.path.join(des_path, newfilename) + '.bin'
        pl.tofile(velodyne_file_new)
    
if __name__ == "__main__":
    fire.Fire()   