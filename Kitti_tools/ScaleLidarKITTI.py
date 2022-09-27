from cProfile import label
from ctypes import LittleEndianStructure
import numpy as np
import os 
import open3d as o3d
import fire

"""
    # 适用于lidar坐标(Left Bottom)位于原点上 缩放过程中是对应与原点进行缩放的 
    # 将lidar对应于kitti的标签同样进行缩放  
"""
# 将指定文件夹中的文件的左下角移动到中心点
def readKitti(kittiPath):
    # 读取pcd文件，并且将pcd文件转换成数组结构
    lidar_data = np.fromfile(kittiPath, dtype=np.float32).reshape(-1, 4)  
    return lidar_data
def writeKitti(lidarData,SRC_path,DST_path):
    # 从SRC_path得到pcd名字
    kitti_name = os.path.basename(SRC_path)
    # construct new pcd fullpath
    DST_KittiPath = os.path.join(DST_path,kitti_name)
    # 将pcd列表格式通过o3d库写入指定路径文件中 http://www.open3d.org/docs/0.9.0/tutorial/Basic/working_with_numpy.html
    lidarData.tofile(DST_KittiPath)

def writeLabel(label_data,SRC_path,DST_path):
    # get Name From SRC_path
    label_name = os.path.basename(SRC_path)
    label_name = label_name.replace(".bin",'.txt')
    DST_labelPath = os.path.join(DST_path,label_name)

    with open(DST_labelPath,'w') as F: # 重写
        for label_d in label_data:
            print(label_d)
            F.writelines(" ".join(str(item) for item in label_d))
            print(" ".join(str(item) for item in label_d))
            F.writelines("\n")
    # ^将label写入到目标文件中

def scaleKitti(lidarData, x_scale,y_scale,z_scale):
    lidarData[:,0] = lidarData[:,0] * x_scale
    lidarData[:,1] = lidarData[:,1] * y_scale
    lidarData[:,2] = lidarData[:,2] * z_scale
    return lidarData # newL_Data


def scaleLabel(labelData, x_scale,y_scale,z_scale):
    newLabelData = []
    for l_d in labelData:
        print(l_d)
        # scale x11 l10 y12 w⑨ z13 h⑧
        l_d[11] = str(float(l_d[11]) * x_scale)
        l_d[10] = str(float(l_d[10]) * x_scale)

        l_d[12] = str(float(l_d[12]) * y_scale)
        l_d[9] = str(float(l_d[9]) * y_scale)

        l_d[13] = str(float(l_d[13])* z_scale)
        l_d[8] = str(float(l_d[8]) * z_scale)
        newLabelData.append(l_d)
    
    return newLabelData
def findAllKittiPath(binfolder):
    # 找到当前文件目录下所有的pcd文件(完整路径)
    bin_list = []
    for root,dirs,files in os.walk(binfolder):
        for f in files:
            print(f)
            if f.endswith(".bin"):
                bin_path = os.path.join(root,f)
                bin_list.append(bin_path)
    return bin_list
def getLabelByKitti(kitti,srcLabelFolder):
    baseName  = os.path.basename(kitti)
    labelName = baseName.replace("bin",'txt')
    labelPath = os.path.join(srcLabelFolder,labelName) # |构Label路径\
    # 一行行读出label
    if os.path.exists(labelPath):
        return [ line.rstrip().lstrip().split(" ") for  line in open(labelPath).readlines()] # 传回label数组
    else:
        return None
def PROCESS(srcfolder,dstfolder,x_scale,y_scale,z_scale):
    # srcfolder-> 1, pcdfolder 2, labelfolder
    # dstfolder -> 1，pcdfolder 2,labelfolder
    srcbinfolder = os.path.join(srcfolder,"velodyne")
    dstbinfolder = os.path.join(dstfolder,"velodyne")
    srclabelfolder = os.path.join(srcfolder,"label_2")
    dstlabelfolder = os.path.join(dstfolder,"label_2")

    bin_list = findAllKittiPath(srcbinfolder)
    print(bin_list)
    for bin in bin_list:
        label_data = getLabelByKitti(bin,srclabelfolder)
        if label_data is None:
            continue

        # 读取pcd文件到数组中
        lidar_data = readKitti(bin)
        # 得到pcd文件的坐标较小值
        lidar_data_new = scaleKitti(lidar_data,x_scale,y_scale,z_scale)
        label_data_new = scaleLabel(label_data,x_scale,y_scale,z_scale)
        writeKitti(lidar_data_new, bin, dstbinfolder)
        # Write_^\/
        writeLabel(label_data_new,bin,dstlabelfolder)                            
        print(bin,"DONE!!!!!!!!")
        
if __name__ == "__main__":
    fire.Fire()

