from ctypes import LittleEndianStructure
import numpy as np
import os 
import open3d as o3d 
import fire
# 将指定文件夹中的文件的左下角移动到中心点
def readPcd(pcdPath):
    # 读取pcd文件，并且将pcd文件转换成数组结构
    pcd = o3d.io.read_point_cloud(pcdPath)
    lidar_data = np.asarray(pcd.points)
    return lidar_data
def writePcd(lidarData,SRC_path,DST_path):
    # 从SRC_path得到pcd名字
    pcd_name = os.path.basename(SRC_path)
    # construct new pcd fullpath
    DST_pcdPath = os.path.join(DST_path,pcd_name)
    # 将pcd列表格式通过o3d库写入指定路径文件中 http://www.open3d.org/docs/0.9.0/tutorial/Basic/working_with_numpy.html
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(lidarData)
    o3d.io.write_point_cloud(DST_pcdPath,pcd,write_ascii=True)

def scalePcd(lidarData, x_scale,y_scale,z_scale):
    lidarData[:,0] = lidarData[:,0] * x_scale
    lidarData[:,1] = lidarData[:,1] * y_scale
    lidarData[:,2] = lidarData[:,2] * z_scale
    return lidarData
def findAllPcdPath(pcdfolder):
    # 找到当前文件目录下所有的pcd文件(完整路径)
    pcd_list = []
    for root,dirs,files in os.walk(pcdfolder):
        for f in files:
            print(f)
            if f.endswith(".pcd"):
                pcd_path = os.path.join(root,f)
                pcd_list.append(pcd_path)
    return pcd_list

def PROCESS(pcdfolder,DST_folder,x_scale,y_scale,z_scale):
    pcd_firstorder_folder = pcdfolder # 第一级路径存储了所有存储pcd文件的文件夹
    for root,dirs,files in os.walk(pcd_firstorder_folder): 
        for subpcdfolder in dirs: # 遍历大文件夹里面的每个放pcd的文件见
            pcdfolder = os.path.join(root,subpcdfolder)
            pcd_list = findAllPcdPath(pcdfolder)
            for pcd in pcd_list:
                # 读取pcd文件到数组中
                lidar_data = readPcd(pcd)
                # 得到pcd文件的坐标较小值
                lidar_data_new = scalePcd(lidar_data,x_scale,y_scale,z_scale)
                # 写入到目标文件中
                dst_folder = os.path.join(DST_folder,subpcdfolder)
                os.makedirs(dst_folder,exist_ok=True)
                writePcd(lidar_data_new, pcd, dst_folder)
                
                print(pcd,"DONE!!!!!!!!")
        
if __name__ == "__main__":
    fire.Fire()