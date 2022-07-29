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

def writePcd0(lidarData,PcdPath):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(lidarData)
    o3d.io.write_point_cloud(PcdPath,pcd,write_ascii=True)
def writePcd(lidarData,SRC_path,DST_path):
    # 从SRC_path得到pcd名字
    pcd_name = os.path.basename(SRC_path)
    # construct new pcd fullpath
    DST_pcdPath = os.path.join(DST_path,pcd_name)
    # 将pcd列表格式通过o3d库写入指定路径文件中 http://www.open3d.org/docs/0.9.0/tutorial/Basic/working_with_numpy.html
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(lidarData)
    o3d.io.write_point_cloud(DST_pcdPath,pcd,write_ascii=True)

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
def CropPcdSingle(lidarData,x_size,y_size,z_size,x_center,y_center,z_center):
    # 将pcd文件中切出在包围盒中的一部分
    x_limit_low = x_center - x_size /2.0
    x_limit_high = x_center + x_size/2.0
    y_limit_low = y_center - y_size / 2.0
    y_limit_high = y_center + y_size / 2.0
    z_limit_low = z_center - z_size/2.0
    z_limit_high = z_center + z_size/2.0
    x_mask = np.logical_and(lidarData[:,0] > x_limit_low, lidarData[:,0] < x_limit_high)
    y_mask = np.logical_and(lidarData[:,1] > y_limit_low, lidarData[:,1] < y_limit_high)
    z_mask = np.logical_and(lidarData[:,2] > z_limit_low, lidarData[:,2] < z_limit_high)
    mask = np.logical_and(x_mask,y_mask)
    mask = np.logical_and(mask, z_mask)
    lidarData_InBOX = lidarData[mask]
    #print(lidarData_InBOX)
    return lidarData_InBOX


def CropPcdRepeat(lidarData,
                    x_size,
                    y_size,
                    z_size,
                    x_src,
                    y_src,
                    z_src,
                    x_stepsize,
                    y_stepsize,
                    z_stepsize,
                    x_repeatTotal,
                    y_repeatTotal,
                    z_repeatTotal):
    # param:
    # x_size 表示箱子的大小 z_size y_size含义同
    # x_src,y_src,z_src表示箱子开始的出发点坐标，具体是箱子的左下角的点
    # x_stepsize 表示在x轴方向迈出的步长，其它两个含义同
    # x_repeatTotal,z_repeatTotal,y_repeatTotal分别表示在三个轴方向的重复的次数，可以理解成箱子在路上
    x_src_old = x_src
    y_src_old = y_src 
    z_src_old = z_src 
    lidardata_list = [] # 每个箱子占位一个
    for i in range(x_repeatTotal):
        x_src = x_src_old + i * x_stepsize
        #y_src = y_src_old
        #z_src = z_src_old
        for j in range(y_repeatTotal):
            y_src = y_src_old + y_stepsize * j
            #z_src = z_src_old
            for k in range(z_repeatTotal):
                z_src = z_src_old * k
                # 得到当前位置的箱子
                #   得到箱子左下角，然后转到箱子中心
                x_center = x_src + x_size / 2.0
                y_center = y_src + y_size / 2.0
                z_center = z_src + z_size / 2.0

                # crop use current box, and return lidardata
                Cropped_LidarData = CropPcdSingle(lidarData,x_size,y_size,z_size,x_center,y_center,z_center)
                lidardata_list.append(Cropped_LidarData)
    return lidardata_list


def writeLidarDataList(pcdFolder,lidarData_List):
    # 将切割后lidar列表转到文件系统中，写入按照、同时序号
    pcd_name_counter = 1
    for lidarData in lidarData_List:
        pcd_name = str(pcd_name_counter )+".pcd"
        pcd_path = os.path.join(pcdFolder,pcd_name)
        writePcd0(lidarData,pcd_path)
        pcd_name_counter = pcd_name_counter + 1


def PROCESS(SRC_FOLDER,DST_FOLDER,x_size,y_size,z_size,x_src,y_src,z_src,x_repeatTotal,y_repeatTotal,z_repeatTotal,x_stepsize,y_stepsize,z_stepsize):
    # 读取
    pcd_list = findAllPcdPath(SRC_FOLDER)
    for pcd in pcd_list:
        # 处理当前的PCD
        lidar_data = readPcd(pcd)
        lidarData_List = CropPcdRepeat(lidar_data,x_size,y_size,z_size,x_src,y_src,z_src,x_stepsize,y_stepsize,z_stepsize,x_repeatTotal,y_repeatTotal,z_repeatTotal)
        # 构建PCD目标地址
        DST_PCDFOLDER = os.path.join(DST_FOLDER , str(os.path.basename(pcd)).replace(".pcd",""))
        os.makedirs(DST_PCDFOLDER,exist_ok=True)
        writeLidarDataList(DST_PCDFOLDER,lidarData_List)

        print(pcd," is Done !!!!!!!!!")


# python CustomizedBoxRepeatCrop.py PROCESS SRC_FOLDER DST_FOLDER 190  190 100 0 0 0 3 8 1 95 95 0
if __name__ == "__main__":
    fire.Fire()