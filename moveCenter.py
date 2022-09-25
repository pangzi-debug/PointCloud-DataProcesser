from cProfile import label
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

def writeLabel(label_data,SRC_path,DST_path):
    # get Name From SRC_path
    label_name = os.path.basename(SRC_path)
    
    DST_pcdPath = os.path.join(DST_path,label_name)

    with open(DST_pcdPath,'w') as F: # 重写
        for label_d in label_data:
            F.writelines(label_d)
    # ^将label写入到目标文件中

def findBottomLeft(lidarData):
    #print(lidarData)
    min_x,min_y,min_z = np.min(lidarData,axis=0)
    return min_x,min_y,min_z
def moveCenter(lidarData,labelData,min_x,min_y,min_z):
    newLidarData = lidarData
    newLidarData[:,0] = newLidarData[:,0] - min_x
    newLidarData[:,1] = newLidarData[:,1] - min_y
    newLidarData[:,2] = newLidarData[:,2] - min_z

    newLabelData = []
    for l_d in labelData:
        l_d_arr = l_d.split(" ")
        l_d_arr = l_d_arr[1:]
        l_d_arr = [float(l) for l in l_d_arr]
        # add 'car'
        #move x y z
        l_d_arr[11] = l_d_arr - min_x
        l_d_arr[12] = l_d_arr - min_y 
        l_d_arr[13] = l_d_arr - min_z
        l_d_arr = ['car'] + l_d_arr
        newLabelData.append(l_d_arr)        
    return newLidarData,newLabelData

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
def getLabelByPcd(pcd,srcLabelFolder):
    baseName,_ = os.path.splitext(pcd)
    labelName = baseName+".txt"
    labelPath = os.path.join(srcLabelFolder,labelName) # |构Label路径\

    # 一行行读出label
    if os.path.exists(labelPath):
        return [ line.rstrip().lstrip() for  line in open(labelPath).readlines()] # 传回label数组
    else:
        return None
    
def PROCESS(srcfolder,dstfolder):
    # srcfolder-> 1, pcdfolder 2, labelfolder
    # dstfolder -> 1，pcdfolder 2,labelfolder
    srcpcdfolder = os.path.join(srcfolder,"pcd")
    dstpcdfolder = os.path.join(dstfolder,"pcd")
    srclabelfolder = os.path.join(srcfolder,"label")
    dstlabelfolder = os.path.join(dstfolder,"label")

    #1,转换原始的pcd文件 同时存储进目标pcd文件夹中
    #2,转换原始的label文件 同时存储进目标label文件夹中


    # 1，找到所有的pcd文件，获得当前文件 # 
    pcd_list = findAllPcdPath(srcpcdfolder)
    for pcd in pcd_list:
        label_data = getLabelByPcd(pcd,srclabelfolder)
        if label is None:
            continue
        # 读取pcd文件到数组中
        lidar_data = readPcd(pcd)
        # 得到pcd文件的坐标较小值
        min_x,min_y,min_z = findBottomLeft(lidar_data)
        # 得到坐标移动后的pcd
        lidar_data_new,label_data_new = moveCenter(lidar_data,label_data,min_x,min_y,min_z)
        # 写入到目标文件中
        writePcd(lidar_data_new, pcd, dstpcdfolder)
        writeLabel(label_data_new,label,dstlabelfolder)
        print(pcd,"DONE!!!!!!!!")
        
if __name__ == "__main__":
    fire.Fire()