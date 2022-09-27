# 指定kitti标签路径，统计其中的长宽高平均值
import os
label_dir = "/root/files/sjwang/PointRCNN-0702/PointRCNN-master/data/KITTI/object/training/label_2"
mean_h = 0
mean_w = 0
mean_l = 0
max_h = 0
max_w = 0
max_l = 0
min_h = 1110
min_w = 1110
min_l = 1110
label_counter = 0
for root, dirs, files in os.walk(label_dir):
    for label_file  in files :
        label_file_abs_path = os.path.join(root,label_file)
        with open(label_file_abs_path,'r') as lf:
            lines = lf.readlines()
            for line in lines:
                line_arr = line.split(" ")
                h = float(line_arr[8])
                w = float(line_arr[9])
                l = float(line_arr[10])
                label_counter = label_counter + 1 
                mean_h = mean_h + h 
                mean_w = mean_w + w
                mean_l = mean_l + l 
                max_h = max(max_h,h)
                max_w = max(max_w,w)
                max_l = max(max_l,l)
                min_h = min(min_h,h)
                min_w = min(min_w,w)
                min_l = min(min_l,l)
mean_h = mean_h / label_counter 
mean_w = mean_w / label_counter 
mean_l = mean_l / label_counter
print(mean_h)
print(mean_w)
print(mean_l)

print(max_h)
print(max_w)
print(max_l)

print(min_h)
print(min_w)
print(min_l)