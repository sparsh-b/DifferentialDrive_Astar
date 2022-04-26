import cv2
import numpy as np
import glob

img_array = []
imgs = glob.glob('./results/explore*jpg')
imgs.sort()
for filename in imgs:
    img = cv2.imread(filename)
    height, width, layers = img.shape
    size = (width,height)
    img_array.append(img)

fps=60
path_img = cv2.imread('./results/final_path.jpg')
for i in range(fps*2):
    img_array.append(path_img)

out = cv2.VideoWriter('./A_star_diff_drive_test2.mkv',cv2.VideoWriter_fourcc(*'DIVX'), fps, size)
 
for i in range(len(img_array)):
    out.write(img_array[i])
out.release()
