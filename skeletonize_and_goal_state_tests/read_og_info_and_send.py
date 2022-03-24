from sysconfig import get_path
import time
from collections import deque
from math import sqrt
import math
import numpy as np
import matplotlib.pyplot as plt
from skimage.morphology import skeletonize
from skimage.morphology import binary_closing, disk, binary_opening
from skimage.io import imread
from skimage.color import  rgb2gray
from scipy import ndimage as ndi

og_pixel_width = 160
og_pixel_depth = 200

checkpoint_i = 0
checkpoint_j = 0
exp_number = 0
og_number = 0
id_number = 0
total_counter = 0

send_to_file = ""
    
file = open("../occupancy_grid_tests_p/occupancy_grid_tests/lembretes_imagens.txt", "r")
while(True):
    line = file.readline()
    if "checkpoints" in line:
        break
while(True):
    line = file.readline()[:-1]
    if "OGs" in line:
        break
    if ":" in line:
        exp_number = int(line[:-1])
        print("EXP_NUMBER:")
        print(exp_number)
        continue
    if "ok" in line:
        # print(line)
        words = line.split()
        og_number = int(words[0])
        print("OG_NUMBER:")
        send_to_file+="exp_number: {:2d}\n".format(exp_number)
        send_to_file+="og_number: {:3d}\n".format(og_number)
        print(og_number)
        id_number = 0
        
        for word in words:
            if "-" in word:
                i,j = word.split("-")
                i = int(i)
                j = int(j)
                checkpoint_i = i
                checkpoint_j = j
                # print(i,j)
                print("HAS_CHECKPOINT")
                send_to_file+="has_checkpoint\n"
                send_to_file+="{} {}\n".format(checkpoint_i, checkpoint_j)
                id_number+=1
                total_counter+=1
                continue
            if "S" in word:
                print("DO_NOT_HAVE_CHECKPOINT")
                send_to_file+="does_not_have_checkpoint\n"
                id_number+=1
                total_counter+=1
file.close()

file = open("checkpoint_file_list/list.txt", "w")
send_to_file+="$\n"
file.write(send_to_file)
file.close()