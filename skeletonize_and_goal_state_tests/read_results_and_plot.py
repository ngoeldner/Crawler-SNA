import readline
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
from os import listdir
from os.path import isfile, join


def plot(results_file):
    og_pixel_depth = 200
    og_pixel_width = 160    
    # read img and morphology
    file = open(results_file)
    
    file.readline()
    og_without_morph = np.zeros((og_pixel_depth, og_pixel_width), dtype=np.int32)
    for i in range(og_pixel_depth):
        og_without_morph[i] = np.array(list(map(float, file.readline().split())), dtype=np.int32)
    
    file.readline()
    distance = np.zeros((og_pixel_depth, og_pixel_width), dtype=np.float64)
    for i in range(og_pixel_depth):
        distance[i] = np.array(file.readline().split(), dtype=np.float64)
    
    og = np.zeros((og_pixel_depth, og_pixel_width), dtype=np.int32)
    for i in range(og_pixel_depth):
        for j in range(og_pixel_width):
            if(distance[i][j] > 0.2):
                og[i][j] = 1
    
    print(file.readline())
    skeleton = np.zeros((og_pixel_depth, og_pixel_width), dtype=np.int32)
    for i in range(og_pixel_depth):
        skeleton[i] = np.array(file.readline().split(), dtype=np.int32)
    
    dist_on_skel = distance*skeleton
    
    print(file.readline())
    skeleton_rem = list()
    len_skeleton_rem = int(file.readline())
    # print(len_skeleton_rem)
    for i in range(len_skeleton_rem):
        skeleton_rem.append(list(map(int, file.readline().split())))

    print(file.readline())
    start_point = list(map(int, file.readline().split()))
    
    print(file.readline())
    # print(file.readline().split())
    has_checkpoint = int(file.readline())
    
    print(file.readline())
    checkpoint = list(map(int, file.readline().split()))
    
    print(file.readline())
    final_point_path = list(map(int, file.readline().split()))
    
    print(file.readline())
    len_path = int(file.readline())
    path = list()
    for i in range(len_path):
        path.append(list(map(int, file.readline().split())))
        
    print(file.readline())
    len_extended_path = int(file.readline())
    extended_path = list()
    for i in range(len_extended_path):
        extended_path.append(list(map(int, file.readline().split())))
        
    print(file.readline())
    line = file.readline().split()
    goal_state_pos = [int(line[0]), int(line[1])]
    goal_state_orientation = float(line[2])
    
    file.close()
    

    fig, axes = plt.subplots(1, 5, figsize=(15, 8), sharex=True, sharey=True)
    ax = axes.ravel()
    img_position = 0
    
    ax[img_position].imshow(og_without_morph)
    # ax[img_position].contour(og, [0.5], colors='w')
    ax[img_position].set_title('(1)')
    ax[img_position].axis('off')
    img_position+=1
    
    ax[img_position].imshow(dist_on_skel, cmap='magma')
    ax[img_position].contour(og, [0.5], colors='w')
    ax[img_position].set_title('(1)')
    ax[img_position].axis('off')
    img_position+=1
    
    dist_on_skel_rem = dist_on_skel.copy()
    for i in range(og_pixel_depth):
        for j in range(og_pixel_width):
            if dist_on_skel_rem[i][j] > 0.2:
                erase = True
                for pixel in skeleton_rem:
                    if pixel[0] == i and pixel[1] == j:
                        erase = False
                if erase:
                    dist_on_skel_rem[i][j] = 0.0                    
                
    
    ax[img_position].imshow(dist_on_skel_rem, cmap='magma')
    ax[img_position].contour(og, [0.5], colors='w')
    ax[img_position].set_title('(2)')
    ax[img_position].scatter(start_point[1], start_point[0], s=20, c='green', marker='o')
    if has_checkpoint:
        ax[img_position].scatter(checkpoint[1], checkpoint[0], s=20, c='red', marker='o')
    ax[img_position].axis('off')
    img_position+=1
    
    dist_on_skel_bfs = np.zeros((og_pixel_depth, og_pixel_width))
    for pixel in path:
        dist_on_skel_bfs[pixel[0]][pixel[1]] = dist_on_skel[pixel[0]][pixel[1]]
    ax[img_position].imshow(dist_on_skel_bfs, cmap='magma')
    ax[img_position].contour(og, [0.5], colors='w')
    ax[img_position].set_title('(3) ')
    ax[img_position].scatter(start_point[1], start_point[0], s=20, c='green', marker='o')
    if has_checkpoint:
        ax[img_position].scatter(final_point_path[1], final_point_path[0], s=20, c='blue', marker='o')
        ax[img_position].scatter(checkpoint[1], checkpoint[0], s=20, c='red', marker='o')
    else:
        ax[img_position].scatter(goal_state_pos[1], goal_state_pos[0], s=20, c='yellow', marker='o')
    ax[img_position].axis('off')
    img_position+=1
    
    dist_on_skel_xpath = np.zeros((og_pixel_depth, og_pixel_width))
    for pixel in extended_path:
        dist_on_skel_xpath[pixel[0]][pixel[1]] = distance[pixel[0]][pixel[1]]
    ax[img_position].imshow(dist_on_skel_xpath, cmap='magma')
    ax[img_position].contour(og, [0.5], colors='w')
    ax[img_position].set_title('(4)')
    ax[img_position].scatter(start_point[1], start_point[0], s=20, c='green', marker='o')
    if has_checkpoint:
        ax[img_position].scatter(final_point_path[1], final_point_path[0], s=20, c='blue', marker='o')
        ax[img_position].scatter(checkpoint[1], checkpoint[0], s=20, c='red', marker='o')
    ax[img_position].scatter(goal_state_pos[1], goal_state_pos[0], s=20, c='yellow', marker='o')   
    orientation_x_red = math.cos(goal_state_orientation)*5
    orientation_z_red = math.sin(goal_state_orientation)*5
    ax[img_position].arrow(goal_state_pos[1], goal_state_pos[0], \
            orientation_x_red, -orientation_z_red, width=1.5, color="yellow")
        
    # if has_checkpoint:
    #     ax[img_position].arrow(extended_min_dist_checkpoint_pixel[1], extended_min_dist_checkpoint_pixel[0], \
    #         orientation_x_red, -orientation_z_red, width=1.5, color="yellow")
    # else:
    #     ax[img_position].arrow(max_dist_crawler_pixel[1], max_dist_crawler_pixel[0], \
    #         orientation_x_red, -orientation_z_red, width=1.5, color="yellow")
    ax[img_position].axis('off')
    img_position+=1

    fig.tight_layout()
    plt.savefig('visualize_cpp_results/{}.png'.format(results_file[12:-4]))
    # plt.show()

dir = "cpp_results/"
onlyfiles = [f for f in listdir(dir) if isfile(join(dir, f))]
onlyfiles.sort()
print(onlyfiles)
for file in onlyfiles:
    plot(dir+file)


    