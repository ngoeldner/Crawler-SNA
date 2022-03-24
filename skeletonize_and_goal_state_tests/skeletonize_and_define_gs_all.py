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

zs_time = 0
opening_time = 0
closing_time = 0
distance_time = 0
pixel_dist_list_time = 0
connections_time = 0
rem_close_contour_time = 0
bfs_time = 0
get_min_max_dist_time = 0
get_path_time = 0
extend_time = 0
total_time = 0
og_pixel_width = 160
og_pixel_depth = 200

checkpoint_i = 0
checkpoint_j = 0
exp_number = 0
og_number = 0
id_number = 0
total_counter = 0

# class pixel_dist:
#     def __init__(self, pos_i, pos_j, dist_countor):
#         self.pos_i = pos_i
#         self.pos_j = pos_j
#         self.dist2contour  = dist_countor
        
# (0,0) is the top left of the top left pixel 
def calculate_dist2crawler(pos_i, pos_j):
    return ((og_pixel_width/2) - pos_j)**2  +  (og_pixel_depth - pos_i)**2

def calculate_dist2checkpoint(pos_i, pos_j):
    global checkpoint_j, checkpoint_i
    return (checkpoint_j - pos_j)**2  +  (checkpoint_i - pos_i)**2

def plot(has_checkpoint, count_time=False):
    global zs_time, opening_time, closing_time, distance_time, total_time, connections_time, rem_close_contour_time, bfs_time, get_min_max_dist_time, get_path_time, extend_time, pixel_dist_list_time
    
    # read img and morphology
    img = rgb2gray(imread("../occupancy_grid_tests_p/correct_nano_ogs/new_ogs/og_output_{:02d}_{:03d}.png".format(exp_number, og_number)))
    print(img.shape)

    threshold = 0
    bin_img = (img > threshold) #binary is when less than trheshold

    start = time.time()
    closed1_open = binary_opening(bin_img, np.ones((5,5)))
    finish = time.time()
    if count_time:
        print("binary_opening 3x3 time = {}".format(finish-start))
        opening_time += finish - start
        total_time += finish - start

    start = time.time()
    closed1 = binary_closing(closed1_open, disk(15))
    finish = time.time()
    if count_time:
        print("binary_closing disk-10 time = {}".format(finish-start))
        closing_time += finish - start
        total_time += finish - start

    fig, axes = plt.subplots(1, 4, figsize=(15, 8), sharex=True, sharey=True)
    ax = axes.ravel()
    img_position = 0

    img = closed1

    img_plot = img
    
    start = time.time()
    distance = ndi.distance_transform_edt(img)
    finish = time.time()
    if count_time:
        print("distance time = {}".format(finish-start))
        distance_time += finish-start
        total_time += finish-start
        

    # Compare with other skeletonization algorithms
    start = time.time()
    skeleton = skeletonize(img)
    finish = time.time()
    if count_time:
        print("Skeletonize time = {}".format(finish-start))
        skeleton_time = finish-start
        zs_time += skeleton_time
        total_time += skeleton_time
    
    # Distance to the background for pixels of the skeleton
    dist_on_skel = distance * skeleton
    
    connections = dict()
    pixel_dist_list = list()
    skel_dist_list = list()
    
    start = time.time()
    for i in range(og_pixel_depth):
        for j in range(og_pixel_width):
            connections[(i,j)] = list()
            if skeleton[i][j]:
                # pixel = pixel_dist(i,j,dist_on_skel[i][j])
                pixel = (i,j)
                skel_dist_list.append(pixel)
                if dist_on_skel[i][j] >= 10:
                    pixel_dist_list.append(pixel)
    finish = time.time()
    if count_time:
        print("pixel_dist_list time = {}".format(finish-start))
        pixel_dist_list_time += finish-start
        total_time += finish-start
            
    start = time.time()
    for i in range(og_pixel_depth):
        for j in range(og_pixel_width):
            if skeleton[i][j] and dist_on_skel[i][j] >= 10:
                # pixel_dist_list.append(pixel_dist(i,j,dist_on_skel[i][j]))
                if i!=199:
                    if skeleton[i+1][j] and dist_on_skel[i+1][j] >= 10:
                        connections[(i,j)].append((i+1,j))
                        connections[(i+1,j)].append((i,j))
                if j!=199:
                    if skeleton[i][j+1] and dist_on_skel[i][j+1] >= 10:
                        connections[(i,j)].append((i,j+1))
                        connections[(i,j+1)].append((i,j))
                if i!=199 and j!=199:
                    if skeleton[i+1][j+1] and dist_on_skel[i+1][j+1] >= 10:
                        connections[(i,j)].append((i+1,j+1))
                        connections[(i+1,j+1)].append((i,j))
                if i!=199 and j!=0:
                    if skeleton[i+1][j-1] and dist_on_skel[i+1][j-1] >= 10:
                        connections[(i,j)].append((i+1,j-1))
                        connections[(i+1,j-1)].append((i,j))
    finish = time.time()
    if count_time:
        print("connections time = {}".format(finish-start))
        connections_time += finish-start
        total_time += finish-start
    
    # pixel_dist_list_rem = list()
    
    # remove pixels close to contour and find start_point for bfs
    start = time.time()
    min_dist_crawler = 9999999
    min_dist_crawler_pixel = (0,0)
    for pixel in pixel_dist_list:
        dist2crawler = calculate_dist2crawler(pixel[0], pixel[1])
        if dist2crawler < min_dist_crawler:
            min_dist_crawler = dist2crawler
            min_dist_crawler_pixel = (pixel[0], pixel[1])
    finish = time.time()
    if count_time:
        print("rem_close_contour time = {}".format(finish-start))
        rem_close_contour_time += finish-start
        total_time += finish-start
    
    print("min_dist_crawler_pixel:")
    print(min_dist_crawler)
    print(min_dist_crawler_pixel)
        
    # BFS
    start = time.time()
    parent = dict()
    queue = deque()
    visited = dict()
    distance_bfs = dict()
    for pixel in pixel_dist_list:
        visited[(pixel[0], pixel[1])] = False
        parent[(pixel[0], pixel[1])] = (-1, -1)
        distance_bfs[(pixel[0], pixel[1])] = 9999999
    
    distance_bfs[min_dist_crawler_pixel] = 0
    visited[min_dist_crawler_pixel] = True
    queue.append(min_dist_crawler_pixel)
    while len(queue) != 0:
        u = queue.popleft()
        for v in connections[u]:
            if not visited[v]:
                visited[v] = True
                queue.append(v)
                parent[v] = u
                distance_bfs[v] = distance_bfs[u] + 1
    
    finish = time.time()
    if count_time:
        print("connections time = {}".format(finish-start))
        bfs_time += finish-start
        total_time += finish-start
    
    
    # get min_dist_checkpoint_pixel
    start = time.time()
    if has_checkpoint:
        min_dist_checkpoint = 9999999
        min_dist_checkpoint_pixel = (0,0)
        for pixel in pixel_dist_list:
            if visited[pixel]:
                dist2checkpoint = calculate_dist2checkpoint(pixel[0], pixel[1])
                if dist2checkpoint < min_dist_checkpoint:
                    min_dist_checkpoint = dist2checkpoint
                    min_dist_checkpoint_pixel = (pixel[0], pixel[1])
        
        print("min_dist_checkpoint_pixel:")
        print(min_dist_checkpoint)
        print(min_dist_checkpoint_pixel)
    
    # get max_dist_crawler_pixel
    else:
        sorted_pixel_dist_list = sorted(pixel_dist_list, key=lambda x: dist_on_skel[x[0]][x[1]], reverse=False)
        # print("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSsss")
        # for pixel in sorted_pixel_dist_list:
            # print(pixel)
            # print(dist_on_skel[pixel[0]][pixel[1]])
        max_dist_crawler = -1
        max_dist_crawler_pixel = (0,0)
        start_index = int(len(sorted_pixel_dist_list)*0.5) - 1
        last_index = len(sorted_pixel_dist_list)
        for index in range(start_index, last_index):
            pixel = sorted_pixel_dist_list[index]
            if visited[pixel]:
                dist2crawler = calculate_dist2crawler(pixel[0], pixel[1])
                if dist2crawler > max_dist_crawler:
                    max_dist_crawler = dist2crawler
                    max_dist_crawler_pixel = (pixel[0], pixel[1])
        
        print("max_dist_crawler_pixel:")
        print(max_dist_crawler)
        print(max_dist_crawler_pixel)
    finish = time.time()
    if count_time:
        print("get_min_max_dist time = {}".format(finish-start))
        get_min_max_dist_time += finish-start
        total_time += finish-start
    
    # get path
    start = time.time()
    path = list()
    if has_checkpoint:
        path.append(min_dist_checkpoint_pixel)
        cur_parent = parent[min_dist_checkpoint_pixel]
    else:
        path.append(max_dist_crawler_pixel)
        cur_parent = parent[max_dist_crawler_pixel]
    while cur_parent != (-1,-1):
        path.append(cur_parent)
        cur_parent = parent[cur_parent]
    path.reverse()
    finish = time.time()
    if count_time:
        print("get_path time = {}".format(finish-start))
        get_path_time += finish-start
        total_time += finish-start
    print("path:")
    print(len(path))
    print(path)
    
    # extend path
    if has_checkpoint:
        start = time.time()
        extended_path = path.copy()
        it_min_dist_checkpoint_pixel = min_dist_checkpoint_pixel
        neighbors = [(-1, -1), (-1, 0), (-1, +1), (0, +1) , (+1, +1), (+1, 0), (+1, -1), (0, -1)]
        while True:
            min_distance = calculate_dist2checkpoint(it_min_dist_checkpoint_pixel[0], it_min_dist_checkpoint_pixel[1])
            sub_it_min_dist_checkpoint_pixel = it_min_dist_checkpoint_pixel
            for n in neighbors:
                if distance[it_min_dist_checkpoint_pixel[0]+n[0]][it_min_dist_checkpoint_pixel[1]+n[1]] >= 10:
                    cur_distance = calculate_dist2checkpoint(it_min_dist_checkpoint_pixel[0]+n[0], it_min_dist_checkpoint_pixel[1]+n[1])
                    if cur_distance < min_distance:
                        min_distance = cur_distance
                        sub_it_min_dist_checkpoint_pixel = (it_min_dist_checkpoint_pixel[0]+n[0], it_min_dist_checkpoint_pixel[1]+n[1])
            if sub_it_min_dist_checkpoint_pixel == it_min_dist_checkpoint_pixel:
                break
            else:
                extended_path.append(sub_it_min_dist_checkpoint_pixel)
                it_min_dist_checkpoint_pixel = sub_it_min_dist_checkpoint_pixel
        finish = time.time()
        if count_time:
            print("extend time = {}".format(finish-start))
            extend_time += finish-start
            total_time += finish-start
        
        print("extended_path:")   
        print(len(extended_path)) 
        print(extended_path)
    
        extended_min_dist_checkpoint_pixel = extended_path[-1]
    
    else:
        extended_path = path.copy()
        
        
    # get orientation
    # orientation_z = 0
    # orientation_x = 0
    print("kkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk")
    print(extended_path[int(len(extended_path)*0.75 - 1)])
    print(extended_path[len(extended_path)-1])
    # orientation_z = extended_path[0][0] - extended_path[len(extended_path)-1][0]
    # orientation_x = extended_path[len(extended_path)-1][1] - extended_path[0][1]
    orientation_z = extended_path[int(len(extended_path)*0.75 - 1)][0] - extended_path[len(extended_path)-1][0]
    orientation_x = extended_path[len(extended_path)-1][1] - extended_path[int(len(extended_path)*0.75 - 1)][1]
    # for i in range(len(extended_path)-1):
    #     orientation_z += extended_path[i][0] - extended_path[i+1][0]
    #     orientation_x += extended_path[i+1][1] - extended_path[i][1]
    print("orientation:")
    print(orientation_z)
    print(orientation_x)
    

    # ax[img_position].imshow(img_plot, cmap=plt.cm.gray)
    # ax[img_position].set_title('original')
    # ax[img_position].axis('off')
    # img_position+=1

    ax[img_position].imshow(dist_on_skel, cmap='magma')
    ax[img_position].contour(img_plot, [0.5], colors='w')
    ax[img_position].set_title('(1)')
    ax[img_position].axis('off')
    img_position+=1
    
    dist_on_skel_rem = dist_on_skel.copy()
    for pixel in skel_dist_list:
        if pixel not in pixel_dist_list:
            dist_on_skel_rem[pixel[0]][pixel[1]] = 0.0
    
    ax[img_position].imshow(dist_on_skel_rem, cmap='magma')
    ax[img_position].contour(img_plot, [0.5], colors='w')
    ax[img_position].set_title('(2)')
    ax[img_position].scatter(min_dist_crawler_pixel[1], min_dist_crawler_pixel[0], s=20, c='green', marker='o')
    if has_checkpoint:
        ax[img_position].scatter(checkpoint_j, checkpoint_i, s=20, c='red', marker='o')
    ax[img_position].axis('off')
    img_position+=1
    
    dist_on_skel_bfs = np.zeros((og_pixel_depth, og_pixel_width))
    for pixel in path:
        dist_on_skel_bfs[pixel[0]][pixel[1]] = dist_on_skel[pixel[0]][pixel[1]]
    ax[img_position].imshow(dist_on_skel_bfs, cmap='magma')
    ax[img_position].contour(img_plot, [0.5], colors='w')
    ax[img_position].set_title('(3) ')
    ax[img_position].scatter(min_dist_crawler_pixel[1], min_dist_crawler_pixel[0], s=20, c='green', marker='o')
    if has_checkpoint:
        ax[img_position].scatter(min_dist_checkpoint_pixel[1], min_dist_checkpoint_pixel[0], s=20, c='blue', marker='o')
        ax[img_position].scatter(checkpoint_j, checkpoint_i, s=20, c='red', marker='o')
    else:
        ax[img_position].scatter(max_dist_crawler_pixel[1], max_dist_crawler_pixel[0], s=20, c='yellow', marker='o')
    ax[img_position].axis('off')
    img_position+=1
    
    dist_on_skel_xpath = np.zeros((og_pixel_depth, og_pixel_width))
    for pixel in extended_path:
        dist_on_skel_xpath[pixel[0]][pixel[1]] = distance[pixel[0]][pixel[1]]
    ax[img_position].imshow(dist_on_skel_xpath, cmap='magma')
    ax[img_position].contour(img_plot, [0.5], colors='w')
    ax[img_position].set_title('(4)')
    ax[img_position].scatter(min_dist_crawler_pixel[1], min_dist_crawler_pixel[0], s=20, c='green', marker='o')
    if has_checkpoint:
        ax[img_position].scatter(min_dist_checkpoint_pixel[1], min_dist_checkpoint_pixel[0], s=20, c='blue', marker='o')
        ax[img_position].scatter(checkpoint_j, checkpoint_i, s=20, c='red', marker='o')
        ax[img_position].scatter(extended_min_dist_checkpoint_pixel[1], extended_min_dist_checkpoint_pixel[0], s=20, c='yellow', marker='o')   
    else:
         ax[img_position].scatter(max_dist_crawler_pixel[1], max_dist_crawler_pixel[0], s=20, c='yellow', marker='o')
    # orientation_length = 10 = sqrt(orientation_z**2 + orientation_x**2)
    # orientation_x/orientation_z = C
    # orientation_x = C*orientation_z
    # 10 = sqrt(orientation_z**2 + (C*orientation_z)**2)
    # 100 = orientation_z**2 + (C*orientation_z)**2
    # 100 = orientation_z**2 + orientation_z**2 * C**2
    # 100 = (1 + C**2)*orientation_z**2
    # 100/(1 + C**2) = orientation_z**2
    # sqrt(100/(1 + C**2)) = orientation_z
    if orientation_z!=0:
        orientation_prop = orientation_x/orientation_z
        orientation_z_red = sqrt(81/(1 + orientation_prop**2))
        orientation_x_red = orientation_z_red * orientation_prop
    else:
        orientation_z_red = 0
        orientation_x_red = 9
    # ax[img_position].arrow(extended_min_dist_checkpoint_pixel[1], extended_min_dist_checkpoint_pixel[0],
    #  orientation_x, -orientation_z, width=1.5)
    print("orientation red:")
    print(orientation_x_red)
    print(orientation_z_red)
    orientation_z_red = abs(orientation_z_red)
    orientation_x_red = abs(orientation_x_red)
    if orientation_z <= 0:
        orientation_z_red *=-1
    if orientation_x <= 0:
        orientation_x_red *=-1
        
    if has_checkpoint:
        ax[img_position].arrow(extended_min_dist_checkpoint_pixel[1], extended_min_dist_checkpoint_pixel[0], \
            orientation_x_red, -orientation_z_red, width=1.5, color="yellow")
    else:
        ax[img_position].arrow(max_dist_crawler_pixel[1], max_dist_crawler_pixel[0], \
            orientation_x_red, -orientation_z_red, width=1.5, color="yellow")
    ax[img_position].axis('off')
    img_position+=1
    
    # to file
    file = open("to_mp/{:02}_{:03d}_{:02}_info.txt".format(exp_number, og_number, id_number), "w")
    # og
    file.write("width: {} depth: {}\n".format(og_pixel_width, og_pixel_depth))
    # print(len(img_plot))
    # print(img_plot.shape)
    for i in range(og_pixel_depth):
        for j in range(og_pixel_width):
            if img_plot[i, j]:
                file.write("{} ".format(1))
            else:
                file.write("{} ".format(0))
        file.write("\n")
    # skeleton path
    file.write("path: {}\n".format(len(extended_path)))
    for i in range(len(extended_path)):
        file.write("{}\n".format(extended_path[i]))
    # goal state
    # orientaion_angle = math.atan2(orientation_z, orientation_x)*180/math.pi #(y,x)
    # if orientaion_angle < 0:
    #     orientaion_angle+=360
    orientaion_angle_rad = math.atan2(orientation_z, orientation_x)
    if has_checkpoint:
        file.write("goal state: {} {}\n".format(extended_min_dist_checkpoint_pixel, orientaion_angle_rad))
    else:
        file.write("goal state: {} {}\n".format(max_dist_crawler_pixel, orientaion_angle_rad))
    
    file.close()

    fig.tight_layout()
    plt.savefig('skeleton_to_goal_state/{:02}_{:03d}_{:02}.png'.format(exp_number, og_number, id_number))
    # plt.show()
    
    
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
                plot(True, True)
                id_number+=1
                total_counter+=1
                continue
            if "S" in word:
                print("DO_NOT_HAVE_CHECKPOINT")
                plot(False, True)
                id_number+=1
                total_counter+=1
                

  
print("zs_time =       {}".format(zs_time))
print("distance_time = {}".format(distance_time))
print("opening_time = {}".format(opening_time))
print("closing_time = {}".format(closing_time))
print("pixel_dist_list_time = {}".format(pixel_dist_list_time))
print("connections_time = {}".format(connections_time))
print("rem_close_contour_time = {}".format(rem_close_contour_time))
print("bfs_time = {}".format(bfs_time))
print("get_min_max_dist_time = {}".format(get_min_max_dist_time))
print("get_path_time = {}".format(get_path_time))
print("extend_time = {}".format(extend_time))
print("total_time = {}".format(total_time))

print("total_counter = {}".format(total_counter))
print("zs_time =       {}".format(zs_time/total_counter))
print("distance_time = {}".format(distance_time/total_counter))
print("opening_time = {}".format(opening_time/total_counter))
print("closing_time = {}".format(closing_time/total_counter))
print("pixel_dist_list_time = {}".format(pixel_dist_list_time/total_counter))
print("connections_time = {}".format(connections_time/total_counter))
print("rem_close_contour_time = {}".format(rem_close_contour_time/total_counter))
print("bfs_time = {}".format(bfs_time/total_counter))
print("get_min_max_dist_time = {}".format(get_min_max_dist_time/total_counter))
print("get_path_time = {}".format(get_path_time/total_counter))
print("extend_time = {}".format(extend_time/total_counter))
print("total_time = {}".format(total_time/total_counter))


