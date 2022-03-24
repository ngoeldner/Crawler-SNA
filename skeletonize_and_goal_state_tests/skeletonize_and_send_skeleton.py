from asyncore import write
import time
import numpy as np
from skimage.morphology import skeletonize
from skimage.morphology import binary_closing, disk, binary_opening
from skimage.io import imread
from skimage.color import  rgb2gray
from scipy import ndimage as ndi

zs_time = 0
opening_time = 0
closing_time = 0
distance_time = 0
send_time = 0
total_time = 0
og_pixel_width = 160
og_pixel_depth = 200

checkpoint_i = 0
checkpoint_j = 0
exp_number = 0
og_number = 0
id_number = 0
total_counter = 0

files = []


def plot(has_checkpoint, count_time=False):
    global zs_time, opening_time, closing_time, distance_time, send_time, total_time, files
    
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
    
    # # Distance to the background for pixels of the skeleton
    # dist_on_skel = distance * skeleton
    
    
    files.append("{:02}_{:03d}_{:02}_info.txt".format(exp_number, og_number, id_number))
    # to file
    start = time.time()
    file = open("to_c/{:02}_{:03d}_{:02}_info.txt".format(exp_number, og_number, id_number), "w")
    # og
    file.write("width: {} depth: {}\n".format(og_pixel_width, og_pixel_depth))
    # print(len(img_plot))
    # print(img_plot.shape)
    # print(type(img_plot))
    # print(type(distance))
    # print(type(skeleton))
    file.write("Distance:\n")
    # write_to_file = np.array2string(distance)
    # write_to_file = 0
    # np.savetxt('test.out', distance, delimiter=',')
    for i in range(og_pixel_depth):
        for j in range(og_pixel_width):
            # write_to_file += "{} ".format(distance[i, j])
            # write_to_file += distance[i, j]
            # pass
            file.write("{} ".format(distance[i, j]))
        # write_to_file += "\n"
        file.write("\n")
    # file.write(write_to_file)
    file.write("Skeleton:\n")
    skeleton_string = ""
    skeleton_counter = 0
    for i in range(og_pixel_depth):
        for j in range(og_pixel_width):
            if skeleton[i, j]:
                skeleton_string += "{} {}\n".format(i, j)
                skeleton_counter += 1
    file.write("{}\n".format(skeleton_counter))
    file.write(skeleton_string)
    if has_checkpoint:
        file.write("has_checkpoint = {}\n".format(1))
        file.write("checkpoint_i = {}\n".format(checkpoint_i))
        file.write("checkpoint_j = {}\n".format(checkpoint_j))
    else:
        file.write("has_checkpoint = {}\n".format(0))
    file.close()
    finish = time.time()
    if count_time:
        print("Send time = {}".format(finish-start))
        send_time += finish - start
        total_time += finish - start
    
    

    
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
                
file = open("to_c/names.txt", "w")
for file_name in files:
    file.write(file_name)
    file.write("\n")
file.write("$")
file.close()
  
print("zs_time =       {}".format(zs_time))
print("distance_time = {}".format(distance_time))
print("opening_time = {}".format(opening_time))
print("closing_time = {}".format(closing_time))
print("send_time = {}".format(send_time))
print("total_time = {}".format(total_time))

print("total_counter = {}".format(total_counter))
print("zs_time =       {}".format(zs_time/total_counter))
print("distance_time = {}".format(distance_time/total_counter))
print("opening_time = {}".format(opening_time/total_counter))
print("closing_time = {}".format(closing_time/total_counter))
print("send_time = {}".format(send_time/total_counter))
print("total_time = {}".format(total_time/total_counter))


