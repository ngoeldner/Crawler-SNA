import time
from os import listdir
from os.path import isfile, join
import numpy as np
import matplotlib.pyplot as plt
from skimage.morphology import medial_axis, skeletonize, thin
from skimage.morphology import binary_closing, disk, binary_opening
from skimage.io import imread
from skimage.color import  rgb2gray
from scipy import ndimage as ndi
import statistics

zs_time = 0
gh_time = 0
medial_time = 0
distance_time = 0
ma_list = list()
zs_list = list()
gh_list = list()
dist_list = list()

def plot(img_plot, count_time=False):
    global img_position, ax, zs_time, gh_time, medial_time, distance_time
    skel, distance = medial_axis(img_plot, return_distance=True)

    # Compare with other skeletonization algorithms
    skeleton = skeletonize(img_plot)
    thinned = thin(img_plot)

    start = time.time()
    skel, distance = medial_axis(img, return_distance=True)
    finish = time.time()
    if count_time:
        print("Medial axis time = {}".format(finish-start))
        medial_axis_time = finish-start
        ma_list.append(medial_axis_time)
        medial_time += medial_axis_time
    
    if count_time:
        start = time.time()
        distance = ndi.distance_transform_edt(img)
        finish = time.time()
        print("distance time = {}".format(finish-start))
        dist_list.append(finish-start)
        distance_time += finish-start
        

    # Compare with other skeletonization algorithms
    start = time.time()
    skeleton = skeletonize(img)
    finish = time.time()
    if count_time:
        print("Skeletonize time = {}".format(finish-start))
        skeleton_time = finish-start
        zs_list.append(skeleton_time)
        zs_time += skeleton_time

    start = time.time()
    thinned = thin(img)
    finish = time.time()
    if count_time:
        print("Thin time        = {}".format(finish-start))
        thin_time = finish-start
        gh_list.append(thin_time)
        gh_time += thin_time

    # Distance to the background for pixels of the skeleton
    dist_on_skel = distance * skel

    # ax[img_position].imshow(img_plot, cmap=plt.cm.gray)
    # ax[img_position].set_title('OG')
    # ax[img_position].axis('off')
    # img_position+=1

    # ax[img_position].imshow(dist_on_skel, cmap='magma')
    # ax[img_position].contour(img_plot, [0.5], colors='w')
    # ax[img_position].set_title('MA')
    # ax[img_position].axis('off')
    # img_position+=1


    # # ax[img_position].imshow(skeleton, cmap=plt.cm.gray)
    # ax[img_position].imshow(skeleton, cmap='magma')
    # ax[img_position].contour(img_plot, [0.5], colors='w')
    # ax[img_position].set_title('ZS')
    # ax[img_position].axis('off')
    # img_position+=1


    # ax[img_position].imshow(thinned, cmap='magma')
    # ax[img_position].contour(img_plot, [0.5], colors='w')
    # ax[img_position].set_title("GH")
    # ax[img_position].axis('off')
    # img_position+=1 



direc = "../occupancy_grid_tests_p/correct_nano_ogs/new_ogs/"
onlyfiles = [f for f in listdir(direc) if isfile(join(direc, f))]
onlyfiles.sort()
print(onlyfiles)
print(len(onlyfiles))
onlyfiles.pop()
for file in onlyfiles:
    if len(file)==20:
        img = rgb2gray(imread(direc+file))
        # img = rgb2gray(imread("/content/000000773.png"))
        print(img.shape)

        threshold = 0
        bin_img = (img > threshold) #binary is when less than trheshold

        start = time.time()
        # closed1_open = binary_opening(bin_img, np.ones((3,3)))
        closed1_open = binary_opening(bin_img, np.ones((5,5)))
        finish = time.time()
        print("binary_opening 3x3 time = {}".format(finish-start))

        start = time.time()
        # closed1 = binary_closing(closed1_open, disk(10))
        closed1 = binary_closing(closed1_open, disk(15))
        finish = time.time()
        print("binary_closing disk-10 time = {}".format(finish-start))

        # closed1_cmp = binary_closing(bin_img, disk(10))

        # closed2_open = binary_opening(bin_img, np.ones((3,3)))

        # start = time.time()
        # closed2 = binary_closing(closed2_open, np.ones((20,20)))
        # finish = time.time()
        # print("binary_closing 20x20 time = {}".format(finish-start))

        # fig, axes = plt.subplots(4, 4, figsize=(12, 12), sharex=True, sharey=True)
        # fig, axes = plt.subplots(1, 4, figsize=(12, 12), sharex=True, sharey=True)
        # ax = axes.ravel()
        img_position = 0

        # img = bin_img
        # plot(img)

        img = closed1
        plot(img, True)

        # img = closed1_cmp
        # plot(img)

        # img = closed2
        # plot(img)

        # fig.tight_layout()
        # plt.savefig('tests02/'+file)
        # break
    # plt.show()
  
print("zs_time =       {}".format(zs_time))
print("gh_time =       {}".format(gh_time))
print("medial_time =   {}".format(medial_time))
print("distance_time = {}".format(distance_time))
print("zs_time =       {}".format(zs_time/70))
print("gh_time =       {}".format(gh_time/70))
print("medial_time =   {}".format(medial_time/70))
print("distance_time = {}".format(distance_time/70))
print("gh/zs =     {}".format(gh_time/zs_time))
print("medial/zs = {}".format(medial_time/zs_time))

print("avg_zs_time =       {}".format(statistics.mean(zs_list)))
print("avg_gh_time =       {}".format(statistics.mean(gh_list)))
print("avg_medial_time =   {}".format(statistics.mean(ma_list)))
print("avg_distance_time = {}".format(statistics.mean(dist_list)))

print("stddev_zs_time =       {}".format(statistics.stdev(zs_list)))
print("stddev_gh_time =       {}".format(statistics.stdev(gh_list)))
print("stddev_medial_time =   {}".format(statistics.stdev(ma_list)))
print("stddev_distance_time = {}".format(statistics.stdev(dist_list)))
