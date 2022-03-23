from os import listdir
from os.path import isfile, join
import cv2
import numpy as np
from math import cos, sin, tan, pi


OCC_WIDTH              = 800
OCC_HEIGHT             = 1000
OCC_CELL_SIZE          = 5   
CAM_DIST               = 50

# CRAWLER_WIDTH          = 50
# CRAWLER_LENGTH         = 50
# CRAWLER_CAM_DIST_FRONT = 10

# control = False
control = True
# with_goal_state = False
with_goal_state = True
crawler_L = 40.0

yaw, deslocamento, rot, steps, x, z = [0]*6

def get_grid_pos_x(x):
    return int((x + (OCC_WIDTH/2.))/OCC_CELL_SIZE)

# def get_grid_pos_z(z):
#     return int((z - 187.5)/OCC_CELL_SIZE)
def get_grid_pos_z(z):
    return OCC_HEIGHT//OCC_CELL_SIZE + CAM_DIST//OCC_CELL_SIZE - 1 - int((z+CAM_DIST)/OCC_CELL_SIZE)

def propagate():
    global yaw, deslocamento, rot, steps, x, z, pxs, pzs
    for _ in range(1, steps):
        
        # print("step")
        # print(deslocamento)
        # print(rot)
        # print(yaw)
        # print(x)
        # print(z)
        x += deslocamento*cos(yaw)
        z += deslocamento*sin(yaw)
        yaw += (deslocamento/crawler_L)*tan(rot)
        if yaw >= 2*pi:
            yaw -= 2*pi
        elif yaw < -2*pi:
            yaw += 2*pi
        px = get_grid_pos_x(x)
        pz = get_grid_pos_z(z)
        pxs.append(px)
        pzs.append(pz)


onlyfiles = [f for f in listdir('solution_paths/') if isfile(join('solution_paths/', f))]
onlyfiles.sort()
# print(onlyfiles)
for file in onlyfiles:
    pxs = list()
    pzs = list()

    with open('solution_paths/'+file) as f:
        print(file)
        h, w = map(int, f.readline().split())
        if with_goal_state:
            goal_state  = list(map(float, f.readline().split()))
        grid = []
        for i in range(h):
            grid.append(list(map(int, f.readline().split())))
        
        _, _, _, p, _ = f.readline().split()
        p = int(p)
        for i in range(p):
            f.readline()
            string = f.readline()
            # print(string)
            # print(string[17:-2])
            x, z = list(map(float, string[17:-2].split()))
            # print(x,z)
            px = get_grid_pos_x(x)
            pz = get_grid_pos_z(z)
            # print(px,pz)
            pxs.append(px)
            pzs.append(pz)
            # a = [int(s) for s in string[17:-1].split() if s.isdigit()]
            # print(a)
            string = f.readline() 
            yaw = float(string[10:-2])
            f.readline()
            if control:
                string = f.readline()
                if string == "\n":
                    continue
                deslocamento, rot = list(map(float, string[35:-2].split()))
                string = f.readline()
                steps = int(string[6:-7])
                propagate()

    # print(len(grid))
    # print(len(grid[0]))
    array = np.zeros((len(grid)+CAM_DIST//5, len(grid[0]),3), dtype=np.uint8) #corrigir valor
    array = np.zeros((len(grid)+CAM_DIST//5, len(grid[0]),3), dtype=np.uint8) #corrigir valor
                
    # print(array.shape)
    # print("PXs and PZs:")
    # for i in range(len(pxs)):
        # print("px = {}; pz = {}".format(pxs[i], pzs[i]))

    for i in range(h):
        for j in range(w):
            if grid[i][j] == 1:
                array[i][j][0] = 255
                array[i][j][1] = 255
                array[i][j][2] = 255

    for i in range(h, h +  CAM_DIST//5):
        for j in range(w):
            array[i][j][0] = 0
            array[i][j][1] = 255
            array[i][j][2] = 255

    # array = np.array(grid, dtype=np.uint8)*255
    # if len(pxs)>0:
    #     cv2.line(array, (0,0), (pzs[0],pxs[0]), (255,0,255), 1)
    for i in range(0,len(pxs)-1):
        cv2.line(array, (pxs[i],pzs[i]), (pxs[i+1],pzs[i+1]), (255,0,255), 1)
    
    if with_goal_state:
        arrow_point = [0,0]
        goal_state_og = [get_grid_pos_x(goal_state[0]), get_grid_pos_z(goal_state[1])]
        arrow_point[0] = int(10*cos(goal_state[2]) + goal_state_og[0])
        arrow_point[1] = int(10*sin(-goal_state[2]) + goal_state_og[1])
        print(goal_state)
        print(arrow_point)
        print(cos(goal_state[2]))
        array = cv2.arrowedLine(array, (goal_state_og[0], goal_state_og[1]), (arrow_point[0], arrow_point[1]), thickness=3,color=(255,0,0))
        # cv2.line(array, (int(goal_state[0]), int(goal_state[1])), (arrow_point[0], arrow_point[1]), thickness=1,color=(255,0,0))
        # cv2.line(array, (0,0), (99,99), (255,255,255), 1)
        # print(array)
    cv2.imwrite("visualize_plan_results/"+file[:-4]+".png", array)
    # cv2.imshow("test", array)
    # cv2.waitKey(0)
    cv2.destroyAllWindows()