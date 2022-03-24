get_all = False
dic = dict()

for i in range(6, 14):
    dic[i] = list()
with open('dist_info.txt', 'r') as f:
    str = f.readline()
    while(str != ""):
        dist = str.split()[-1]
        dist = float(dist)
        dic[int(dist//10)].append(dist)
        str = f.readline()

print("dists")
for i in range(6,14):
    print(dic[i])

print("len:")
for i in range(6,14):
    print(len(dic[i]))


