from os import listdir
from os.path import isfile, join

dir = "../../../og_images/"
onlyfiles = [f for f in listdir(dir) if isfile(join(dir, f))]
onlyfiles.sort()
print(onlyfiles)

send_to_file = ""
for file in onlyfiles:
    send_to_file +=file+"\n"
    
    
file = open("list.txt", "w")
send_to_file+="$\n"
file.write(send_to_file)
file.close()