import os
import time

i=0
while True:
  i=i+1
  os.system('libcamera-still -n -o "data_storage/current_img.jpg --width 224 --height 224"')
  time.sleep(0.1)
  print("img {} generated\n".format(i))
