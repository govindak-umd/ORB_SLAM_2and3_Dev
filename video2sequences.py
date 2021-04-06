import cv2
import os

video_name = "video_11"
os.mkdir(video_name)
# Read the video
vidcap = cv2.VideoCapture(video_name+'.avi')
def getFrame(sec):
    vidcap.set(cv2.CAP_PROP_POS_MSEC,sec*1000)
    hasFrames,image = vidcap.read()
    if hasFrames:
    	# Create a folder with the name of the video
    	# and save files, in the same manner read by mono_kitti
    	# It has to be 000000.png onwards
        len_number = len(str(count))
        number_name = "0"*(6-len_number)
        cv2.imwrite(video_name+"/"+number_name+str(count)+".png", image)
    return hasFrames
sec = 0
# It will capture image in each 0.5 second
frameRate = 0.1 
count=0
success = getFrame(sec)
print('Video Read')
f= open(video_name+".txt","w+")
while success:
    sec2write = "{:e}".format(sec)
    print("sec2write ", sec2write)
    f.write(str(sec2write)+"\n")
    count = count + 1
    print('Sequenced ', count, ' frames')
    sec = sec + frameRate
    sec = round(sec, 2)
    success = getFrame(sec)
f.close()
