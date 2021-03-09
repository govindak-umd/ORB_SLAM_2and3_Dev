import cv2

# Read the video
vidcap = cv2.VideoCapture('VideoCapture.avi')
def getFrame(sec):
    vidcap.set(cv2.CAP_PROP_POS_MSEC,sec*1000)
    hasFrames,image = vidcap.read()
    if hasFrames:
    	# Create a folder with the name of the video
        cv2.imwrite("VideoCapture/image"+str(count)+".jpg", image)     # save frame as JPG file
    return hasFrames
sec = 0
# It will capture image in each 0.5 second
frameRate = 0.1 
count=1
success = getFrame(sec)
print('Video Read')
while success:
    count = count + 1
    print('Sequenced ', count, ' frames')
    sec = sec + frameRate
    sec = round(sec, 2)
    success = getFrame(sec)