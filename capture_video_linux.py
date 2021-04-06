import cv2
import time
import os
from utils import test_name

os.mkdir(test_name)
capture = cv2.VideoCapture(1)

dim = (640,480)
print('Saving Image Frames now ...')
count = 0
try:
    while (True):
        ret, frame = capture.read()

        if ret:
            len_number = len(str(count))
            number_name = "0"*(6-len_number)
            frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)

            cv2.imwrite(test_name+"/"+number_name+str(count)+".png", frame)
	    count+=1

        if cv2.waitKey(1) == 27:
            break

# Save when the user exits
except KeyboardInterrupt:
    print('Video Closed')
