import numpy as np
import cv2

frame = np.zeros((600,600,3))
frame[0:200,200:400] = [0,0,255]
frame[0:150,250:350] = [0,0,0]

cv2.imshow('orig',frame)

pixcol = [0,0,255]
l = []
thresh = 100
bound = []
b = 0

#video_path = 'vid.mp4'
#cap = cv2.VideoCapture(0)

#_,frame2 = cap.read()
#while True:
#   _,framecap.read()
    
for i in range(frame.shape[1]):
    col = frame[:,i]
    l.append(np.sum([(col[j]==pixcol).all() for j in range(frame.shape[0])]))

print(l)

for ind,val in enumerate(l):
    if not b:
        if val >= thresh:
            bound.append(ind)
            b=1
    else:
        if val < thresh:
            bound.append(ind)
            b=0

print(bound)
rup = (bound[0],0)
rdown = (bound[1],200)
lup = (bound[2],0)
ldown = (bound[3],200)

cv2.rectangle(frame,rup,rdown,[255,0,0],3)
cv2.rectangle(frame,lup,ldown,[255,0,0],3)

cv2.imshow('win',frame)
cv2.waitKey(0) 
#if cv2.waitKey(1) & 0xFF == ord('q'):
#    break
#cap.release()
cv2.destroyAllWindows()
