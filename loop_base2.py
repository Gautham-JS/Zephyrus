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
    l.append(np.sum([(col[j]==pixcol).all() for j in range(0,frame.shape[0],2)]))

print(l)

for ind,val in enumerate(l):
    if len(bound)!=4:
        if not b:
            if val >= thresh:
                bound.append(ind*2)
                b=1
        else:import numpy as np
import cv2
import time

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
tik = time.time()
for i in range(0,frame.shape[1],2):
    col = frame[:,i]
    l.append(np.sum([(col[j]==pixcol).all() for j in range(frame.shape[0])]))

print(len(l))

for ind,val in enumerate(l):
    if len(bound)!=4:
        if not b:
            if val >= thresh:
                bound.append(ind*2)
                b=1
        else:
            if val < thresh:
                bound.append(ind*2)
                b=0

col = frame[:,bound[0]+5]
r = pixcol
i = 0 
while i in range(frame.shape[0]):
    if not (frame[i,bound[0]+5] == pixcol).all():
        break
    i+=1
    


print(bound)
rup = (bound[0],0)
rdown = (bound[1],i)
lup = (bound[2],0)
ldown = (bound[3],i)

cv2.rectangle(frame,rup,rdown,[255,0,0],3)
cv2.rectangle(frame,lup,ldown,[255,0,0],3)
tok = time.time()

tiktok = tok - tik
print(tiktok)
cv2.imshow('win',frame)
cv2.waitKey(0) 
#if cv2.waitKey(1) & 0xFF == ord('q'):
#    break
#cap.release()
cv2.destroyAllWindows()


            if val < thresh:
                bound.append(ind*2)
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
