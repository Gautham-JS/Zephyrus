"""
@gothWare
"""


import cv2
import numpy as np
import imutils

"""
threshold = np.zeros([600,600,3])
threshold[200:400,200:400]=np.array([0,255,255])
threshold[50:60,50:60] = np.array([0,255,255])
threshold[225:375,225:375]=np.array([0,0,0])
#threshold[50:100,50:100] = np.array([255,0,0])

result_im = np.zeros([600,600,3])
result_im[200:400,200:400]=np.array([0,255,255])
result_im[50:60,50:60] = np.array([0,255,255])
result_im[225:375,225:375]=np.array([0,0,0])
"""
thr = 110
cam = cv2.VideoCapture(0)
delta = 10
pixcol = [0,0,255]


for frame in range(100):
    _, img = cam.read()
    img2 = cv2.medianBlur(img,5)
    retval, threshold = cv2.threshold(img2, thr, 255, cv2.THRESH_BINARY)
    retval, result_im = cv2.threshold(img2, thr, 255, cv2.THRESH_BINARY)

    bottom_right_corner = np.zeros(2)
    top_left_corner = np.array([threshold.shape[1],threshold.shape[0]])
    if cv2.waitKey(200q) & 0xFF == ord('q'):
        break
    for itr in range(1000):
            search_ind = np.zeros(2,np.int32)
            search_ind[0] = np.random.randint(0,threshold.shape[0])
            search_ind[1] = np.random.randint(0,threshold.shape[1])
            #print(threshold[search_ind[0]][search_ind[1]])
            if list(threshold[search_ind[0],search_ind[1]]) == pixcol:
                #print("searching around red pixel at {}".format(search_ind))
                fy = search_ind[0]
                fx = search_ind[1]
                sz = 0
                while True:
                    try:
                        uppix = threshold[fy-delta][fx]
                        downpix = threshold[fy+delta][fx]
                        up_down_composite = np.logical_and(list(uppix)==pixcol , list(downpix)==pixcol)
                        print("pixel delta {}".format([up_down_composite, uppix, downpix]))
                        if np.logical_and(list(uppix)==pixcol , list(downpix)==pixcol)==1:
                            #print("downpix and uppix are also red and within delta")
                            upleftpix = threshold[fy-delta][fx-delta]
                            uprightpix = threshold[fy-delta][fx+delta]

                            downleftpix = threshold[fy+delta][fx-delta]
                            downrightpix = threshold[fy+delta][fx+delta]

                            leftref = np.logical_and(upleftpix,downleftpix)
                            rightref = np.logical_and(downrightpix, uprightpix)
                            upref = np.logical_and(upleftpix,uprightpix)
                            downref = np.logical_and(downleftpix,downrightpix)
                            #threshold[fy:fy+delta,fx:fx+delta] = np.array([255,0,0])

                            
                            #print("\nLRcomp is {}\n".format([np.logical_and(np.logical_and(list(leftref),pixcol) ,np.logical_and(list(rightref),pixcol)),leftref,rightref]))
                            if list(np.logical_and( np.logical_and(list(leftref),pixcol),
                            np.logical_and(list(rightref),pixcol)) ) == [False,False,True]:
                                print("valid crosspoint detected")
                                #result_im[fy:fy+10,fx:fx+10] = np.array([255,0,0])
                                if fy>bottom_right_corner[0]:
                                    bottom_right_corner[0]=fy
                                if fx>bottom_right_corner[1]:
                                    bottom_right_corner[1]=fx

                                if fy<top_left_corner[0]:
                                    top_left_corner[0]=fy
                                if fx<top_left_corner[1]:
                                    top_left_corner[1]=fx
                                break
                            else:
                                break
                        else:
                            break
                    except IndexError as Ind:
                        break


    br = [int(i) for i in bottom_right_corner]
    tl = [int(i) for i in top_left_corner]
    #result_im[br[0]:br[0]+20, br[1]:br[1]+20] = np.array([0,0,255])
    #result_im[tl[0]-20:tl[0], tl[1]-20:tl[1]] = np.array([0,0,255])

    cv2.rectangle(result_im,(tl[1]-delta,tl[0]-delta),(br[1]+delta,br[0]+delta), [200,255,0],5)
    cv2.putText(result_im,'Detection',(br[1]+10,br[0]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),2)

    mask = cv2.inRange(result_im,np.array([199,254,0]), np.array([200,255,0]))
    out = cv2.bitwise_and(result_im,result_im,mask=mask)

    out = cv2.cvtColor(out,cv2.COLOR_BGR2GRAY)

    res_2 = cv2.findContours(out ,cv2.RETR_EXTERNAL ,cv2.CHAIN_APPROX_SIMPLE)
    res_2 = imutils.grab_contours(res_2)

    for c in res_2:
        M = cv2.moments(c)
        cx = int(M["m10"]/M["m00"])
        cy = int(M["m01"]/M["m00"])

        cv2.circle(result_im, (cx,cy), 5,(255,255,255),-1)
        cv2.putText(result_im, "center", (cx-20, cy-20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, [225,225,225], 2)


    cv2.imshow("original", img)
    cv2.imshow("threshold", threshold)
    cv2.imshow("detected", result_im)
    cv2.waitKey(100)
    
    
cam.release()
cv2.destroyAllWindows()
