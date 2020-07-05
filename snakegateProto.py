"""
@gothWare
<<<PARTIALLY COMPLETE CODE>>>
"""


import cv2
import numpy as np

threshold = np.zeros([600,600,3])
threshold[200:400,200:400]=np.array([0,0,255])
threshold[50:100,50:100] = np.array([255,0,0])
threshold[225:375,225:375]=np.array([0,0,0])
#threshold[50:100,50:100] = np.array([255,0,0])

result_im = np.zeros([600,600,3])
result_im[200:400,200:400]=np.array([0,0,255])
result_im[50:100,50:100] = np.array([255,0,0])
result_im[225:375,225:375]=np.array([0,0,0])

delta = 5
pixcol = [0,0,255]
for itr in range(10000):
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
                            result_im[fy,fx] = np.array([255,0,0])
                            break
                        else:
                            result_im[fy:fy+delta,fx:fx+delta] = np.array([0,255,0])
                            #if list(leftref)!=
                            break

                    else:
                        result_im[fy:fy+delta,fx:fx+delta] = np.array([0,255,0])


                        break
                except IndexError as Ind:
                    break
#print(im)
#res_med = cv2.medianBlur(threshold,3)
cv2.imshow("img", threshold)
cv2.imshow("det", result_im)
cv2.waitKey()
cv2.destroyAllWindows()