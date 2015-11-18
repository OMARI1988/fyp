import cv2
import numpy as np



def drawRect(x,y,size):

    cv2.rectangle(img,(int(x-size),int(y-size)),(int(x+size),int(y+size)),(0,0,0),10)

   

height = 3508#600#3508
width = 4961#960#4961

thickness = 20#2#20
radius = 200#50#150

img = np.zeros((height,width,3), np.uint8) 
img[:,:] = (255,255,255)

xc = width/2
yc = height/2


mod = float(height/3.1)
size1 = float(mod/2)

#draw grid boxes
drawRect(xc+mod,yc,size1)
drawRect(xc,yc+mod,size1)
drawRect(xc+mod,yc+mod,size1)
drawRect(xc-mod,yc-mod,size1)
drawRect(xc-mod,yc+mod,size1)
drawRect(xc+mod,yc-mod,size1)
drawRect(xc,yc-mod,size1)
drawRect(xc-mod,yc,size1)
drawRect(xc,yc,size1)

#draw tracking circles
cv2.circle(img,(int(xc-(mod*2)),int(yc-mod)),radius,(0,0,0),thickness)
cv2.circle(img,(int(xc+(mod*2)),int(yc-mod)),radius,(0,0,0),thickness)
cv2.circle(img,(int(xc-(mod*2)),int(yc+mod)),radius,(0,0,0),thickness)
cv2.circle(img,(int(xc+(mod*2)),int(yc+mod)),radius,(0,0,0),thickness)


#cv2.imshow('img',img)
#cv2.waitKey()
cv2.imwrite('board4.png',img)

