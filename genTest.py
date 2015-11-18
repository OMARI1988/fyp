import cv2
import numpy as np



def drawCross(x,y,size):
    mod = size * 10
    cv2.line(img,(x,y-mod),(x,y+mod),(255,50,100),size)
    cv2.line(img,(x-mod,y),(x+mod,y),(255,50,100),size)

height = 3508#600#3508
width = 4961#960#4961
size1 = 10
size2 = 30
thickness = 20
radius = 200

img = np.zeros((height,width,3), np.uint8) 
img[:,:] = (255,255,255)

xc = width/2
yc = height/2

mod = height/3

drawCross(xc+mod,yc,size1)
cv2.circle(img,(xc+mod,yc),radius,(0,0,0),thickness)

drawCross(xc,yc+mod,size1)
cv2.circle(img,(xc,yc+mod),radius,(0,0,0),thickness)

drawCross(xc+mod,yc+mod,size1)
cv2.circle(img,(xc+mod,yc+mod),radius,(0,0,0),thickness)

drawCross(xc-mod,yc-mod,size1)
cv2.circle(img,(xc-mod,yc-mod),radius,(0,0,0),thickness)

drawCross(xc-mod,yc+mod,size1)
cv2.circle(img,(xc-mod,yc+mod),radius,(0,0,0),thickness)

drawCross(xc+mod,yc-mod,size1)
cv2.circle(img,(xc+mod,yc-mod),radius,(0,0,0),thickness)

drawCross(xc,yc-mod,size1)
cv2.circle(img,(xc,yc-mod),radius,(0,0,0),thickness)

drawCross(xc-mod,yc,size1)
cv2.circle(img,(xc-mod,yc),radius,(0,0,0),thickness)


drawCross(xc,yc,size2)

cv2.circle(img,(xc,yc),2,(0,0,255), -1)#cv2.circle(img,(xc,yc),(255,50,100),3)
#cv2.imshow('img',img)
#cv2.waitKey()
cv2.imwrite('test.png',img)

