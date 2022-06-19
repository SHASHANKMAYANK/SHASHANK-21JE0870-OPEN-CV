import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import math
pshape=r"""C:\Users\SHASHANK\Desktop\opencv roboism\images"""+"/shapes.jpg"
pm1=r"""C:\Users\SHASHANK\Desktop\opencv roboism\images"""+"/marker 1.jpg"
pm2=r"""C:\Users\SHASHANK\Desktop\opencv roboism\images"""+"/marker 2.jpg"
pm3=r"""C:\Users\SHASHANK\Desktop\opencv roboism\images"""+"/marker 3.jpg"
pm4=r"""C:\Users\SHASHANK\Desktop\opencv roboism\images"""+"/marker 4.jpg"
arucoimglist=[]
#function to read and rotate arucos in the right confirmation
def arucoreader(pathname):
 img=cv.imread(pathname,0)
 key=getattr(aruco,f'DICT_5X5_250')
 arucodict=aruco.Dictionary_get(key)
 parameter=aruco.DetectorParameters_create()
 (corners,ids,rejected)=cv.aruco.detectMarkers(img,arucodict,parameters=parameter)
 print(corners)
 x1=corners[0][0][2][0]
 y1=corners[0][0][2][1]
 x2=corners[0][0][3][0]
 y2=corners[0][0][3][1]
 o=0
 if (x2-x1!=0):
  dy=y2-y1
  dx=x2-x1
  slope=dy/dx
  o = math.degrees(math.atan((dy / dx)))
  if (dx < 0 and dy > 0):
   o = o + 180
  elif (dx < 0 and dy <= 0):
   o = o + 180
  elif (dy < 0 and dx > 0):
   o = 360 + o
 else:
  o=90
 print(o)
 cx=int(corners[0][0][2][0]+corners[0][0][0][0])/2
 cy=int(corners[0][0][2][1]+corners[0][0][0][1])/2
 M = cv.getRotationMatrix2D((cx,cy), o, 1.0)
 rotated = cv.warpAffine(img, M,img.shape, borderValue=(255, 255, 255))
 arucoimglist.append(rotated)
 cv.imshow("Rotated image", rotated)
 cv.imshow("Real image",img)
 cv.waitKey(0)
 cv.destroyAllWindows()
arucoreader(pm1)
arucoreader(pm2)
arucoreader(pm3)
arucoreader(pm4)


def see(s,img):
    cv.imshow(s, img)
    cv.waitKey(0)
    cv.destroyAllWindows()

# reading images using cv2.imread function
orig=cv.imread(pshape)
dup=cv.imread(pshape,0)
_,thrash = cv.threshold(dup,240,255,cv.THRESH_BINARY)
contours,_=cv.findContours(thrash,cv.RETR_TREE,cv.CHAIN_APPROX_NONE)
for contour in contours:
    approx=cv.approxPolyDP(contour,0.01*cv.arcLength(contour,True),True)

    cv.drawContours(orig,[approx],0,(0,0,0),5)
    x=approx.ravel()[0]
    y=approx.ravel()[1]
    i=0
    if len(approx)==4 :
        X,Y,w,h=cv.boundingRect(approx)
        aspectratio=float(w)/h
        if(aspectratio>=1and aspectratio<=1):
            cv.putText(orig, "square", (X, Y + 150), cv.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0))
            print(approx[0])
            print(approx[1])
            x1=approx[0][0][0]
            y1=approx[0][0][1]
            x2=approx[1][0][0]
            y2=approx[1][0][1]
            dy=y1-y2
            dx=x1-x2
            slope=dy/dx
            o1 = math.degrees(math.atan((dy / dx)))
            if (dx < 0 and dy > 0):
                o1 = o1 + 180
            elif (dx < 0 and dy <= 0):
                o1 = o1 + 180
            elif (dy < 0 and dx > 0):
                o1 = 360 + o1
        else:
            o1 = 90
        print(o1)
        arucoimg = arucoimglist[i].copy()
        i=i+1
        key = getattr(aruco, f'DICT_5X5_250')
        arucodict = aruco.Dictionary_get(key)
        parameter = aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv.aruco.detectMarkers(arucoimg, arucodict, parameters=parameter)
        print(corners)
        x1 = corners[0][0][2][0]
        y1 = corners[0][0][2][1]
        x2 = corners[0][0][3][0]
        y2 = corners[0][0][3][1]
        o = 0
        if (x2 - x1 != 0):
            dy = y2 - y1
            dx = x2 - x1
            slope = dy / dx
            o = math.degrees(math.atan((dy / dx)))
            if (dx < 0 and dy > 0):
                o = o + 180
            elif (dx < 0 and dy <= 0):
                o = o + 180
            elif (dy < 0 and dx > 0):
                o = 360 + o
        else:
            o = 90
        print(o)
        cx = int(corners[0][0][2][0] + corners[0][0][0][0]) / 2
        cy = int(corners[0][0][2][1] + corners[0][0][0][1]) / 2
        M = cv.getRotationMatrix2D((cx, cy), o1, 1.0)
        rotated = cv.warpAffine(arucoimg, M, arucoimg.shape, borderValue=(255, 255, 255))

        cv.imshow("Real image", arucoimg)
        cv.imshow("Rotated image", rotated)

        cv.waitKey(0)
        cv.destroyAllWindows()
see("orig",orig)
