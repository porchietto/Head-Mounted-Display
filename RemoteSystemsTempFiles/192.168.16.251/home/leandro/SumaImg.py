
import cv2
import numpy as np



WIDTH = 800
HEIGHT = 600

OffSetX = 100
OffSetY = 0


Frame = np.zeros((HEIGHT,WIDTH),dtype=np.uint8)



fondo_img = cv2.imread('fondo2.png',cv2.IMREAD_GRAYSCALE)
horizonte_img = cv2.imread('horizonte2.png',cv2.IMREAD_GRAYSCALE)



Frame[:, OffSetX:OffSetX+fondo_img.shape[1]] = fondo_img

def dibujarHorizonte(grados,horizonte_img):
    global Frame
    (h, w) = horizonte_img.shape[:2]

    center = (w / 2, h / 2)

    M = cv2.getRotationMatrix2D(center, grados, 1)
    rotated90 = cv2.warpAffine(horizonte_img, M, (h, w))
    
    porcion_fondo = Frame[int(WIDTH/2):int((fondo_img.shape[0]-rotated90.shape[0])),int((WIDTH/2)-(rotated90.shape[0]/2)):int((WIDTH/2)+(rotated90.shape[0]/2))]

    dst = cv2.addWeighted(rotated90,1,porcion_fondo,1,0)
    
    Frame[int(WIDTH/2):int((fondo_img.shape[0]-rotated90.shape[0])),int((WIDTH/2)-(rotated90.shape[0]/2)):int((WIDTH/2)+(rotated90.shape[0]/2))] = dst
    
    return True
  
if dibujarHorizonte:
    dibujarHorizonte(90, horizonte_img)     
    cv2.imwrite('frame_suma.png',Frame)
