import numpy as np
import imutils
import cv2

print(cv2.getVersionString())

cap = cv2.VideoCapture(0)

print(cap.isOpened())

blocksize = 5
blurRad = 5
done = False
C1=1
C2=100

ratioUpper = 2.5
ratioLower = 0.5
T = 0
B = 100
L = 0
R = 100
minArea = 0
maxArea = 1000

def onThresh(val):
    global blocksize
    blocksize = val + (val % 2-1)


def onBlur(val):
    global blurRad
    blurRad = val + (val % 2-1)
        
def onC1(val):
    global C1
    C1 = val
def onC2(val):
    global C2
    C2 = val

def onRatioUpper(val):
    global ratioUpper
    ratioUpper = val/10

def onRatioLower(val):
    global ratioLower
    ratioLower = val/10

    
def onTop(val):
    global T
    T = val
def onBottom(val):
    global B
    B = val
def onLeft(val):
    global L
    L = val
def onRight(val):
    global R
    R = val
def onMinArea(val):
    global minArea
    minArea = val
def onMaxArea(val):
    global maxArea
    maxArea = val


onThresh(228)
onBlur(3)
onRatioUpper(18)
onRatioLower(13)
onTop(344)
onBottom(433)
onLeft(420)
onRight(640)



def doOnce():
    global done
    if done:
        return
    done = True
    cv2.createTrackbar('threshold', 'thresh', 228, 500, onThresh)
    cv2.createTrackbar('blur', 'blurred', 3, 50, onBlur)
    cv2.createTrackbar('C1', 'canny', 10, 500, onC1)
    cv2.createTrackbar('C2', 'canny', 10, 500, onC2)
    cv2.createTrackbar('ratioUpper', 'results', 18, 100, onRatioUpper)
    cv2.createTrackbar('ratioLower', 'results', 13, 100, onRatioLower)
    cv2.createTrackbar('T', 'results', 344, 480, onTop)
    cv2.createTrackbar('B', 'results', 433, 480, onBottom)
    cv2.createTrackbar('L', 'results', 420, 640, onLeft)
    cv2.createTrackbar('R', 'results', 640, 640, onRight)
    cv2.createTrackbar('minArea', 'results', 10, 500, onMinArea)
    cv2.createTrackbar('maxArea', 'results', 10, 500, onMaxArea)


while(True):
    # Load image
    ret, frame = cap.read()
    raw = frame

    img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow('image', img)

    img = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                cv2.THRESH_BINARY, blocksize, 2)
    cv2.imshow('thresh', img)

    img = cv2.medianBlur(img, blurRad)
    cv2.imshow('blurred', img)

    img = cv2.Canny(img, C1, C2, -1, 3, True)
    cv2.imshow('canny', img)

    # ret, thresh = cv2.threshold(img, 127, 255, 0)
    # print(thresh[0])
    # type(thresh[0])
    # contours, hierarchy  = cv2.findContours(thresh, 1, 2)

    contours, hierarchy = cv2.findContours(img.copy(), cv2.RETR_TREE,
                                cv2.CHAIN_APPROX_SIMPLE)
    mask = np.zeros(raw.shape, np.uint8)
    i = 0
    _raw = raw.copy()
    for c in contours:
        x,y,w,h = cv2.boundingRect(c)
        area = cv2.contourArea(c)
        if w/h < ratioUpper and w/h > ratioLower and y > T and y+h < B and x > L and x+w < R:
            
            roi = _raw[y:y+h, x:x+w]
            roi = cv2.resize(roi, (w*10, h*10))
            cv2.imshow("ROI "+str(i), roi)

            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.rectangle(raw, (x,y), (x+w,y+h), (0, 0, 255), 2)
            cv2.drawContours(raw,[box],0,(255,0,255),2)
            cv2.drawContours(mask, [box], 0, 255, -1,)
            i = i+1
    cv2.rectangle(raw, (L,T), (R, B), (0,255,0), 2)
    # type(contours[0])
    # cv2.drawContours(raw, contours, -1, (0,255,0), 3)
    # cv2.imshow('contours', img)
    cv2.imshow('results', raw)

    img = cv2.bitwise_and(raw, raw, mask=mask[:,:,0])
    cv2.imshow('masked', img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    doOnce()

cap.release()
cv2.destroyAllWindows()
