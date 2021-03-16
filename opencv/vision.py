import numpy as np
import imutils
import cv2

print(cv2.getVersionString())

cap = cv2.VideoCapture(0)

print(cap.isOpened())

blocksize = 5
blurRad = 5
done = False

ratioUpper = 2.5
ratioLower = 0.5
T = 0
B = 100

def onThresh(val):
    global blocksize
    blocksize = val + (val % 2-1)


def onBlur(val):
    global blurRad
    blurRad = val + (val % 2-1)

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

onThresh(720)
onBlur(5)


def doOnce():
    global done
    if done:
        return
    done = True
    cv2.createTrackbar('threshold', 'thresh', 720, 10000, onThresh)
    cv2.createTrackbar('blur', 'blurred', 1, 50, onBlur)
    cv2.createTrackbar('ratioUpper', 'results', 1, 100, onRatioUpper)
    cv2.createTrackbar('ratioLower', 'results', 1, 100, onRatioLower)
    cv2.createTrackbar('T', 'results', 0, 480, onTop)
    cv2.createTrackbar('B', 'results', 0, 480, onBottom)


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

    img = cv2.Canny(img, 1, 100)
    cv2.imshow('canny', img)

    # ret, thresh = cv2.threshold(img, 127, 255, 0)
    # print(thresh[0])
    # type(thresh[0])
    # contours, hierarchy  = cv2.findContours(thresh, 1, 2)

    contours, hierarchy = cv2.findContours(img.copy(), cv2.RETR_TREE,
                                cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        x,y,w,h = cv2.boundingRect(c)
        if w/h < ratioUpper and w/h > ratioLower and y > T and y+h < B:
            cv2.rectangle(raw, (x,y), (x+w,y+h), (0, 0, 255), 2)
    cv2.rectangle(raw, (0,T), (640, B), (0,255,0), 2)
    # type(contours[0])
    # cv2.drawContours(raw, contours, -1, (0,255,0), 3)
    # cv2.imshow('contours', img)

    cv2.imshow('results', raw)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    doOnce()

cap.release()
cv2.destroyAllWindows()
