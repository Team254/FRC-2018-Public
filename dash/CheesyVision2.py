import numpy as np
import cv2
from networktables import NetworkTables

import math
from time import perf_counter
import collections
import argparse



##########################################
############# some utilities #############
##########################################

def fadeHSV(image, mask):
    fade = cv2.multiply(image, (0.6,))
    cv2.subtract(image, fade, image, cv2.bitwise_not(mask))

def getKernel(size):
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size,size))

def getColorMask(input):
    # convert to HSV
    hsv = cv2.cvtColor(input, cv2.COLOR_BGR2HSV)
    cv2.medianBlur(hsv, 5, hsv)
    
    # threshold
    global minColor, maxColor
    halfW = hsv.shape[1] // 2
    maskL = cv2.inRange(hsv[:, :halfW], minColor[0], maxColor[0])
    maskR = cv2.inRange(hsv[:, halfW:], minColor[1], maxColor[1])
    mask = np.hstack((maskL, maskR))
    
    return mask, hsv



#########################################
############# main pipeline #############
#########################################

MAX_LINE_ANGLE = 15

angleMap = np.zeros((1,1))
angleMask = None
pivotChanged = True
def initAngleMap(shape):
    start = perf_counter()
    
    global angleMap, angleMask, pivotChanged
    height, width = shape
    
    # create angleMap
    dxs = np.tile(np.arange(width) - pivotLoc[0], (height,1))
    dys = np.tile(np.arange(height).reshape(height,1) - pivotLoc[1], (1,width))
    invertMask = dxs < 0
    dxs[invertMask] = np.negative(dxs[invertMask])
    dys[invertMask] = np.negative(dys[invertMask])
    angleMap = np.degrees(np.arctan2(dys, dxs))
    
    # create angleMask
    angleMask = cv2.inRange(angleMap, -MAX_LINE_ANGLE, +MAX_LINE_ANGLE)
    cx, cy, r = int(pivotLoc[0]), int(pivotLoc[1]), shape[1]//12
    cv2.rectangle(angleMask, (cx-r,cy-r), (cx+r,cy+r), 0, cv2.FILLED)
    
    pivotChanged = False
    
    end = perf_counter()
    if args.debug_timing: print(f"initAngleMap took {int((end-start)*1000)} ms")

autoPivotMask = None
def autoDetectPivot():
    start = perf_counter()
    
    global pivotLoc, pivotChanged
    if autoPivotMask is not None:
        # filter contours by aspect ratio
        _, contours, _ = cv2.findContours(autoPivotMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        def keep(contour):
            _, (w, h), _ = cv2.minAreaRect(contour)
            if w == 0 or h == 0: return False
            aspect = w / h
            if aspect < 1.0: aspect = 1/aspect
            return aspect > 4.5
        contours = [c for c in contours if keep(c)]
        if len(contours) == 0: return
        
        # set pivot to median of the filtered blobs
        xs, ys = np.hstack(c[:, 0, 0] for c in contours), np.hstack(c[:, 0, 1] for c in contours)
        if len(xs) > 0 and len(ys) > 0:
            pivotLoc = ((xs.max()+xs.min())/2, (ys.max()+ys.min())/2)
            pivotChanged = True
    
    end = perf_counter()
    if args.debug_timing: print(f"autoDetectPivot took {int((end-start)*1000)} ms")

def process(input):
    shape = input.shape[:2]
    height, width = shape
    
    global pivotLoc
    if pivotLoc is None:
        pivotLoc = (width//2, height//2)
    if pivotChanged or angleMap.shape != shape:
        initAngleMap(shape)
    
    if args.auto:
        autoSetColor(input)
    
    start = perf_counter()
    
    # get the color mask
    global curFrame
    mask, curFrame = getColorMask(input)
    
    # dilate the mask a bit
    cv2.dilate(mask, getKernel(4), mask)
    
    # compute distance transform
    dist = cv2.distanceTransform(cv2.copyMakeBorder(mask, 1,1,1,1, cv2.BORDER_CONSTANT, value=0), cv2.DIST_L2, 3)
    dist = dist[1:-1, 1:-1] # cut off the temporary border
    maxDist = float(dist.max())
    
    # threshold the distance transform
    mask2 = cv2.inRange(dist, maxDist*0.7, maxDist) if maxDist > 0 else np.zeros(shape, dtype=np.uint8)
    global autoPivotMask
    autoPivotMask = mask2.copy()
    
    # mask out bad angles
    cv2.bitwise_and(mask2, angleMask, mask2)
    
    if cv2.countNonZero(mask2) > 0:
        # compute average angle
        angle = cv2.mean(angleMap, mask2)[0]
        
        # histogram...
        #...
        
        updateAngle([-angle, -angle])
    else:
        angle = None
        updateAngle(None)
    
    
    
    end = perf_counter()
    if args.debug_timing: print(f"process took {int((end-start)*1000)} ms")
    
    ### draw debug info onto the input image and show it ###
    if args.debug_mask:
        cv2.imshow("mask", getColorMask(input)[0])
        cv2.imshow("dist transform", dist/(maxDist+0.01))
        if mask2 is not None: cv2.imshow("mask2", autoPivotMask)
    
    output = input.copy()
    fadeHSV(output, mask)
    
    # blow up image for easier viewing
    if args.roi_scale != 1.0:
        output = cv2.resize(output, (0,0), fx=args.roi_scale, fy=args.roi_scale, interpolation=cv2.INTER_NEAREST)
    
    # draw pivot and detected angle
    cx, cy = int(pivotLoc[0]*args.roi_scale), int(pivotLoc[1]*args.roi_scale)
    if angle is not None:
        dx = int(2000*math.cos(math.radians(angle)))
        dy = int(2000*math.sin(math.radians(angle)))
        cv2.line(output, (cx-dx,cy-dy), (cx+dx,cy+dy), (0,255,0), 1, cv2.LINE_AA)
    cv2.circle(output, (cx,cy), 3, (255,255,0), cv2.FILLED)
    
    def drawText(text, x, y, color, size=0.4, fromM=0):
        textSz, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, size, 1)
        y += int(textSz[1]*fromM)
        cv2.putText(output, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                    size, color, 1, cv2.LINE_AA)
    
    # FPS/debug text
    global dt, fps
    debugStr = ""
    if dt is not None:
        debugStr += f"{int(dt*1000)} ms"
    if fps is not None:
        debugStr += f" ({int(fps)} FPS)"
    drawText(debugStr, 10, int(height*args.roi_scale)-10, (0,255,0))
    if not NetworkTables.isConnected():
        drawText("Not Connected!", 10, int(height*args.roi_scale)-45, (0,0,255))
    else:
        drawText("Connected!", 10, int(height*args.roi_scale)-45, (0,255,0))
    drawText("tuning mode = auto" if args.auto else "tuning mode = manual", 60, 29, (0, 255, 0))
    if selectingPivot:
        drawText("SELECTING PIVOT", 60, 47, (0, 255, 0))
    drawText(f"using device {args.device}", 10, int(height*args.roi_scale)-30, (0,255,0))
    
    # visualize the detected angle and state
    angle = getAngle()
    if angle is not None:
        PRE_SZ = 50
        cv2.rectangle(output, (0, 0), (PRE_SZ, PRE_SZ), (255,255,255), cv2.FILLED)
        rads = math.radians(angle)
        offX = 200*math.cos(rads)
        offY = -200*math.sin(rads)
        cv2.line(output[0:PRE_SZ, 0:PRE_SZ], (int(PRE_SZ/2-offX),int(PRE_SZ/2-offY)), (int(PRE_SZ/2+offX),int(PRE_SZ/2+offY)), (0,0,0), lineType=cv2.LINE_AA)
        drawText(f"angle = {int(angle*100)/100} deg  (tip = {getTip()})", 60, 3, (0,255,0), fromM=1)
        if errorMsg is not None:
            drawText(errorMsg, 5, 60, (0,0,255), fromM=1)
    
    cv2.line(output, (output.shape[1]//2, 0), (output.shape[1]//2, output.shape[0]), (255, 0, 0), 2)
    cv2.imshow("raw", output)



#########################################
######### automatic calibration #########
#########################################

HIST_SMOOTH_RADIUS = 2
HIST_SMOOTH_KERNEL = np.hamming(HIST_SMOOTH_RADIUS*2 + 1)
HIST_SMOOTH_KERNEL /= HIST_SMOOTH_KERNEL.sum()
def getHistogram(hsv, channel, mask, normMax=255, reduce=3):
    maxV = [180,255,255][channel]
    hist = cv2.calcHist([hsv[:, :, channel]], [0], mask, [maxV//reduce], [0,maxV])
    
    # smooth histogram
    hist = np.r_[hist[-HIST_SMOOTH_RADIUS:], hist, hist[:HIST_SMOOTH_RADIUS]]
    hist = hist.reshape(len(hist))
    hist = np.convolve(hist, HIST_SMOOTH_KERNEL, mode="valid")
    
    cv2.normalize(hist, hist, 0, normMax, cv2.NORM_MINMAX)
    return hist

def drawHistogram(hist, markers=[], bestMarker=-1):
    n = hist.shape[0]
    m = 180//n
    histImage = np.zeros((256, n*2, 3), np.uint8)
    hist = np.int32(np.around(hist))
    markers = markers + [bestMarker]
    for x,y in enumerate(hist):
        cv2.line(histImage, (x*2,256), (x*2,256-y), (x*m,255,255))
        cv2.line(histImage, (x*2+1,256), (x*2+1,256-y), (x*m+1,255,255))
        if x in markers:
            color = (0,0,255)
            if x == bestMarker: color = (60,255,255)
            cv2.line(histImage, (x*2,0), (x*2,256-y), color)
    histImage = cv2.cvtColor(histImage, cv2.COLOR_HSV2BGR)
    cv2.imshow("histogram", histImage)

def getMaxima(hist):
    def at(i): return hist[(i) % len(hist)]
    return [i for i in range(1, len(hist)-1) if hist[i] > at(i-1) and hist[i] > at(i+1)]
def getClosestExtremum(extrema, target):
    return extrema[np.argmin([abs(i-target) for i in extrema])]
def getClosestMinLeft(hist, maxI):
    minI = maxI
    while minI > 1 and hist[minI] > hist[minI-1]:
        minI -= 1
    return minI
def getClosestMinRight(hist, maxI):
    minI = maxI
    while minI < len(hist)-2 and hist[minI] > hist[minI+1]:
        minI += 1
    return minI

lastMax, lastMin0, lastMin1 = None, None, None
MIN_HUE, TARGET_HUE, MAX_HUE = 126, 148, 164
def computeHueRange(hsv):
    start = perf_counter()
    
    # compute hue histogram
    mask = cv2.inRange(hsv, (0, 64, 64), (180, 255, 255))
    hist = getHistogram(hsv, 0, mask, reduce=2)
    
    # find hue range
    maxima = getMaxima(hist)
    bestMax = getClosestExtremum(maxima, TARGET_HUE / 2)
    min0 = getClosestMinLeft(hist, bestMax)
    min1 = getClosestMinRight(hist, bestMax)
    
    # update positions
    global lastMax, lastMin0, lastMin1
    def update(last, cur):
        cur = min(max(cur, MIN_HUE//2), MAX_HUE//2) # clamp to for-sure range
        if last is None: return cur
        if cur > last: return last+1
        if cur < last: return last-1
        return last
    lastMax = update(lastMax, bestMax)
    lastMin0 = update(lastMin0, min0)
    lastMin1 = update(lastMin1, min1)
    
    end = perf_counter()
    if args.debug_timing: print(f"    computeHueRange took {int((end-start)*1000)} ms")
    
    if args.debug_histograms:
        # visualize the histogram
        drawHistogram(hist, [lastMin0, lastMin1], lastMax)
        #drawHistogram(hist, [MIN_HUE//2, MAX_HUE//2], TARGET_HUE//2)
    
    return lastMin0*2, lastMin1*2, lastMax*2

SCALE = 8
H_BINS = 256//SCALE
hists = collections.deque(maxlen=15)

def svHist(hsv, mask, peakHue):
    start = perf_counter()
    
    # compute histogram
    hsv = hsv[mask > 128]
    weights = hsv[:,0] - peakHue
    weights = 1 / (0.004 + weights*weights)
    hist, _, _ = np.histogram2d(hsv[:,2], hsv[:,1], weights=weights, bins=H_BINS, range=[[0,255],[0,255]])
    hists.append(hist)
    
    # normalize
    hist = np.sum(hists, axis=0)
    cv2.GaussianBlur(hist, (3,3), 0.5, hist)
    hist = hist/hist.max() # normalize
    
    def tryThreshold(thresh):
        # create mask
        mask = cv2.inRange(hist, thresh, 999999)
        # cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (2,2)), mask)
        
        # find best contour
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            return None, 0, mask
        
        def contourScore(c):
            cMask = np.zeros((H_BINS,H_BINS), dtype=np.uint8)
            cv2.drawContours(cMask, [c], 0, 255, cv2.FILLED)
            integral = hist[cMask>128].sum()
            x,y,w,h = cv2.boundingRect(c)
            d = (x+w/2) + (y+h/2) # distance along diagonal
            return integral + d*0.5
        scores = [contourScore(c) for c in contours]
        bestIndex = np.argmax(scores)
        return contours[bestIndex], scores[bestIndex], mask
    
    contour, score, mask = tryThreshold(np.percentile(hist, 90))
    
    if args.debug_histograms:
        cv2.imshow("SV mask", cv2.resize(mask, (512,512), interpolation=cv2.INTER_NEAREST))
    
    # get bounding rect
    x,y,w,h = cv2.boundingRect(contour)
    
    end = perf_counter()
    if args.debug_timing: print(f"    svHist took {int((end-start)*1000)} ms")
    
    if args.debug_histograms:
        # display
        # hist = mask.astype(np.float)/255
        hist = cv2.resize((hist*255).astype(np.uint8), (512,512), interpolation=cv2.INTER_NEAREST)
        hist = cv2.cvtColor(hist, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(hist, (x*SCALE*2,y*SCALE*2), ((x+w)*SCALE*2,(y+h)*SCALE*2), (0,255,0))
        cv2.imshow("SV histogram", hist)
    
    return x*SCALE, (x+w)*SCALE, y*SCALE, (y+h)*SCALE

def autoSetColor(input):
    start = perf_counter()
    
    # convert to HSV
    hsv = cv2.cvtColor(input, cv2.COLOR_BGR2HSV)
    
    # get the hue range
    minH, maxH, peakHue = computeHueRange(hsv)
    
    # process/visualize S-V histogram
    mask = cv2.inRange(hsv, (minH, 40, 40), (maxH, 255, 255))
    # TODO: add hysteresis to this rectangle (like for the hue range)
    minS,maxS,minV,maxV = svHist(hsv, mask, peakHue)
    if args.debug_hue_mask:
        scaledMask = cv2.resize(mask, (0,0), fx=args.roi_scale, fy=args.roi_scale, interpolation=cv2.INTER_NEAREST)
        cv2.imshow("hue mask", scaledMask)
    
    # set color range
    global minColor, maxColor
    minColor[0] = (minH, minS, minV)
    maxColor[0] = (maxH, 255, 255)
    minColor[1] = minColor[0]
    maxColor[1] = maxColor[0]
    
    
    end = perf_counter()
    if args.debug_timing: print(f"autoSetColor took {int((end-start)*1000)} ms")



#########################################
############ angle filtering ############
#########################################

# constants (TODO: tune these)
MAX_SCALE_SPEED = 40.0 # maximum normal movement speed (degrees per second)
SMOOTH_HISTORY = 1.0 # amount of history to consider for smoothing (seconds)
SMOOTH_FIT_DEGREE = 2 # degree of polynomial fit for smoothing
STEADY_HISTORY = 1.0 # amount of history to consider for steadiness (seconds)
STEADY_THRESHOLD = 4.0 # angle variation considered "steady" (degrees)
MAX_SKEW = 3.0 #2.2 # maximum skew between the top & bottom lines (degrees)
TIPPED_THRESHOLD = 3.5 # angle at which the scale is "tipped" (degrees)
UNTIPPED_THRESHOLD = 2.7 # angle at which the scale is no longer "tipped" (degrees)

# ignore RankWarnings from np.polyfit
import warnings
warnings.simplefilter("ignore", np.RankWarning)

curAngle = 0
zeroPoint = 0
zeroed = False
lastUpdate = None
smoothHistory = collections.deque()
steadyHistory = collections.deque()
waitingForSteady = True

errorMsg = None

def isSteady():
    if len(steadyHistory) == 0: return False
    angles = [e[1] for e in steadyHistory]
    minA = min(angles)
    maxA = max(angles)
    return maxA - minA < STEADY_THRESHOLD

def updateAngle(lineAngles):
    global curAngle, zeroPoint, zeroed, lastUpdate, waitingForSteady, errorMsg
    errorMsg = None
    if not zeroed:
        errorMsg = "NOT ZEROED YET"
    
    # calculate dt
    now = perf_counter()
    if lastUpdate is None: lastUpdate = now
    dt = now - lastUpdate
    lastUpdate = now
    
    maxDelta = MAX_SCALE_SPEED*dt
    newAngle = None
    
    # history update functions
    def updateHistory(list, history, value):
        list.append((now, value))
        while now - list[0][0] > history:
            list.popleft()
    def updateSmoothHistory():
        updateHistory(smoothHistory, SMOOTH_HISTORY, curAngle)
    
    # use the the angles from the lines, if available
    if lineAngles is None:
        errorMsg = "LINES FAILED"
    else:
        a1, a2 = lineAngles
        if abs(a1 - a2) > MAX_SKEW:
            errorMsg = "SKEWED"
        else:
            newAngle = (a1 + a2) / 2
    
    # fall back to angles from centers of blobs, if necessary
    if newAngle is None:
        if errorMsg is None: errorMsg = "NO GOOD DATA"
        updateSmoothHistory()
        return
    
    delta = newAngle - curAngle
    
    # update history
    updateHistory(steadyHistory, STEADY_HISTORY, newAngle)
    
    # if it's moving too fast, stop updating until it's steady again
    if abs(delta) > maxDelta:
        waitingForSteady = True
    if waitingForSteady and not isSteady():
        errorMsg = "STEADYING"
        updateSmoothHistory()
        return
    waitingForSteady = False
    
    curAngle += min(max(delta, -maxDelta), +maxDelta)
    updateSmoothHistory()

def getRawAngle():
    return curAngle

def getAngle():
    if len(smoothHistory) == 0: return 0.0
    # do a polynomial fit on the history, putting more weight on recent data points
    weights = [x**0 for x in range(1, len(smoothHistory)+1)]
    fitFunc = np.poly1d(np.polyfit(*zip(*smoothHistory), SMOOTH_FIT_DEGREE, w=weights))
    lastTime = smoothHistory[-1][0]
    return fitFunc(lastTime) - zeroPoint

curTip = 0
def getTip():
    global curTip
    angle = getAngle()
    if abs(angle) < UNTIPPED_THRESHOLD: curTip = 0
    elif angle > +TIPPED_THRESHOLD: curTip = +1
    elif angle < -TIPPED_THRESHOLD: curTip = -1
    return curTip

def zeroAngle():
    global zeroPoint, zeroed
    angle = getRawAngle()
    if angle is not None:
        zeroPoint = angle
        zeroed = True



##########################################
######### command-line arguments #########
##########################################

parser = argparse.ArgumentParser(description="Program to track the scale arm using OpenCV. (by Quinn Tucker '18)")
parser.add_argument("-n", "--no-network", action="store_true", help="don't initialize/output to NetworkTables")
optGroup = parser.add_mutually_exclusive_group()
optGroup.add_argument("-d", "--device", type=int, default=0, metavar="ID",
                    help="device ID of the camera to use (default: %(default)s)")
optGroup.add_argument("-i", "--input-image", metavar="FILE", help="optional image to use instead of a live camera")
optGroup.add_argument("-v", "--input-video", metavar="FILE", help="optional video to use instead of a live camera")
parser.add_argument("-s", "--scale", type=float, default=1.0, metavar="FACTOR",
                      help="amount to up/downsample each frame (optional)")
parser.add_argument("-a", "--auto", action='store_true', default=False,
                    help="set hue value automatically")
parser.add_argument("--hue-shift", type=float, default=0.0, metavar="DEGREES",
                      help="amount to shift the hue of each frame (optional)")
parser.add_argument("--raw-scale", type=float, default=1.0, metavar="FACTOR",
                    help="amount to scale the raw frame display by (default: %(default)s)")
parser.add_argument("--roi-scale", type=float, default=2.0, metavar="FACTOR",
                    help="amount to scale the region-of-interest display by (default: %(default)s)")
parser.add_argument("--csv-output", type=argparse.FileType("w"), metavar="FILE",
                    help="optional file to write angle data to")
parser.add_argument("--ip", type=str, default='10.2.54.2', metavar="IP",
                    help="ip to connect to.")
debugGroup = parser.add_argument_group(title="debug flags")
debugGroup.add_argument("--debug-histograms", action="store_true", help="show the hue and saturation-value histograms")
debugGroup.add_argument("--debug-mask", action="store_true", help="show the thresholded color mask / debug masks")
debugGroup.add_argument("--debug-hue-mask", action="store_true", help="show the hue mask")
debugGroup.add_argument("--debug-timing", action="store_true", help="print how long various operations take")
args = parser.parse_args()



#########################################
########## robot communication ##########
#########################################

if args.no_network:
    print("Skipping NetworkTables initialization")
else:
    print("Using: " + args.ip)
    NetworkTables.initialize(server=args.ip)
    smartDashboard = NetworkTables.getTable("SmartDashboard")

#########################################
############### main code ###############
#########################################

minColor = [(0,0,0), (0,0,0)]
maxColor = [(0,0,0), (0,0,0)]
H_PAD = 10
S_PAD = 20
V_PAD = 20

roi = None
gotROI = False

pivotLoc = None
selectingPivot = False

frozen = False

global width, height
def onMouse_raw(event, x, y, flags, param):
    global roi, gotROI, width, height
    x = int(x/args.raw_scale)
    y = int(y/args.raw_scale)
    x = min(max(x, 0), width-1)
    y = min(max(y, 0), height-1)
    if event == cv2.EVENT_LBUTTONDOWN:
        roi = (x, y, x, y)
        gotROI = False
    elif event == cv2.EVENT_LBUTTONUP:
        if roi[0] > roi[2]: roi = (roi[2], roi[1], roi[0], roi[3])
        if roi[1] > roi[3]: roi = (roi[0], roi[3], roi[2], roi[1])
        if roi[0] < roi[2] and roi[1] < roi[3]:
            gotROI = True
    
    if flags & cv2.EVENT_FLAG_LBUTTON:
        roi = roi[:2] + (x, y)

def onMouse(event, x, y, flags, param):
    global curFrame, minColor, maxColor, selectingPivot, pivotLoc, pivotChanged
    h, w = curFrame.shape[:2]
    x = int(x/args.roi_scale)
    y = int(y/args.roi_scale)
    
    if event == cv2.EVENT_LBUTTONUP or event == cv2.EVENT_RBUTTONUP:
        selectingPivot = False
    
    leftDown  = flags & cv2.EVENT_FLAG_LBUTTON != 0
    rightDown = flags & cv2.EVENT_FLAG_RBUTTON != 0
    if not (leftDown or rightDown):
        return
    
    if selectingPivot:
        pivotLoc = (x, y)
        pivotChanged = True
        return
    
    ci = 0 if x < w/2 else 1
    
    if event == cv2.EVENT_RBUTTONDOWN:
        maxColor[ci] = (0,0,0)
    
    def addPixel(x, y):
        global curFrame, minColor, maxColor
        if x >= w or y >= h:
            return
        color = curFrame[y, x]
        newMinColor = (float(color[0])-H_PAD, float(color[1])-S_PAD, float(color[2])-V_PAD)
        newMaxColor = (float(color[0])+H_PAD, float(color[1])+S_PAD, float(color[2])+V_PAD)
        if maxColor[ci] == (0,0,0):
            minColor[ci] = newMinColor
            maxColor[ci] = newMaxColor
        elif leftDown or rightDown:
            minColor[ci] = tuple(map(min, minColor[ci], newMinColor))
            maxColor[ci] = tuple(map(max, maxColor[ci], newMaxColor))
    
    BRUSH_R = 4 # radius of brush
    for x2 in range(x-BRUSH_R, x+BRUSH_R+1):
        for y2 in range(y-BRUSH_R, y+BRUSH_R+1):
            addPixel(x2, y2)

def onKey(key):
    global minColor, maxColor, gotROI, exposure, selectingPivot
    print(f"key: {key}")
    if key == ord("c") or key == ord("C"):
        minColor[0], minColor[1] = (0,0,0), (0,0,0)
        maxColor[0], maxColor[1] = (0,0,0), (0,0,0)
    if key == ord("z") or key == ord("Z"):
        zeroAngle()
    if key == ord("a") or key == ord("A"):
        args.auto = True
    if key == ord("m") or key == ord("M"):
        args.auto = False
    if key == ord("r") or key == ord("R"):
        gotROI = False
    if key == ord("p") or key == ord("P"):
        selectingPivot = not selectingPivot
    if key == ord("o") or key == ord("O"):
        autoDetectPivot()
    if key == ord("d") or key == ord("D"):
        args.device += 1
        print(f"    switching device id to {args.device}")
        global cap
        cap = initCapture()
    if key == ord("s") or key == ord("S"):
        cap.set(cv2.CAP_PROP_SETTINGS, 1)
    if key == ord("f") or key == ord("F"):
        global frozen
        frozen = not frozen
    if key == ord("-") or key == ord("_"):
        exposure -= 1
        cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
    if key == ord("=") or key == ord("+"):
        exposure += 1
        cap.set(cv2.CAP_PROP_EXPOSURE, exposure)

######### VideoCapture #########
def initCapture():
    if args.input_image is not None: return None
    print("Initializing VideoCapture...")
    if args.input_video is not None:
        cap = cv2.VideoCapture(args.input_video)
    else:
        cap = cv2.VideoCapture(args.device)
        if not cap.isOpened():
            print(f"    failed to open camera device {args.device}")
            print(f"    resetting device id to 0")
            args.device = 0
            initCapture()
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    global exposure
    exposure = cap.get(cv2.CAP_PROP_EXPOSURE)
    cap.set(cv2.CAP_PROP_EXPOSURE, exposure) # disable auto exposure & white balance
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    print("    done.")
    return cap
global cap
cap = initCapture()

if args.input_image is not None:
    inputImage = cv2.imread(args.input_image)
    inputImage = cv2.resize(inputImage, (0,0), fx=args.scale, fy=args.scale)

global dt, fps
dt = fps = None
frameCount = 0
heartbeat = 0
lastSecond = perf_counter()
first_call = True

while True:
    # read the next frame and make sure it's valid
    if args.input_image is not None:
        frame = inputImage.copy()
    elif not frozen:
        ret, frame = cap.read()
        def isFrameOK():
            if not ret or frame is None:
                return False
            for i in [0,1,2]:
                if cv2.countNonZero(frame[:,:,i]) > 0:
                    return True
            return False
        if not isFrameOK():
            print("Got a bad frame, reinitializing.")
            cap.release()
            cap = initCapture() # reopen the VideoCapture
            continue
        if args.scale != 1.0:
            frame = cv2.resize(frame, (0,0), fx=args.scale, fy=args.scale)
    
    height, width = frame.shape[:2]
    
    if args.hue_shift != 0.0:
        start = perf_counter()
        
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        shift = (args.hue_shift//2) % 180
        if shift < 0: shift += 180
        if 180+shift > 255:
            tmp = frame[:, :, 0].astype(np.uint16) + np.uint16(shift)
            tmp[tmp > 180] -= 180
            frame[:, :, 0] = tmp
        else:
            frame[:, :, 0] += np.uint8(shift)
        frame = cv2.cvtColor(frame, cv2.COLOR_HSV2BGR)
        
        end = perf_counter()
        if args.debug_timing: print(f"hue shift took {int((end-start)*1000)} ms")
    
    # show the raw frame (with ROI rect)
    frameDisp = frame.copy()
    if args.raw_scale != 1.0:
        frameDisp = cv2.resize(frameDisp, (0,0), fx=args.raw_scale, fy=args.raw_scale)
    if roi is not None:
        sROI = tuple(int(v*args.raw_scale) for v in roi)
        cv2.rectangle(frameDisp, sROI[:2], sROI[2:], (0, 0, 255), 2)
    if not NetworkTables.isConnected():
        cv2.putText(frameDisp, "NetworkTables is not connected", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 1, cv2.LINE_AA)
    else:
      cv2.putText(frameDisp, "NetworkTables is connected", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 1, cv2.LINE_AA)

    if not gotROI:
        cv2.imshow("raw", frameDisp)
        cv2.setMouseCallback("raw", onMouse_raw)
    if first_call:
        cv2.moveWindow("raw", 0, 0)
        first_call = False
    
    if gotROI:
        start = perf_counter()
        frameROI = frame[roi[1]:roi[3], roi[0]:roi[2]]
        process(frameROI)
        end = perf_counter()
        dt = end - start
        
        frameCount += 1
        if perf_counter() - lastSecond > 1.0:
            lastSecond += 1.0
            fps = frameCount
            frameCount = 0
        
        cv2.setMouseCallback("raw", onMouse)
        
        # if args.csv_output is not None and dbg1 is not None:
        #     args.csv_output.write(f"{start}, {dbg1}, {dbg2}, {getTip()}\n")
    else:
        frameROI = None
    
    if not args.no_network:
        smartDashboard.putNumber("scaleAngle", getAngle())
        smartDashboard.putNumber("scaleTip", getTip())
        smartDashboard.putBoolean("scaleError", errorMsg is not None)
        smartDashboard.putNumber("scaleHeartbeat", heartbeat)
        heartbeat += 1
    
    key = cv2.waitKey(1) & 0xFF
    if key == 27:
        break
    elif key != 0xFF:
        onKey(key)

# cleanup VideoCapture, windows, and CSV output
if args.input_image is None:
    cap.release()
cv2.destroyAllWindows()
if args.csv_output is not None:
    args.csv_output.close()
