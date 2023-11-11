#!/usr/bin/env python
import numpy as np
import cv2, sys
from cv_utils import *
import argparse

wait_time = 100

def noop(dummy): pass

class Filter:
    # Range is (name, min, max, min0, max0) (last 2 are optional)
    ranges = None
    transform_type = None

    def __init__(self, range1, range2, range3, transform_type=None):
        self.transform_type = transform_type
        self.ranges = []
        self.ranges.append(range1)
        self.ranges.append(range2)
        self.ranges.append(range3)
        for i in range(3):
            if (len(self.ranges[i]) == 3):
                self.ranges[i] += self.ranges[i][1:3]

    def transform (self, image):
        return (image if not self.transform_type else
                cv2.cvtColor(image, self.transform_type))

filtersG = {'BGR' : Filter(['B', 0, 255], ['G', 0, 255], ['R', 0, 255]),
            'HSV' : Filter(['H', 0, 200, 0, 179], ['S', 0, 255], ['V', 0, 255],
                           cv2.COLOR_BGR2HSV),
            'LAB' : Filter(['L', 0, 255], ['A', 0, 255], ['B', 0, 255],
                           cv2.COLOR_BGR2LAB),
            'LUV' : Filter(['L', 0, 255], ['U', 0, 255], ['V', 0, 255],
                           cv2.COLOR_BGR2LUV),
            'XYZ' : Filter(['X', 0, 255], ['Y', 0, 255], ['Z', 0, 255],
                           cv2.COLOR_BGR2XYZ),
            'YCrCb' : Filter(['Y', 16, 235], ['Cr', 16, 250], ['Cb', 16, 240],
                           cv2.COLOR_BGR2YCrCb),
            'YUV' : Filter(['Y', 0, 255], ['U', 0, 255], ['V', 0, 255],
                           cv2.COLOR_BGR2YUV),
            }

# Transform a BGR image into another image, depending on the color_space
def transformFromBGR(image, color_space):
    if (not color_space in filtersG.keys()):
        print("ERROR: Unknown filter: %s" %color_space)
        exit()
    return (image if color_space == 'BGR' else
            cv2.cvtColor(image, filtersG[color_space].transform_type))

# Interactive GUI to set color space parameters to mask out certain portions
def filterImage(image, color_space='BGR', width=500, height=500):
    global filtersG

    wname = color_space + ' filter'
    createWindow(wname, width, height)

    if (not color_space in filtersG.keys()):
        print("ERROR: Unknown filter: %s" %color_space)
        exit()

    filter = filtersG[color_space]
    suffixes = ['_min', '_max']
    for r in range(3):
        name = filter.ranges[r][0]
        for i in range(2):
            cv2.createTrackbar(name+suffixes[i], wname,
                           filter.ranges[r][1], filter.ranges[r][2], noop)
            cv2.setTrackbarPos(name+suffixes[i], wname, filter.ranges[r][3+i])

    rVals = [[0,0] for _ in range(3)]
    for i in range(3):
        rVals[i] = filter.ranges[i][3:5]
    pVals = [[0,0] for _ in range(3)]

    trans_image = filter.transform(image)

    while True:
        for r in range(3):
            name = filter.ranges[r][0]
            for i in range(2):
                rVals[r][i] = cv2.getTrackbarPos(name+suffixes[i], wname)

        if (np.sum(np.abs(np.subtract(rVals, pVals))) > 0):
            for r in range(3):
                for i in range(2): pVals[r][i] = rVals[r][i]

            mask = createMask(trans_image, rVals, color_space)
            output = cv2.bitwise_and(image, image, mask=mask)
            showImage(wname, output)

        if (cv2.waitKey(wait_time) & 0xFF == ord('q')):
            cv2.destroyWindow(wname)
            return (rVals, mask)

# Text-based approach to set color space parameters
def filterImageText(image, color_space='BGR', width=500, height=500):
    global filtersG

    wname = color_space + ' filter'
    createWindow(wname, width, height)

    if (not color_space in filtersG.keys()):
        print("ERROR: Unknown filter: %s" %color_space)
        exit()

    filter = filtersG[color_space]
    rVals = [range[3:5] for range in filter.ranges]
    channels = [range[0] for range in filter.ranges]
    trans_image = filter.transform(image)
    mask = createMask(trans_image, rVals, color_space)
    showImageWait(wname, trans_image, 300)

    print("Available color channels: %s" %channels)
    while True:
        # Get input: <channel> <min> <max>
        print("Input: ", end='', flush=True)
        input = sys.stdin.readline().strip().capitalize().split(' ')
        if (input[0] == 'Q'):
            return (rVals, mask)
        elif (len(input) != 3): 
            print("Illegal input", input)
        elif (input[0] not in channels):
            print("Unknown channel for color space:", input[0])
        else:
            r = channels.index(input[0])
            rVals[r] = [max(int(input[1]), filter.ranges[r][1]),
                        min(int(input[2]), filter.ranges[r][2])]
            print("rVals", rVals)
            mask = createMask(trans_image, rVals, color_space)
            output = cv2.bitwise_and(image, image, mask=mask)
            showImage(wname, output)

# Create a single channel mask, with same dimensions as 'image'.
# 'rVals' is a list of pais, where each pair is the lower/upper bound
# on one of the channels of the color_space
def createMask(image, rVals, color_space):
    if (color_space == 'HSV'):
        mask = createMaskHSV(image, rVals)
    else:
        lower = np.array([v[0] for v in rVals])
        upper = np.array([v[1] for v in rVals])
        mask = cv2.inRange(image, lower, upper)
    return mask

# Special for HSV - allows for wrapping around color circle
#  (anything above 179 wraps to 0)
# Is called by createMask, so no need to call it on its own
def createMaskHSV(hsv, rVals):
    hMin = rVals[0][0]; hMax = rVals[0][1]
    sMin = rVals[1][0]; sMax = rVals[1][1]
    vMin = rVals[2][0]; vMax = rVals[2][1]

    lower = np.array([min(hMin, 179), sMin, vMin])
    upper = np.array([min(hMax, 179), sMax, vMax])
    mask = cv2.inRange(hsv, lower, upper)

    if (hMin > 179 or hMax > 179): # Wrapping around color circle
        lower = np.array([(hMin-179), sMin, vMin])
        upper = np.array([(hMax-179), sMax, vMax])
        mask2 = cv2.inRange(hsv, lower, upper)
        mask = cv2.add(mask, mask2)

    return mask

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = "filter color")
    parser.add_argument('image', help="image file")
    parser.add_argument('-f', '--filter', help="filter type (e.g., 'HSV', 'LAB'",
                        default='BGR')
    parser.add_argument('-n', '--nosliders', action='store_true',
                        help='use text-based interaction')
    args = parser.parse_args()

    image = readImage(args.image)
    filter, mask = (filterImageText(image, args.filter) if args.nosliders else
                    filterImage(image, args.filter, 750, 750))
    print(filter)
