import numpy as np
import cv2
from cv_utils import *
from filterColor import createMask, transformFromBGR


# Detect the plants in the image using color histograms. Return a mask
#  (black/white image, where 0 indicates no plants and 255 indicates plants)
def classifyFoliage(image):
    foliage_mask = np.zeros(image.shape[0:2], np.uint8)
    # Create a mask that has 255 where there is part of a plant in the image
    #   and 0 everywhere else
    # BEGIN STUDENT CODE
    rValsDict = {
        'BGR': [[0,90], [39,255], [0,179]],
        'LAB': [[87,138], [3,120], [156,205]],
        'LUV': [[94,155], [78,113], [159,211]],
        'YUV':[[62,114], [90,118], [108,145]],
        'YCrCb': [[75,156] , [111,141], [66,113]],
        'HSV': [[23,90], [104,255], [39,200]]
    }
    color = 'BGR'
    image =cv2.blur(image, (40, 40))
    mask_bgr = createMask(image, rValsDict[color], color)

    color = 'HSV'
    transformed = transformFromBGR(image, color)
    mask_hsv = createMask(transformed, rValsDict[color], color)
    # END STUDENT CODE
    return mask_hsv 

# Return the bounding box of the measuring stick
# You can either use the image to find the stick or determine the corners by hand,
#   using some of the utilities we've provided
def findStick (image):
    boundingBox = np.array([[0,0]])
    # BEGIN STUDENT CODE
    top_left = [1960, 285]  #1950
    top_right = [1980, 285] #2055
    bottom_left = [1910, 2000] #1863
    bottom_right = [1930, 2000]
    
    for x in range(1880, 2020):
        for y in range(285, 2000):
            if is_inside(top_left, bottom_left, bottom_right, top_right,(x, y)):
                boundingBox=np.append(boundingBox, [[x, y]],axis=0)
    boundingBox=np.delete(boundingBox, [0,0],axis=0)
    # END STUDENT CODE
    return boundingBox

def is_inside(p1, p2, p3, p4, point):        
    a = (p2[0] - p1[0]) * (point[1] - p1[1]) - (p2[1] - p1[1]) * (point[0] - p1[0])
    b = (p3[0]- p2[0]) * (point[1] -p2[1]) - (p3[1] - p2[1]) * (point[0] - p2[0])
    c = (p4[0] - p3[0]) * (point[1] - p3[1]) - (p4[1] - p3[1]) * (point[0] - p3[0])
    d = (p1[0] - p4[0]) * (point[1] - p4[1]) - (p1[1]- p4[1]) * (point[0] - p4[0])
    return (a >= 0 and b >= 0 and c >= 0 and d >= 0) or (a <= 0 and b <= 0 and c <= 0 and d <= 0)
       

# Find the height of the tallest that occludes (is in front of) the measuring stick.
# Use the bounding box returned by "findStick" to create a mask, 


# Given the foliage mask (as would be returned from classifyFoliage), 
#   return the height in cms the tallest plant that crosses in front 
#   of the measuring stick. Return None if no foliage overlaps the stick
def measureHeight(image, foliage_mask):
    # First, use the bounding box returned from findStick to create a mask
    #   of the measuring stick
    stick_mask = np.zeros(foliage_mask.shape[0:2], np.uint8)
    # BEGIN STUDENT CODE
    c = findStick(image)
    lines=[2000,1850,1720,1555,1380,1200,1010,810,590,360]
    stick_mask=cv2.drawContours(stick_mask, [c], 0, 255, -1)
    # END STUDENT CODE
    # Find the maximum height of plants that overlap the measuring stick
    #   in the foliage_mask
    height = max_row = None
    # BEGIN STUDENT CODE
    combined_mask = stick_mask & foliage_mask 
    # createWindow()
    # showImage('image',combined_mask)
    # cv2.waitKey(0)
    max_row = np.unravel_index(combined_mask.argmax(), combined_mask.shape)[0]
    for index, element in enumerate(lines):
        if(max_row>=element and index != 0):
            height = ((lines[index-1]-max_row)/(lines[index-1]-element))+index-1
            # print(height, max_row)
            return (height, max_row)  
        elif max_row < element and index == 9 and max_row!=0:
            return (10, max_row)   
    return (None, None)
    # END STUDENT CODE

# Use the color calibration squares to find a transformation that will
#   color-correct the image such that the mean values of the calibration
#   squares are the given "goal" colors.
# Return the color-corrected image
def colorCorrect(image, blue_goal, green_goal, red_goal):
    # Find a transform c' = T c, c is the pixel value in the image,
    #   c' is the transformed pixel, and T is the 3x3 transformation matrix
    # Do this by solving d = A x, as per the lecture notes.
    # Note that while the lecture notes describe an affine (3x4) transform,
    #  here we have only 3 colors, so it has to be a Euclidean (3x3) tranform
    # BEGIN STUDENT CODE
    import os
    if os.name == 'nt':
        Bmask = cv2.imread('masks\\blue_mask.jpg', 0)
        Gmask = cv2.imread('masks\\green_mask.jpg', 0)
        Rmask = cv2.imread('masks\\red_mask.jpg', 0)
    else:
        Bmask = cv2.imread('masks/blue_mask.jpg', 0)
        Gmask = cv2.imread('masks/green_mask.jpg', 0)
        Rmask = cv2.imread('masks/red_mask.jpg', 0)
        
    Bbox_mean = cv2.mean(image, mask=Bmask)[:3]
    Gbox_mean = cv2.mean(image, mask=Gmask)[:3]
    Rbox_mean = cv2.mean(image, mask=Rmask)[:3]
    
    #print(Bbox_mean)
    #print(Gbox_mean)
    #print(Rbox_mean)
    # END STUDENT CODE

    # Fill in the rows of the "A" matrix, according to the notes
    A = np.zeros((9, 9), np.float64)
    # BEGIN STUDENT CODE
    A[0][0] = A[1][3] = A[2][6] = Bbox_mean[0]
    A[0][1] = A[1][4] = A[2][7] = Bbox_mean[1]
    A[0][2] = A[1][5] = A[2][8] = Bbox_mean[2]
    A[3][0] = A[4][3] = A[5][6] = Gbox_mean[0]
    A[3][1] = A[4][4] = A[5][7] = Gbox_mean[1]
    A[3][2] = A[4][5] = A[5][8] = Gbox_mean[2]
    A[6][0] = A[7][3] = A[8][6] = Rbox_mean[0]
    A[6][1] = A[7][4] = A[8][7] = Rbox_mean[1]
    A[6][2] = A[7][5] = A[8][8] = Rbox_mean[2]
    # END STUDENT CODE

    # Fill in the "d" vector with the "goal" colors 
    d = np.zeros((1,9))
    # BEGIN STUDENT CODE
    d[0][0] = blue_goal[0]
    d[0][1] = blue_goal[1]
    d[0][2] = blue_goal[2]
    d[0][3] = green_goal[0]
    d[0][4] = green_goal[1]
    d[0][5] = green_goal[2]
    d[0][6] = red_goal[0]
    d[0][7] = red_goal[1]
    d[0][8] = red_goal[2]
    # END STUDENT CODE

    x = np.matmul(np.matmul(np.linalg.pinv(np.matmul(A.T, A)), A.T), d.T)
    T = x.reshape((3,3))

    # Apply the transform to the pixels of the image and return the
    #  new corrected_image
    corrected_image = image.copy()
    # BEGIN STUDENT CODE
    """
    rows, cols, channels = image.shape
    image_reshaped = image.reshape((rows * cols, channels))
    transformed_pixels = np.dot(T, image_reshaped.T).T
    transformed_pixels = np.clip(transformed_pixels, 0, 255)
    corrected_image = transformed_pixels.reshape((rows, cols, channels)).astype(np.uint8)
    """
    
    corrected_image = cv2.transform(corrected_image, T)
    corrected_image = np.clip(corrected_image, 0, 255)
    """
    for i in range(len(corrected_image)):
        for j in range(len(corrected_image[0])):
            corrected_image[i][j] = np.matmul(T, corrected_image[i][j])
        if i%100==0:
            print(i)
    """
    # END STUDENT CODE
    return corrected_image

# Given an image, return three values:
# 1. An image with all non-foliage parts masked out
# 2. The original image with (a) an outline of the stick (from findStick) and
#    (b) a line at the height of the foliage (from measureHeight), if any
# 3. The height of the foliage
def foliageImages (image):
    foliage_mask = classifyFoliage(image)
    height, row = measureHeight(image, foliage_mask)
    foliageImage = None
    # BEGIN STUDENT CODE
    foliageImage=cv2.bitwise_and(image,image, mask= foliage_mask)
    c = findStick(image)
    if(row!=None and row !=0):
        line = []
        for point in c:
            if(point[1]==row):
                line.append(point[0])
        image=cv2.line(image, (line[0],row),(line[-1],row),(0,0,255),20)
    pts = np.array([[1950, 285],[2050, 285],[1940, 2000],[1880, 2000]], np.int32)
    pts = pts.reshape((-1,1,2))
    image=cv2.polylines(image,[pts],True,(255,0,0),10)
    # END STUDENT CODE
    return foliageImage, image, height

# Given an image, return two values:
# 1. The amount of foliage in the image
# 2. An estimate of the plant health (as a string), based on changes from the
#    previous day (changes in both the amount of foliage and plant height).
preheight=0.0001
prefoliage=0.003
def plantHealth (image):
    foliage_mask = classifyFoliage(image)
    height, row = measureHeight(image, foliage_mask)
    greenery = 0
    health_msg = ""
    # BEGIN STUDENT CODE
    global preheight
    global prefoliage
    green_pixels = cv2.countNonZero(foliage_mask)
    total_pixels = image.shape[0] * image.shape[1]
    greenery = green_pixels/total_pixels
    if(height==None):
        height=0
    if greenery-prefoliage>0.01 or height/preheight>1:
       health_msg="good"
    elif greenery/prefoliage>=1:
       health_msg="OK"
    else:
       health_msg="bad"
    preheight=height
    prefoliage=greenery
    # END STUDENT CODE
    return greenery, health_msg

