import numpy as np
import cv2

win_max_dimensions = {}
win_scale = {}

def imageScale(wname): return win_scale[wname]

def print_clicks(event, x, y, flags, wname):
    if (event == cv2.EVENT_LBUTTONDOWN):
        scale = imageScale(wname)
        print("Clicked at: %d %d (%d %d)" %(x, y, int(x*scale), int(y*scale)))

def createWindow(wname='image', width=500, height=500, x0=0, y0=0,
                 mouse_callback=print_clicks, callback_data=None):
    global win_max_dimensions

    cv2.namedWindow(wname, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(wname, width, height)
    win_max_dimensions[wname] = float(max(width, height))
    cv2.moveWindow(wname, x0, y0)
    cv2.setMouseCallback(wname, mouse_callback, (callback_data or wname))

# Reduce max dimension to 600, o/w display is really lagging
def _showImage(wname, image):
    global win_scale

    scale = max(image.shape[0]/win_max_dimensions[wname],
                image.shape[1]/win_max_dimensions[wname])
    win_scale[wname] = scale
    disp_img = cv2.resize(image, (int(image.shape[1]/scale),
                                  int(image.shape[0]/scale)))
    cv2.imshow(wname, disp_img)

def showImage(wname, image):
    _showImage(wname, image)
    cv2.waitKey(1)

def showImageWait(wname, image, wait_time=-1):
    _showImage(wname, image)
    return cv2.waitKey(wait_time) & 0xFF

def readImage(filename):
    image = cv2.imread(filename)
    if (not isinstance(image, np.ndarray)):
        raise Exception("File not found: %s" %filename)
    return image

def writeImage(filename, image):
    status = cv2.imwrite(filename, image)
    if (not status):
        raise Exception("File not written: %s" %filename)
    return status

def readMask(filename):
    mask = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
    if (not isinstance(mask, np.ndarray)):
        raise Exception("File not found: %s" %filename)
    # Apparently, some values get read in not 0 or 255
    mask[mask > 0] = 255 
    return mask

def writeMask(filename, mask):
    status = cv2.imwrite(filename, mask)
    if (not status):
        raise Exception("File not written: %s" %filename)
    return status
