"""
https://answers.opencv.org/question/134248/how-to-define-the-lower-and-upper-range-of-a-color/
"""

import cv2
import numpy as np

file_path = r'C:\Users\gizem\Desktop\output_imagee.jpg'
image_hsv = None   # global ;(
pixel = (20,60,80) # some stupid default

# mouse callback function
def pick_color(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDOWN:
        pixel = image_hsv[y,x]

        #you might want to adjust the ranges(+-10, etc):
        upper =  np.array([pixel[0] + 10, pixel[1] + 50, pixel[2] + 50])
        lower =  np.array([pixel[0] - 10, pixel[1] - 50, pixel[2] - 50])
        print(pixel, lower, upper)
        print(pixel, "[",lower[0],",",lower[1],",", lower[2],"],", "[",upper[0],",",upper[1],",",upper[2],"]")

        image_mask = cv2.inRange(image_hsv,lower,upper)
        cv2.imshow("mask",image_mask)

def main():
    import sys
    global image_hsv, pixel # so we can use it in mouse callback

    image_src = cv2.imread(file_path)  # pick.py my.png
    if image_src is None:
        print ("the image read is None............")
        return
    cv2.imshow("bgr",image_src)

    ## NEW ##
    cv2.namedWindow('hsv')
    cv2.setMouseCallback('hsv', pick_color)

    # now click into the hsv img , and look at values:
    image_hsv = cv2.cvtColor(image_src,cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv",image_hsv)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__=='__main__':
    main()