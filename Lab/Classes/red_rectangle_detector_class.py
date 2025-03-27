import cv2
import numpy as np

class RedRectangle:
    
    def __init__(self) -> None:
        self.contours = None
        self.middle_point = (0, 0)
        self.height = 130 #mm
        self.width = 10 #mm
        self.measured_distance = 205 #mm
        self.initial_width_pixels = 39
        self.measured_width_pixels = 0
        self.measured_height_pixels = 0
        self.focal_length = (self.initial_width_pixels * self.measured_distance) / self.width 
	

    # TODO: HSV values as input?
    def detect_red_stick(self, frame):
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define range of red color in HSV
        hsv_vals = [[ 161 , 163 , 79 ], [ 181 , 263 , 179 ]]
        lower_red = np.array(hsv_vals[0])
        upper_red = np.array(hsv_vals[1])

        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Apply morphological operations to remove noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        max_area = -1
        max_contour = []

        # Iterate through the contours
        for contour in contours:

            ## 1) Only vertical rectangle
            # # Get the bounding rectangle
            # x, y, w, h = cv2.boundingRect(contour)
            # aspect_ratio = w / float(h)

            # # Filter out rectangles that are not vertical enough
            # if 0.01 < aspect_ratio and h > 150:
            #     # Draw the bounding rectangle around the stick
            #     cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            #     print("w:", w, "    h:", h)

            # Calculate the area of the contour
            area = cv2.contourArea(contour)
            
            # Check if the current contour has a larger area than the maximum found so far
            if area > max_area:
                max_area = area
                max_contour = contour

            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            # Draw the rotated rectangle
            # cv2.drawContours(frame, [box], 0, (255, 255, 0), 2)

        # for contour in contours:

            ## 2) Parallelogram -- thats what we need
            # Fit a rotated rectangle to the contour
        if len(max_contour) > 0:
            rect = cv2.minAreaRect(max_contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            self.measured_width_pixels = abs(min(box[:,0]) - max(box[:,0]))
            self.measured_height_pixels = abs(min(box[:,1]) - max(box[:,1]))

            # Draw the rotated rectangle
            cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)

            # Fin the center
            x, y, w, h = cv2.boundingRect(max_contour)
            
            # Calculate the middle point of the rectangle using x,y,w and h.
            middle_x = x + w // 2
            middle_y = y + h // 2
            self.middle_point = (middle_x, middle_y)
            
            # Draw origin and mouse position on the frame
            cv2.circle(frame, self.middle_point, 5, (255, 0, 0), 3)
            # cv2.circle(frame, box[0], 5, (255, 0, 0), 2) # bottom_left
            # cv2.circle(frame, box[1], 5, (0, 255, 0), 4) # top_left
            # cv2.circle(frame, box[2], 5, (255, 255, 0), 6) # top_right
            # cv2.circle(frame, box[3], 5, (255, 0, 255), 8) # bottom_right

        return frame
    
    def get_com_pixels(self):
        return self.middle_point
    
    def get_stick_width_in_pixels(self):
        return self.measured_width_pixels    
    
    def get_stick_height_in_pixels(self):
        return self.measured_height_pixels
    
    # distance estimation function 
    def distance_finder(self, face_width_in_frame):  
        # distance = (self.width * self.focal_length)/face_width_in_frame 
        distance = (self.width * self.focal_length)/face_width_in_frame
        return distance 