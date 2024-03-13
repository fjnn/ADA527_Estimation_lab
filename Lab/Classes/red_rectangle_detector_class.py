import cv2
import numpy as np

class RedRectangle:
    
    def __init__(self) -> None:
        self.contours = None
        self.middle_point = (0, 0)
        self.height = 130 #mm
        self.width = 10 #mm
        self.measured_distance = 205 #mm
        self.initial_width_pixels = 24
        self.measured_width_pixels = 0
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
        max_contour = None

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

        # for contour in contours:

            ## 2) Parallelogram -- thats what we need
            # Fit a rotated rectangle to the contour
            rect = cv2.minAreaRect(max_contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            # Draw the rotated rectangle
            cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)

            # Fin the center
            x, y, w, h = cv2.boundingRect(max_contour)
            # Get the endpoints of the bounding rectangle
            top_left = (x, y)
            top_right = (x + w, y)
            bottom_left = (x, y + h)
            bottom_right = (x + w, y + h)

            # Calculate the distance between the top and bottom sides
            self.measured_width_pixels = np.sqrt((bottom_right[0] - bottom_left[0])**2 + (bottom_right[1] - bottom_left[1])**2)
            # self.measured_width_pixels = np.sqrt((bottom_right[0] - bottom_left[0])**2 + (bottom_right[1] - bottom_left[1])**2)
    
            # Calculate the middle point of the rectangle
            middle_x = x + w // 2
            middle_y = y + h // 2
            self.middle_point = (middle_x, middle_y)
            
            # Draw origin and mouse position on the frame
            cv2.circle(frame, self.middle_point, 5, (255, 0, 0), 3)

        return frame
    
    def get_com_pixels(self):
        return self.middle_point
    
    def get_stick_width_in_pixels(self):
        return self.measured_width_pixels
    
    # distance estimation function 
    def distance_finder(self, face_width_in_frame):  
        # distance = (self.width * self.focal_length)/face_width_in_frame 
        distance = (self.width * self.focal_length)/face_width_in_frame
        return distance 