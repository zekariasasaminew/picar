import time
import numpy as np
import picar_4wd as fc
from picamera2 import Picamera2, Preview
import cv2

# Initialize PiCar motors and sensors from picar_4wd
left_front = fc.Motor(fc.PWM("P13"), fc.Pin("D4"), is_reversed=False)
right_front = fc.Motor(fc.PWM("P12"), fc.Pin("D5"), is_reversed=False)
left_rear = fc.Motor(fc.PWM("P8"), fc.Pin("D11"), is_reversed=False)
right_rear = fc.Motor(fc.PWM("P9"), fc.Pin("D15"), is_reversed=False)

# Grayscale sensors initialization
gs0 = fc.ADC('A5')
gs1 = fc.ADC('A6')
gs2 = fc.ADC('A7')

# Function to get grayscale values
def get_grayscale_list():
    adc_value_list = [gs0.read(), gs1.read(), gs2.read()]
    return adc_value_list

# Function to detect line status (center, left, right)
def get_line_status(ref, fl_list):
    if fl_list[1] <= ref:
        return 0  # Center
    if fl_list[0] <= ref:
        return -1  # Left
    elif fl_list[2] <= ref:
        return 1  # Right
    return None

# Initialize PiCamera2
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration()
picam2.configure(camera_config)
picam2.start_preview(Preview.QTGL)
picam2.start()

# Function to capture frame from PiCamera2 and process it for lane detection
def capture_frame():
    # Capture a single frame
    frame = picam2.capture_array()

    # Convert the frame to grayscale for lane detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    return binary

# Main function to follow the lane
def follow_lane():
    fc.forward(.5)  # Start moving forward at speed 
    
    while True:
        # Capture frame from the PiCamera2
        frame = capture_frame()

        # Get the grayscale sensor values
        gs_list = get_grayscale_list()
        line_status = get_line_status(1000, gs_list)  # 170 as a reference threshold for grayscale sensors

        # Check if the robot is off the paper (grayscale value < 1400)
        if any(val < 1200 for val in gs_list):
            # The robot has gone off the A4 paper. Turn left or right based on the current line status
            if line_status == -1:
                fc.turn_right(.5)  # Turn right if it was heading left
            elif line_status == 1:
                fc.turn_left(.5)  # Turn left if it was heading right
            else:
                fc.turn_left(.5)  # Default to turning left if no clear direction

        else:
            # Normal lane following logic
            if line_status == -1:
                fc.turn_left(.5)  # Turn left if it's off-center to the left
            elif line_status == 1:
                fc.turn_right(.5)  # Turn right if it's off-center to the right
            else:
                fc.forward(.5)  # Move forward if centered on the line

        # Display the processed frame for debugging purposes (optional)
        cv2.imshow("Lane Detection", frame)

        # Stop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    picam2.stop()
    cv2.destroyAllWindows()

# Start lane following
follow_lane()
