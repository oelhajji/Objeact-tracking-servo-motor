import numpy as np
import serial
import time
import math
from picamera2 import Picamera2, Preview
import cv2

# Configure the serial connection with the Arduino
# '/dev/ttyACM0' is the typical port for Arduino on Raspberry Pi; adjust if needed.
# 9600 is the baud rate, and timeout specifies the read timeout period.
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)  # Wait for the serial connection to initialize

# Initialize the Raspberry Pi Camera
picam2 = Picamera2()
# Set the preview configuration with the desired image format
picam2.preview_configuration.main.format = "XRGB8888"  
picam2.configure("preview")  # Configure the camera for preview
picam2.start()
time.sleep(5)  # Allow the camera to stabilize

# Set the target centroid coordinates (center of the camera frame)
center_x, center_y = 360, 240  # Assuming 720x480 resolution

# Define motor control parameters
max_speed = 255  # Maximum PWM motor speed
max_turn = 100  # Maximum turn adjustment
kp_translation = 0.01  # Proportional gain for forward/backward control
kp_rotation = 0.1  # Proportional gain for rotational control

# Control loop parameters
tf, Hz = 3, 10  # Total time (3 seconds) and frequency (10 Hz)
dt = 1 / Hz
time_elapsed = 0

# Servo motor angle constraints
max_angle = 179  # Maximum servo angle
min_angle = 0    # Minimum servo angle
angle_1 = 90  # Initial angle of the servo motor

# Function to define HSV color range for detection
# h_min/h_max: Hue range
# s_min/s_max: Saturation range
# v_min/v_max: Value range
def define_color_range(h_min, h_max, s_min, s_max, v_min, v_max):
    lower_bound = np.array([h_min, s_min, v_min])
    upper_bound = np.array([h_max, s_max, v_max])
    return lower_bound, upper_bound

# Placeholder function for trackbars (if used later for dynamic adjustments)
def nothing(x):
    pass

# Allow the camera to warm up
time.sleep(2)

# Start the main loop for object tracking
while True:
    # Capture a frame from the PiCamera
    frame = picam2.capture_array()

    # Define the HSV color range for object detection (example values)
    lower_color, upper_color = define_color_range(106, 179, 94, 255, 0, 255)

    # Convert the captured frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a binary mask for the specified color range
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # Apply morphological operations to reduce noise
    kernel = np.ones((5, 5), np.uint8)  # Define a kernel for erosion/dilation
    mask = cv2.erode(mask, kernel, iterations=1)  # Remove noise
    mask = cv2.dilate(mask, kernel, iterations=1)  # Fill gaps

    # Find contours in the binary mask
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        # Process only if the contour area is significant
        if cv2.contourArea(contour) > 500:
            # Compute the bounding box for the detected contour
            x, y, w, h = cv2.boundingRect(contour)

            # Draw a small circle at the center of the bounding box
            cv2.circle(frame, (int(x + w / 2), int(y + h / 2)), 3, (0, 255, 0), 1)
            # Add text displaying the centroid coordinates
            cv2.putText(frame, f"{int(x + w / 2), int(y + h / 2)}", 
                        (int(x + 10 + w / 2), int(y + 10 + h / 2)), 2, 2, (0, 255, 0), 2)
            # Draw a rectangle around the detected object
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Calculate the centroid of the detected object
    centroid_x = x + w / 2
    centroid_y = y + h / 2

    # Compute the error between the object's centroid and the desired center position
    error_x = centroid_x - center_x  # Horizontal error
    error_y = centroid_y - center_y  # Vertical error

    # Compute control signals for the motors
    forward_speed = kp_translation * error_y  # Adjust speed based on y-axis error
    turn_speed = kp_rotation * error_x        # Adjust turning speed based on x-axis error

    # Convert speed adjustments into servo motor angles
    forward_angle = angle_1 - forward_speed * 0.5
    turn_angle = angle_1 - turn_speed * 0.5
    angle_1 = turn_angle  # Update angle

    # Send computed angles to the Arduino
    arduino.write(chr(int(forward_angle)).encode())
    time.sleep(0.2)
    arduino.write(chr(int(turn_angle)).encode())

    # Print centroid information and control signals for debugging
    print(f"Centroid: ({centroid_x:.2f}, {centroid_y:.2f})")
    print(f"Forward Speed: {forward_speed}, Forward Angle: {forward_angle}")

    # Display the processed frames
    cv2.circle(frame, (int(center_x), int(center_y)), 3, (0, 255, 0), 1)  # Mark the target center
    cv2.imshow('Object Tracking', frame)  # Display the annotated frame
    cv2.imshow('Mask', mask)  # Display the binary mask

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Close the serial connection to the Arduino
arduino.close()

# Release camera resources and close all OpenCV windows
camera.close()
cv2.destroyAllWindows()
