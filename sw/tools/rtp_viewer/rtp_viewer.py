import cv2
import sys

cap = cv2.VideoCapture("rtp_stream.sdp")

if not cap.isOpened():
    sys.exit("Can't open video stream")

while True:
    # Read a frame from the video capture
    ret, frame = cap.read()

    # Quit if frame could not be retrieved or 'q' is pressed
    if not ret or cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Show the image in a window
    cv2.imshow('rtp', frame)
