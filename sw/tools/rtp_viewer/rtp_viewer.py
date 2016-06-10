#! /usr/bin/python

import cv2
import sys

from os import path, getenv

PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_SRC + "/sw/ext/pprzlink/lib/v1.0/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

mouse_begin = None
mouse_current = None

def on_mouse(event, x, y, flags, param):
    global mouse_begin, mouse_current, frame

    if event == cv2.EVENT_LBUTTONDOWN:
        mouse_begin = (x, y)

    if event == cv2.EVENT_MOUSEMOVE:
        mouse_current = (x, y)

    if event == cv2.EVENT_LBUTTONUP:
        if not mouse_begin:
            return

        msg = PprzMessage("datalink", "VIDEO_ROI")

        msg['ac_id'] = None
        msg['startx'] = mouse_begin[0]
        msg['starty'] = mouse_begin[1]
        msg['width'] = abs(x - mouse_begin[0])
        msg['height'] = abs(y - mouse_begin[1])
        msg['downsized_width'] = frame.shape[1]

        ivy_interface.send_raw_datalink(msg)

        mouse_begin = None

cap = cv2.VideoCapture("rtp_viewer.sdp")

if not cap.isOpened():
    sys.exit("Can't open video stream")

cv2.namedWindow('rtp')
cv2.setMouseCallback('rtp', on_mouse)

ivy_interface = IvyMessagesInterface("RTPviewer", start_ivy=False)
ivy_interface.start()

while True:
    # Read a frame from the video capture
    ret, frame = cap.read()

    # Quit if frame could not be retrieved or 'q' is pressed
    if not ret or cv2.waitKey(1) & 0xFF == ord('q'):
        break

    if mouse_begin:
        cv2.rectangle(frame, mouse_begin, mouse_current, (0, 255, 0))

    # Show the image in a window
    cv2.imshow('rtp', frame)

ivy_interface.shutdown()