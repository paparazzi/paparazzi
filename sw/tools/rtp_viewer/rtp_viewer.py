#! /usr/bin/python

import cv2
import sys
from os import path, getenv

PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../')))
sys.path.append(PPRZ_SRC + "/sw/ext/pprzlink/lib/v1.0/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage


class RtpViewer:
    frame = None
    mouse = dict()

    def __init__(self, src):
        # Create the video capture device
        self.cap = cv2.VideoCapture(src)

        # Start the ivy interface
        self.ivy = IvyMessagesInterface("RTPviewer", start_ivy=False)
        self.ivy.start()

        # Create a named window and add a mouse callback
        cv2.namedWindow('rtp')
        cv2.setMouseCallback('rtp', self.on_mouse)

    def run(self):
        # Start an 'infinite' loop
        while True:
            # Read a frame from the video capture
            ret, self.frame = self.cap.read()

            # Quit if frame could not be retrieved or 'q' is pressed
            if not ret or cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Run the computer vision function
            self.cv()

    def cv(self):
        # If a selection is happening
        if self.mouse.get('start'):
            # Draw a rectangle indicating the region of interest
            cv2.rectangle(self.frame, self.mouse['start'], self.mouse['now'], (0, 255, 0), 2)

        # Show the image in a window
        cv2.imshow('rtp', self.frame)

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.mouse['start'] = (x, y)

        if event == cv2.EVENT_RBUTTONDOWN:
            self.mouse['start'] = None

        if event == cv2.EVENT_MOUSEMOVE:
            self.mouse['now'] = (x, y)

        if event == cv2.EVENT_LBUTTONUP:
            # If mouse start is defined, a region has been selected
            if not self.mouse.get('start'):
                return

            # Obtain mouse start coordinates
            sx, sy = self.mouse['start']

            # Create a new message
            msg = PprzMessage("datalink", "VIDEO_ROI")
            msg['ac_id'] = None
            msg['startx'] = sx
            msg['starty'] = sy
            msg['width'] = abs(x - sx)
            msg['height'] = abs(y - sy)
            msg['downsized_width'] = self.frame.shape[1]

            # Send message via the ivy interface
            self.ivy.send_raw_datalink(msg)

            # Reset mouse start
            self.mouse['start'] = None

    def cleanup(self):
        # Shutdown ivy interface
        self.ivy.shutdown()


if __name__ == '__main__':
    viewer = RtpViewer("rtp_viewer.sdp")

    if not viewer.cap.isOpened():
        viewer.cleanup()
        sys.exit("Can't open video stream")

    viewer.run()
    viewer.cleanup()

