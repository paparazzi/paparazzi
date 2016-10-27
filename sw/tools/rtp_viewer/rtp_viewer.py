#! /usr/bin/python

import cv2
import sys
import getopt
import re
from os import path, getenv

PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../')))
sys.path.append(PPRZ_SRC + "/sw/ext/pprzlink/lib/v1.0/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

def Usage(scmd):
    lpathitem = scmd.split('/')
    fmt = '''Usage: %s [-h | --help] [-p PORT | --port=PORT] [-s SCALE | --scale=SCALE] [-r ROTATE | --rotate=ROTATE] []
where
\t-h | --help print this message
\t-p PORT | --port=PORT where PORT is the port number to open for the RTP stream (5000 or 6000)
\t-s SCALE | --scale=SCALE where SCALE is the scaling factor to apply to the incoming video stream (default: 1)
\t-r ROTATE | --rotate=ROTATE where ROTATE is the number of clockwise 90deg rotations to apply to the stream [0-3] (default: 0)
'''
    print(fmt % lpathitem[-1])
    
def GetOptions():
    options = {'port':[], 'scale':[], 'rotate':[]}
    try:
        optlist, left_args = getopt.getopt(sys.argv[1:],'h:p:s:r:', ['help', 'port=', 'scale=', 'rotate='])
    except getopt.GetoptError:
        # print help information and exit:
        Usage(sys.argv[0])
        sys.exit(2)
    for o, a in optlist:
        if o in ("-h", "--help"):
            Usage(sys.argv[0])
            sys.exit()
        elif o in ("-p", "--port"):
            options['port'].append(int(a))
        elif o in ("-s", "--scale"):
            options['scale'].append(float(a))
        elif o in ("-r", "--rotate"):
            options['rotate'].append(int(a))

    return options
    
class RtpViewer:
    running = False
    scale = 1
    rotate = 0
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
        self.running = True

        # Start an 'infinite' loop
        while self.running:
            # Read a frame from the video capture
            ret, self.frame = self.cap.read()

            # Quit if frame could not be retrieved
            if not ret:
                break

            # Run the computer vision function
            self.cv()

            # Process key input
            self.on_key(cv2.waitKey(1) & 0xFF)

    def cv(self):
        # Rotate the image by increments of 90
        if self.rotate % 2:
            self.frame = cv2.transpose(self.frame)

        if self.rotate > 0:
            self.frame = cv2.flip(self.frame, [1, -1, 0][self.rotate - 1])

        # If a selection is happening
        if self.mouse.get('start'):
            # Draw a rectangle indicating the region of interest
            cv2.rectangle(self.frame, self.mouse['start'], self.mouse['now'], (0, 255, 0), 2)

        if self.scale != 1:
            h, w = self.frame.shape[:2]
            self.frame = cv2.resize(self.frame, (int(self.scale * w), int(self.scale * h)))

        # Show the image in a window
        cv2.imshow('rtp', self.frame)

    def on_key(self, key):
        if key == ord('q'):
            self.running = False

        if key == ord('r'):
            self.rotate = (self.rotate + 1) % 4
            self.mouse['start'] = None

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.rotate == 0 and False:
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
    import sys
    import os

    options = GetOptions()
    if not options['port']:
          Usage(sys.argv[0])
    filename = os.path.dirname(os.path.abspath(__file__)) + "/rtp_" + str(options['port'][0]) + ".sdp"
    print(filename)
    
    #set defaults
    if not options['scale']:
        options['scale'][0] = 1.
    if not options['rotate']:
        options['rotate'][0] = 0
        
    print(options['scale'][0])
    print(options['rotate'][0])

    viewer = RtpViewer(filename)
    viewer.scale = options['scale'][0]
    viewer.rotate = options['rotate'][0]

    if not viewer.cap.isOpened():
        viewer.cleanup()
        raise IOError("Can't open video stream")

    viewer.run()
    viewer.cleanup()

