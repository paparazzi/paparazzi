import os
import platform

PAPARAZZI_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                         '../../..')))
PAPARAZZI_HOME = os.getenv("PAPARAZZI_HOME", PAPARAZZI_SRC)

if os.getenv('IVY_BUS') is not None:
    IVY_BUS = os.getenv('IVY_BUS')
elif platform.system() == 'Darwin':
    IVY_BUS = "224.255.255.255:2010"
else:
    IVY_BUS = ""
