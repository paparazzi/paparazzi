#!/usr/bin/python
#
#       <message NAME="DC_SHOT" ID="110">
#        <field TYPE="int16" NAME="photo_nr"/>
#        <field UNIT="cm" TYPE="int32" NAME="utm_east"/>
#        <field UNIT="cm" TYPE="int32" NAME="utm_north"/>
#        <field UNIT="m" TYPE="float" NAME="z"/>
#        <field TYPE="uint8" NAME="utm_zone"/>
#        <field UNIT="decideg" TYPE="int16" NAME="phi"/>
#        <field UNIT="decideg" TYPE="int16" NAME="theta"/>
#        <field UNIT="decideg" TYPE="int16" NAME="course"/>
#        <field UNIT="cm/s" TYPE="uint16" NAME="speed"/>
#        <field UNIT="ms" TYPE="uint32" NAME="itow"/>
#      </message>
#

# sudo apt-get install python-gdal
# sudo apt-get install gir1.2-gexiv2-0.4

from gi.repository import GExiv2
import glob
import os
import re
import fnmatch
import sys
import math

M_PI=3.14159265358979323846
M_PI_2=(M_PI/2)
M_PI_4=(M_PI/4)

def RadOfDeg( deg ):
    return (deg / M_PI) * 180.

# converts UTM coords to lat/long.  Equations from USGS Bulletin 1532
# East Longitudes are positive, West longitudes are negative.
# North latitudes are positive, South latitudes are negative
# Lat and Long are in decimal degrees.
# Written by Chuck Gantz- chuck.gantz@globalstar.com

# ( I had some code here to use GDAL and which looked much simpler, but couldn't get that to work )

def UTMtoLL( northing, easting, utm_zone ):

    k0 = 0.9996;
    a = 6378137; # WGS-84
    eccSquared = 0.00669438; # WGS-84
    e1 = (1-math.sqrt(1-eccSquared))/(1+math.sqrt(1-eccSquared));

    x = easting - 500000.0; # remove 500,000 meter offset for longitude
    y = northing;

    is_northern = northing < 0
    if ( not is_northern ):
		y -= 10000000.0 # remove 10,000,000 meter offset used for southern hemisphere

    LongOrigin = (utm_zone - 1)*6 - 180 + 3;  # +3 puts origin in middle of zone

    eccPrimeSquared = (eccSquared)/(1-eccSquared);

    M = y / k0;
    mu = M/(a*(1-eccSquared/4-3*eccSquared*eccSquared/64-5*eccSquared*eccSquared*eccSquared/256));

    phi1Rad = mu	+ (3*e1/2-27*e1*e1*e1/32)*math.sin(2*mu) + (21*e1*e1/16-55*e1*e1*e1*e1/32)*math.sin(4*mu) +(151*e1*e1*e1/96)*math.sin(6*mu);
    phi1 = RadOfDeg(phi1Rad);

    N1 = a/math.sqrt(1-eccSquared*math.sin(phi1Rad)*math.sin(phi1Rad));
    T1 = math.tan(phi1Rad)*math.tan(phi1Rad);
    C1 = eccPrimeSquared*math.cos(phi1Rad)*math.cos(phi1Rad);
    R1 = a*(1-eccSquared)/math.pow(1-eccSquared*math.sin(phi1Rad)*math.sin(phi1Rad), 1.5);
    D = x/(N1*k0);

    Lat = phi1Rad - (N1*math.tan(phi1Rad)/R1)*(D*D/2-(5+3*T1+10*C1-4*C1*C1-9*eccPrimeSquared)*D*D*D*D/24+(61+90*T1+298*C1+45*T1*T1-252*eccPrimeSquared-3*C1*C1)*D*D*D*D*D*D/720);
    Lat = RadOfDeg(Lat)

    Long = (D-(1+2*T1+C1)*D*D*D/6+(5-2*C1+28*T1-3*C1*C1+8*eccPrimeSquared+24*T1*T1)*D*D*D*D*D/120)/math.cos(phi1Rad)
    Long = LongOrigin + RadOfDeg(Long)

    return Lat, Long


# At least the directory must be given
if len(sys.argv) < 2:
    print "This script requires one argument: A directory containing photos and the paparazzi .data file"
    sys.exit()

path = str(sys.argv[ 1] )

if os.path.isdir(path) == False:
    print "The indicated path '%s' is not a directory"%(path)
    sys.exit()

# Searching for all files with .data extension in indicated directory.
# It should only have one.
list_path = [i for i in os.listdir(path) if os.path.isfile(os.path.join(path, i))]
files = [os.path.join(path, j) for j in list_path if re.match(fnmatch.translate('*.data'), j, re.IGNORECASE)]

if  len(files) > 1:
    print "Too many data files found. Only one is allowed."
    sys.exit()

if len(files) == 0:
    print "No data files in 'data'. Copy data file there."
    sys.exit()

# Now searching for all photos (extension .jpg) in directory
list_path = [i for i in os.listdir(path) if os.path.isfile(os.path.join(path, i))]
photos = [os.path.join(path, j) for j in list_path if re.match(fnmatch.translate('*.jpg'), j, re.IGNORECASE)]

# Photos must be sorted by number
photos.sort()

# Opening the data file, iterating all lines and searching for DC_SHOT messages
f = open( files[0], 'r' )
for line in f:
    line = line.rstrip()
    line = re.sub(' +',' ',line)
    if 'DC_SHOT' in line:
        # 618.710 1 DC_SHOT 212 29133350 -89510400 8.5 25 -9 29 0 0 385051650
        splitted = line.split( ' ' )

        try:
            # old DC_SHOT message has 10 data fields with pos in UTM:
            # photo_nr, utm_east, utm_north, z, utm_zone, phi, theta, course, speed, itow
            if len(splitted) == 13:
                photonr = int(splitted[ 3 ])
                utm_east = ( float(int(splitted[ 4 ])) / 100. )
                utm_north = ( float(int(splitted[ 5 ])) / 100. )
                alt = float(splitted[ 6 ])
                utm_zone = int(splitted[ 7 ])
                phi = int(splitted[ 8 ])
                theta = int(splitted[ 9 ])
                course = int(splitted[ 10 ])
                speed = int(splitted[ 11 ])
                itow = int(splitted[ 12 ])

                lon, lat = UTMtoLL( utm_north, utm_east, utm_zone )

            # current DC_SHOT messages has 11 data fields with pos in LLA:
            # photo_nr, latitude, longitude, altitude, hmsl, phi, theta, psi, course, speed, itow
            else if len(splitted) == 14:
                photonr = int(splitted[ 3 ])
                lat = RadOfDeg(int(splitted[ 4 ]) * 0.0000001) # to radians
                lon = RadOfDeg(int(splitted[ 5 ]) * 0.0000001) # to radians
                alt = int(splitted[ 6 ]) * 0.001 # to meters
                hmsl = int(splitted[ 7 ]) * 0.001
                phi = int(splitted[ 8 ])
                theta = int(splitted[ 9 ])
                psi = int(splitted[ 10 ]))
                course = int(splitted[ 11 ])
                speed = int(splitted[ 12 ])
                itow = int(splitted[ 13])
            else:
                continue

            # Check that there as many photos and pick the indicated one.
            # (this assumes the photos were taken correctly without a hiccup)
            # It would never be able to check this anyway, since the camera could stall or
            # not interpret the pulse?  Leading to an incorrect GPS coordinate.
            if len( photos ) < photonr:
                print "Photo data %d found, but ran out of photos in directory"%(photonr)
                continue

            # I've seen log files with -1 as DC_SHOT number due to an int8 I think. This should be
            # fixed now, but just in case someone runs this on old data.
            if (photonr < 0):
                print "Negative photonr found."
                continue

            # Pick out photo, open it through exiv2,
            photoname = photos[ photonr - 1 ]
            photo = GExiv2.Metadata( photoname )

            photo.set_gps_info(lat, lon, alt)
            photo.save_file()

            print "Photo %s and photonr %d merged. Lat/Lon/Alt: %f, %f, %f"%(photoname, photonr, lat, lon, alt)

        except ValueError as e:
            print "Cannot read line: %s"%(line)
            print "Value error(%s)"%(e)
            continue

print "Finished! exiting."

