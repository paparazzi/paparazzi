#! /usr/bin/env python

import struct

with open('uart_log.bin', 'rb') as data:
    with open('converted.csv', 'w') as out:
        out.write("sync,nr,gyro,,,accelero,,,phi/100,theta/100,psi/100\n")
        while (1):
            sync, nr, g1, g2, g3, s1, s2, s3, a1, a2, a3, crc = struct.unpack('<H10hB', data.read(23))
            out.write(hex(sync) + "," + str(nr) + "," + str(g1) + "," + str(g2) + "," + str(g3) + "," + str(s1) + "," + str(s2) + "," + str(s3) + "," + str(a1) + "," + str(a2) + "," + str(a3) + "\n")
