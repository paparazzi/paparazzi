#!/usr/bin/python
#This Python script generates a list of source files.
#It finds the correct VPATH to each source file from make, and then feeds all info to gcc -MM.
import sys
import os
from os import path, getenv, environ

pprzswdir = os.environ['PAPARAZZI_SRC']
pprzswdir = path.join(pprzswdir, "sw/airborne")

cc = sys.argv[1]

with open(sys.argv[2]) as fp:
    vpathsl = fp.readline()
    srcsl = fp.readline()
    cflagsl = fp.readline()
    includesl = fp.readline()
    toptl = fp.readline()

    vpaths = vpathsl.strip().split(" ")
    srcs = srcsl.strip().split(" ")
    cflags = cflagsl.strip().split(" ")
    includes = includesl.strip().split(" ")

    
    cflagsn = "" #" -I" + pprzswdir + " -I" + pprzvardir + " "
    for i in range(0,len(cflags)):
        f = cflags[i]          

        nf = "" # append all quotes with escape char
        for j in range(0,len(f)):
            if f[j] == "\"":
                nf = nf +  "\\\""
            else:
                nf = nf + f[j]
        f = nf


        if f.startswith("-I"):   #for whatever reason, these includes miss the pprz path. Which gives issues with gcc         
            f = "-I" + path.join(pprzswdir, f[2:])                    
        elif f.startswith ("-M"):
            break;
            f = " "
        cflagsn = cflagsn + f + " "

    topt = toptl.strip().split(" ")

    vpaths.append(pprzswdir)    

    #create the command for gcc -MM cflags srcs
    cmd = cc + ' -MM ' + includesl.strip() + " " + cflagsn.strip() + " " + toptl.strip()

    #print("****************************************************")
    for i in range(0,len(srcs)):
        f = srcs[i]
        found = 0
        for vpath in vpaths:
            if os.path.isfile(path.join(vpath, f)) :
                if found == 0:
                    # add file in the path only once
                    cmd = cmd + path.join(vpath, f) + ' '
                    found = found +1
                # break
        if found == 0:
            tmps = "ERROR: could not find src: " + f
            cmd = cmd + tmps #hack to make the error known
            print(tmps)            
        if found > 1:
            # NOTE: this will never happen
            tmps = "ERROR: found src more than once: " + f
            cmd = cmd + tmps #hack to make the error known
            print(tmps)            
    #print(cmd)
    #call gcc
    os.system(cmd)

