#!/usr/bin/python
#This Python script generates a list of source files.
#It finds the correct VPATH to each source file from make, and then feeds all info to gcc -MM.
import sys
import os
from os import path, getenv

# first argument is the gcc compiler
# thereafter are the vpaths closed by a '+'
# thereafter are the srcs closed by a '+'
# thereafter are the cflags

#parse the input arguments into seperate lists
cc = sys.argv[1]
vpaths = list()
srcs = list()
cflags = list()
mode = 0
for i in range(2,len(sys.argv)):
  vpath = sys.argv[i]
  if vpath == '+':
    mode = mode + 1
  else:
    if mode == 0:
      vpaths.append(vpath)
    elif mode == 1:
      srcs.append(sys.argv[i])
    elif mode == 2:
      cflags.append(sys.argv[i])

#create the command for gcc -MM cflags srcs
cmd = cc + ' -MM '
for flg in cflags:
  qflg = "" # append all quotes with escape char
  for i in range(0,len(flg)):
    if flg[i] == "\"":
      qflg = qflg +  "\\\""
    else:
      qflg = qflg + flg[i]
  cmd = cmd + qflg + ' '

for i in range(0,len(srcs)):
  f = srcs[i]
  found = 0
  for vpath in vpaths:
    if os.path.isfile(path.join(vpath, f)) :
      cmd = cmd + path.join(vpath, f) + ' '
      found = found +1
      break
  if found == 0:
    print("ERROR, could not find: " + f)
  if found > 1:
    print("ERROR, found more than once: " + f)

#call gcc
os.system(cmd)

