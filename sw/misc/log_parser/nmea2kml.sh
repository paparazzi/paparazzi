#!/bin/bash
#	kml                   Google Earth (Keyhole) Markup Language
#	  deficon               Default icon name 
#	  lines                 (0/1) Export linestrings for tracks and routes 
#	  points                (0/1) Export placemarks for tracks and routes 
#	  line_width            Width of lines, in pixels 
#	  line_color            Line color, specified in hex AABBGGRR 
#	  floating              (0/1) Altitudes are absolute and not clamped to ground 
#	  extrude               (0/1) Draw extrusion line from trackpoint to ground 
#	  trackdata             (0/1) Include extended data for trackpoints (default = 1 
#	  trackdirection        (0/1) Indicate direction of travel in track icons (defau 
#	  units                 Units used when writing comments ('s'tatute or 'm' 
#	  labels                (0/1) Display labels on track and routepoints  (default  
#	  max_position_point    Retain at most this number of position points  (0  

gpsbabel -i nmea -f "$@" -o kml,deficon=funjet,line_width=1,floating=1,labels=0 -F ppzout.kml
