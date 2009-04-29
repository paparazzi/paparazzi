#! /usr/bin/env python

import time
import serial  # sudo apt-get install python-serial

#open()                  #open port
#close()                 #close port immediately
#setBaudrate(baudrate)   #change baudrate on an open port
#inWaiting()             #return the number of chars in the receive buffer
#read(size=1)            #read "size" characters
#write(s)                #write the string s to the port
#flushInput()            #flush input buffer, discarding all it's contents
#flushOutput()           #flush output buffer, abort output
#sendBreak()             #send break condition
#setRTS(level=1)         #set RTS line to specified logic level
#setDTR(level=1)         #set DTR line to specified logic level
#getCTS()                #return the state of the CTS line
#getDSR()                #return the state of the DSR line
#getRI()                 #return the state of the RI line
#getCD()                 #return the state of the CD line


class arduino_dangerboard():
  def __init__(self):
    self.port = serial.Serial('/dev/ttyUSB0', 115200)  
    self.SLIDER_COUNT = 3
    self.sliders = [0] * self.SLIDER_COUNT

  def HandleEvent(self):
    pass
    
  def poll(self):
    while( True):
    
      self.port.write('G');

      foo = self.port.inWaiting()
            
      if foo == 6:
        a = ord( self.port.read())
        b = ord( self.port.read())
        c = ord( self.port.read())
        d = ord( self.port.read())
        e = ord( self.port.read())
        f = ord( self.port.read())
        
        self.sliders[0] = (a << 8) | b;
        self.sliders[1] = (c << 8) | d;
        self.sliders[2] = (e << 8) | f;

        self.HandleEvent()
      
      else:  # flush queue
        while foo:
          foo -= 1
          data = self.port.read()
      
      time.sleep(0.25);
      


def main():
  try:
    foo = dangerboard()
    foo.poll()
  except KeyboardInterrupt:
    pass


if __name__ == '__main__':
    main()
