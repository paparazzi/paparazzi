import redis
import time
import os
import sys
import getopt
import signal
from ivy.std_api import *
import logging
import re

DEFAULT_SERVER = 'localhost'
DEFAULT_PORT = 6379

server = None

class IvyMessagesInterface():
    def __init__(self, callback, initIvy = True):
        self.callback = callback
        self.ivy_id = 0
        self.InitIvy(initIvy)

    def Stop(self):
        IvyUnBindMsg(self.ivy_id)

    def Shutdown(self):
        self.Stop()
        IvyStop()

    def __init__del__(self):
        try:
            IvyUnBindMsg(self.ivy_id)
        except:
            pass

    def InitIvy(self, initIvy):
        if initIvy:
            IvyInit("Messages %i" % os.getpid(), "READY", 0, lambda x,y: y, lambda x,y: y)
            logging.getLogger('Ivy').setLevel(logging.WARN)
            IvyStart("")
        self.ivy_id = IvyBindMsg(self.OnIvyMsg, "(.*)")

    def OnIvyMsg(self, agent, *larg):
        """ Split ivy message up into the separate parts
        Basically parts/args in string are separated by space, but char array can also contain a space:
        |f,o,o, ,b,a,r| in old format or "foo bar" in new format
        """
        # first split on array delimiters
        l = re.split('([|\"][^|]*[|\"])', larg[0])
        # strip spaces and filter out emtpy strings
        l = [str.strip(s) for s in l if str.strip(s) is not '']
        data = []
        for s in l:
            # split non-array strings further up
            if '|' not in s and '"' not in s:
                data += s.split(' ')
            else:
                data.append(s)
        try:
            ac_id = data[0]
            name = data[1]
            values = list(filter(None, data[2:]))
            self.callback(ac_id, name, values)
        except IndexError as ie:
            logging.error( "Index error on this line: %s"%( l ) )

class Ivy2RedisServer():
    def __init__( self, redishost, redisport ):
        self.interface = IvyMessagesInterface( self.message_recv )
        self.r = redis.StrictRedis(host=redishost, port=redisport, db=0)
        self.keep_running = True

    def message_recv(self, ac_id, name, values):
        key = ac_id + "." + name
        self.r.publish( key, values )
        self.r.set( key, values )

    def run( self ):
        while( self.keep_running ):
            time.sleep( 0.1 )

    def stop( self ):
        self.keep_running = False
        self.interface.Shutdown()

def Usage(scmd):
    lpathitem = scmd.split('/')
    fmt = '''Usage: %s [-h | --help] [-s redisserver] [-p redisport]
\t-h | --help print this message
\t-s hostname | --server=redisserver the hostname where redis runs
\t-p PORT | --port=PORT where PORT is the number of the port used by redis
'''
    print fmt   %  lpathitem[-1]

def GetOptions():
    options = {'server':DEFAULT_SERVER, 'port':DEFAULT_PORT}
    try:
        optlist, left_args = getopt.getopt(sys.argv[1:],'h:s:p:', ['help', 'server=', 'port='])
    except getopt.GetoptError:
        # print help information and exit:
        Usage(sys.argv[0])
        sys.exit(2)

    for o, a in optlist:
        if o in ("-h", "--help"):
            Usage(sys.argv[0])
            sys.exit()
        elif o in ("-s", "--server"):
            options['server'] = [ int(a) ]
        elif o in ("-p", "--port"):
            options['port'] = a
    return options

def signal_handler(signal, frame):
    global server
    server.stop()

def main():
    global server
    signal.signal(signal.SIGINT, signal_handler)
    options = GetOptions()
    server = Ivy2RedisServer( options['server'], options['port'] )
    server.run()

if __name__ == '__main__':
    main()
