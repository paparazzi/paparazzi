#Copyright 2014, Antoine Drouin
"""
Phoenix is a Python library for interacting with Paparazzi
"""

import math

"""
Unit convertions
"""
def rad_of_deg(d): return d/180.*math.pi

def deg_of_rad(r): return r*180./math.pi

def rps_of_rpm(r): return r*2.*math.pi/60.

def rpm_of_rps(r): return r/2./math.pi*60.

def m_of_inch(i): return i*0.0254


"""
Plotting
"""
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

my_title_spec = {'color'    : 'k', 'fontsize'   : 20 }

def save_if(filename):
    if filename: matplotlib.pyplot.savefig(filename, dpi=80)

def prepare_fig(fig=None, window_title=None, figsize=(20.48, 10.24), margins=None):
    if fig == None:
        fig = plt.figure(figsize=figsize)
#    else:
#        plt.figure(fig.number)
    if margins:
        left, bottom, right, top, wspace, hspace = margins
        fig.subplots_adjust(left=left, right=right, bottom=bottom, top=top,
                            hspace=hspace, wspace=wspace)
    if window_title:
         fig.canvas.set_window_title(window_title)
    return fig

def decorate(ax, title=None, xlab=None, ylab=None, legend=None, xlim=None, ylim=None):
    ax.xaxis.grid(color='k', linestyle='-', linewidth=0.2)
    ax.yaxis.grid(color='k', linestyle='-', linewidth=0.2)
    if xlab:
        ax.xaxis.set_label_text(xlab)
    if ylab:
        ax.yaxis.set_label_text(ylab)
    if title:
        ax.set_title(title, my_title_spec)
    if legend <> None:
        ax.legend(legend, loc='best')
    if xlim <> None:
        ax.set_xlim(xlim[0], xlim[1])
    if ylim <> None:
        ax.set_ylim(ylim[0], ylim[1])


"""
Messages
"""

#: dictionary mapping the C type to its length in bytes (e.g char -> 1)
TYPE_TO_LENGTH_MAP = {
    "char"      :   1,
    "uint8"     :   1,
    "int8"      :   1,
    "uint16"    :   2,
    "int16"     :   2,
    "uint32"    :   4,
    "int32"     :   4,
    "float"     :   4,
    "double"    :   8,
}

#: dictionary mapping the C type to correct format string
TYPE_TO_PRINT_MAP = {
        float   :   "%f",
        str     :   "%s",
        chr     :   "%c",
        int     :   "%d"
}

ACID_ALL            = 0xFF
ACID_TEST           = 0xFE
ACID_GROUNDSTATION  = 0xFD

#: dictionary mapping debug types to format characters
DEBUG_MESSAGES = {
    "DEBUG_UINT8"   :   "%d",
    "DEBUG_INT32"   :   "%d",
    "DEBUG_FLOAT"   :   "%#f"
}




"""
Binary logs


"""

import struct

def hex_of_bin(b): return ' '.join( [ "%02X" % ord( x ) for x in b ] )
import pdb

def read_binary_log(filename, tick_freq = 2*512.):
    f = open(filename, "rb")
    d = f.read()
    packet_header_len = 6
    msg_header_len = 2

    def read_packet(d, packet_start):
        payload_start = packet_start+packet_header_len
        timestamp, payload_len = struct.unpack("IH", d[packet_start:payload_start])
        msgs = read_packet_payload(d, payload_start, payload_len)
        next_packet = payload_start+payload_len+2
        return timestamp, msgs, next_packet

    def read_packet_payload(d, s, l):
        msgs = []
        packet_end = s+l; msg_start = s
        while msg_start<packet_end:
            payload_start = msg_start+msg_header_len
            msg_len, msg_id = struct.unpack("BB", d[msg_start:payload_start])
            payload_end = payload_start+msg_len
            msg_payload = d[payload_start:payload_end]
            msgs.append([msg_id, msg_payload])
            #print msg_id, msg_len, hex_of_bin(msg_payload)
            msg_start = payload_end
        return msgs

    packets = []
    packet_start=0
    while packet_start<len(d):
        timestamp, msgs, next_packet = read_packet(d, packet_start)
        packets.append([timestamp/tick_freq, msgs])
        #print timestamp, msgs
        packet_start = next_packet
    f.close()
    return packets


def extract_from_binary_log(protocol, packets, msg_names, t_min=None, t_max=None):
    ret = [{'time':[], 'data':[]} for m in msg_names]
    if t_min == None: t_min = packets[0][0]
    if t_max == None: t_max = packets[-1][0]
    for t, msgs in packets:
        if t>= t_min and t<= t_max:
            for id, payload in msgs:
                m = protocol.get_message_by_id('telemetry', id)
                try: i = msg_names.index(m.name)
                except: pass
                finally: ret[i]['time'].append(t);  ret[i]['data'].append(m.unpack_scaled_values(payload))
    return ret
