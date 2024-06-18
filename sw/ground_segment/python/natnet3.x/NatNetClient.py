#Copyright © 2018 Naturalpoint
#
#Licensed under the Apache License, Version 2.0 (the "License")
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at
#
#http://www.apache.org/licenses/LICENSE-2.0
#
#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.

# OptiTrack NatNet direct depacketization library for Python 3.x

import socket
import struct
from threading import Thread
import copy
import time
import DataDescriptions
import MoCapData

def trace( *args ):
    # uncomment the one you want to use
    #print( "".join(map(str,args)) )
    pass

#Used for Data Description functions
def trace_dd( *args ):
    # uncomment the one you want to use
    #print( "".join(map(str,args)) )
    pass

#Used for MoCap Frame Data functions
def trace_mf( *args ):
    # uncomment the one you want to use
    #print( "".join(map(str,args)) )
    pass

def get_message_id(data):
    message_id = int.from_bytes( data[0:2], byteorder='little' )
    return message_id


# Create structs for reading various object types to speed up parsing.
Vector2 = struct.Struct( '<ff' )
Vector3 = struct.Struct( '<fff' )
Quaternion = struct.Struct( '<ffff' )
FloatValue = struct.Struct( '<f' )
DoubleValue = struct.Struct( '<d' )
NNIntValue = struct.Struct( '<I')
FPCalMatrixRow = struct.Struct( '<ffffffffffff' )
FPCorners      = struct.Struct( '<ffffffffffff')

class NatNetClient:
    # print_level = 0 off
    # print_level = 1 on
    # print_level = >1 on / print every nth mocap frame
    print_level = 20
    
    def __init__( self ):
        # Change this value to the IP address of the NatNet server.
        self.server_ip_address = "127.0.0.1"

        # Change this value to the IP address of your local network interface
        self.local_ip_address = "127.0.0.1"

        # This should match the multicast address listed in Motive's streaming settings.
        self.multicast_address = "239.255.42.99"

        # NatNet Command channel
        self.command_port = 1510

        # NatNet Data channel
        self.data_port = 1511

        self.use_multicast = True

        # Set this to a callback method of your choice to receive per-rigid-body data at each frame.
        self.rigid_body_listener = None
        self.new_frame_listener  = None
        self.rigid_body_list_listener = None
        self.marker_set_listener = None

        # Set Application Name
        self.__application_name = "Not Set"

        # NatNet stream version server is capable of. This will be updated during initialization only.
        self.__nat_net_stream_version_server = [0,0,0,0]

        # NatNet stream version. This will be updated to the actual version the server is using during runtime.
        self.__nat_net_requested_version = [0,0,0,0]

        # server stream version. This will be updated to the actual version the server is using during initialization.
        self.__server_version = [0,0,0,0]

        # Lock values once run is called
        self.__is_locked = False

        # Server has the ability to change bitstream version
        self.__can_change_bitstream_version = False

        self.command_thread = None
        self.data_thread = None
        self.command_socket = None
        self.data_socket = None

        self.stop_threads=False


    # Client/server message ids
    NAT_CONNECT               = 0
    NAT_SERVERINFO            = 1
    NAT_REQUEST               = 2
    NAT_RESPONSE              = 3
    NAT_REQUEST_MODELDEF      = 4
    NAT_MODELDEF              = 5
    NAT_REQUEST_FRAMEOFDATA   = 6
    NAT_FRAMEOFDATA           = 7
    NAT_MESSAGESTRING         = 8
    NAT_DISCONNECT            = 9
    NAT_KEEPALIVE             = 10
    NAT_UNRECOGNIZED_REQUEST  = 100
    NAT_UNDEFINED             = 999999.9999


    def set_client_address(self, local_ip_address):
        if not self.__is_locked:
            self.local_ip_address = local_ip_address

    def get_client_address(self):
        return self.local_ip_address

    def set_server_address(self,server_ip_address):
        if not self.__is_locked:
            self.server_ip_address = server_ip_address

    def get_server_address(self):
        return self.server_ip_address


    def set_use_multicast(self, use_multicast):
        if not self.__is_locked:
            self.use_multicast = use_multicast

    def can_change_bitstream_version(self):
        return self.__can_change_bitstream_version

    def set_nat_net_version(self, major, minor):
        """checks to see if stream version can change, then changes it with position reset"""
        return_code = -1
        if self.__can_change_bitstream_version and \
            ((major != self.__nat_net_requested_version[0]) or\
             (minor != self.__nat_net_requested_version[1])):
            sz_command = "Bitstream,%1.1d.%1.1d"%(major, minor)
            return_code = self.send_command(sz_command)
            if return_code >=0:
                self.__nat_net_requested_version[0] = major
                self.__nat_net_requested_version[1] = minor
                self.__nat_net_requested_version[2] = 0
                self.__nat_net_requested_version[3] = 0
                print("changing bitstream MAIN")
                # get original output state
                #print_results = self.get_print_results()
                #turn off output
                #self.set_print_results(False)
                # force frame send and play reset
                self.send_command("TimelinePlay")
                time.sleep(0.1)
                tmpCommands=["TimelinePlay",
                    "TimelineStop",
                    "SetPlaybackCurrentFrame,0",
                    "TimelineStop"]
                self.send_commands(tmpCommands,False)
                time.sleep(2)
                #reset to original output state
                #self.set_print_results(print_results)
            else:
                print("Bitstream change request failed")
        return return_code


    def get_major(self):
        return self.__nat_net_requested_version[0]

    def get_minor(self):
        return self.__nat_net_requested_version[1]

    def set_print_level(self, print_level=0):
        if(print_level >=0):
            self.print_level = print_level
        return self.print_level

    def get_print_level(self):
        return self.print_level


    def connected(self):
        ret_value = True
        # check sockets
        if self.command_socket == None:
            ret_value = False
        elif self.data_socket ==None:
            ret_value = False
        # check versions
        elif self.get_application_name() == "Not Set":
            ret_value = False
        elif (self.__server_version[0] == 0) and\
            (self.__server_version[1] == 0) and\
            (self.__server_version[2] == 0) and\
            (self.__server_version[3] == 0):
            ret_value = False
        return ret_value


    # Create a command socket to attach to the NatNet stream
    def __create_command_socket( self ):
        result = None
        if self.use_multicast :
            # Multicast case
            result = socket.socket( socket.AF_INET, socket.SOCK_DGRAM, 0 )
            # allow multiple clients on same machine to use multicast group address/port
            result.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                result.bind( ('', 0) )
            except socket.error as msg:
                print("ERROR: command socket error occurred:\n%s" %msg)
                print("Check Motive/Server mode requested mode agreement.  You requested Multicast ")
                result = None
            except  socket.herror:
                print("ERROR: command socket herror occurred")
                result = None
            except  socket.gaierror:
                print("ERROR: command socket gaierror occurred")
                result = None
            except  socket.timeout:
                print("ERROR: command socket timeout occurred. Server not responding")
                result = None
            # set to broadcast mode
            result.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            # set timeout to allow for keep alive messages
            result.settimeout(2.0)
        else:
            # Unicast case
            result = socket.socket( socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            try:
                result.bind( (self.local_ip_address, 0) )
            except socket.error as msg:
                print("ERROR: command socket error occurred:\n%s" %msg)
                print("Check Motive/Server mode requested mode agreement.  You requested Unicast ")
                result = None
            except socket.herror:
                print("ERROR: command socket herror occurred")
                result = None
            except socket.gaierror:
                print("ERROR: command socket gaierror occurred")
                result = None
            except socket.timeout:
                print("ERROR: command socket timeout occurred. Server not responding")
                result = None

            # set timeout to allow for keep alive messages
            result.settimeout(2.0)
            result.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        return result

    # Create a data socket to attach to the NatNet stream
    def __create_data_socket( self, port ):
        result = None

        if self.use_multicast:
            # Multicast case
            result = socket.socket( socket.AF_INET,     # Internet
                                  socket.SOCK_DGRAM,
                                  0)    # UDP
            result.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            result.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(self.multicast_address) + socket.inet_aton(self.local_ip_address))
            try:
                result.bind( (self.local_ip_address, port) )
            except socket.error as msg:
                print("ERROR: data socket error occurred:\n%s" %msg)
                print("  Check Motive/Server mode requested mode agreement.  You requested Multicast ")
                result = None
            except socket.herror:
                print("ERROR: data socket herror occurred")
                result = None
            except socket.gaierror:
                print("ERROR: data socket gaierror occurred")
                result = None
            except socket.timeout:
                print("ERROR: data socket timeout occurred. Server not responding")
                result = None
        else:
            # Unicast case
            result = socket.socket( socket.AF_INET,     # Internet
                                  socket.SOCK_DGRAM,
                                  socket.IPPROTO_UDP)
            result.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            #result.bind( (self.local_ip_address, port) )
            try:
                result.bind( ('', 0) )
            except socket.error as msg:
                print("ERROR: data socket error occurred:\n%s" %msg)
                print("Check Motive/Server mode requested mode agreement.  You requested Unicast ")
                result = None
            except socket.herror:
                print("ERROR: data socket herror occurred")
                result = None
            except socket.gaierror:
                print("ERROR: data socket gaierror occurred")
                result = None
            except socket.timeout:
                print("ERROR: data socket timeout occurred. Server not responding")
                result = None
            
            if(self.multicast_address != "255.255.255.255"):
                result.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(self.multicast_address) + socket.inet_aton(self.local_ip_address))

        return result

    # Unpack a rigid body object from a data packet
    def __unpack_rigid_body( self, data, major, minor, rb_num):
        offset = 0

        # ID (4 bytes)
        new_id = int.from_bytes( data[offset:offset+4], byteorder='little' )
        offset += 4

        trace_mf( "RB: %3.1d ID: %3.1d"% (rb_num, new_id))

        # Position and orientation
        pos = Vector3.unpack( data[offset:offset+12] )
        offset += 12
        trace_mf( "\tPosition    : [%3.2f, %3.2f, %3.2f]"% (pos[0], pos[1], pos[2] ))

        rot = Quaternion.unpack( data[offset:offset+16] )
        offset += 16
        trace_mf( "\tOrientation : [%3.2f, %3.2f, %3.2f, %3.2f]"% (rot[0], rot[1], rot[2], rot[3] ))

        rigid_body = MoCapData.RigidBody(new_id, pos, rot)

        # Send information to any listener.
        if self.rigid_body_listener is not None:
            self.rigid_body_listener( new_id, pos, rot )

        # RB Marker Data ( Before version 3.0.  After Version 3.0 Marker data is in description )
        if( major < 3  and major != 0) :
            # Marker count (4 bytes)
            marker_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
            offset += 4
            marker_count_range = range( 0, marker_count )
            trace_mf( "\tMarker Count:", marker_count )

            rb_marker_list=[]
            for i in marker_count_range:
                rb_marker_list.append(MoCapData.RigidBodyMarker())

            # Marker positions
            for i in marker_count_range:
                pos = Vector3.unpack( data[offset:offset+12] )
                offset += 12
                trace_mf( "\tMarker", i, ":", pos[0],",", pos[1],",", pos[2] )
                rb_marker_list[i].pos=pos

            if major >= 2:
                # Marker ID's
                for i in marker_count_range:
                    new_id = int.from_bytes( data[offset:offset+4], byteorder='little' )
                    offset += 4
                    trace_mf( "\tMarker ID", i, ":", new_id )
                    rb_marker_list[i].id=new_id

                # Marker sizes
                for i in marker_count_range:
                    size = FloatValue.unpack( data[offset:offset+4] )
                    offset += 4
                    trace_mf( "\tMarker Size", i, ":", size[0] )
                    rb_marker_list[i].size=size

            for i in marker_count_range:
                rigid_body.add_rigid_body_marker(rb_marker_list[i])
        if major >= 2 :
            marker_error, = FloatValue.unpack( data[offset:offset+4] )
            offset += 4
            trace_mf( "\tMean Marker Error: %3.2f"% marker_error )
            rigid_body.error = marker_error

        # Version 2.6 and later
        if ( ( major == 2 ) and ( minor >= 6 ) ) or major > 2 :
            param, = struct.unpack( 'h', data[offset:offset+2] )
            tracking_valid = ( param & 0x01 ) != 0
            offset += 2
            is_valid_str='False'
            if tracking_valid:
                is_valid_str = 'True'
            trace_mf( "\tTracking Valid: %s"%is_valid_str)
            if tracking_valid:
                rigid_body.tracking_valid = True
            else:
                rigid_body.tracking_valid = False

        return offset, rigid_body

    # Unpack a skeleton object from a data packet
    def __unpack_skeleton( self, data, major, minor, skeleton_num=0):

        offset = 0
        new_id = int.from_bytes( data[offset:offset+4], byteorder='little' )
        offset += 4
        trace_mf( "Skeleton %3.1d ID: %3.1d"% (skeleton_num, new_id ))
        skeleton = MoCapData.Skeleton(new_id)

        rigid_body_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
        offset += 4
        trace_mf( "Rigid Body Count : %3.1d"% rigid_body_count )
        if(rigid_body_count > 0):
            for rb_num in range( 0, rigid_body_count ):
                offset_tmp, rigid_body = self.__unpack_rigid_body( data[offset:], major, minor, rb_num )
                skeleton.add_rigid_body(rigid_body)
                offset+=offset_tmp

        return offset, skeleton

#Unpack Mocap Data Functions
    def __unpack_frame_prefix_data( self, data):
        offset = 0
        # Frame number (4 bytes)
        frame_number = int.from_bytes( data[offset:offset+4], byteorder='little' )
        offset += 4
        trace_mf( "Frame #: %3.1d"% frame_number )
        frame_prefix_data=MoCapData.FramePrefixData(frame_number)
        return offset, frame_prefix_data

    def __unpack_data_size(self, data, major, minor):
        sizeInBytes=0
        offset=0

        if( ( (major == 4) and (minor>0) ) or (major > 4)):
            sizeInBytes = int.from_bytes( data[offset:offset+4], byteorder='little' )
            offset += 4
            trace_mf( "Byte Count: %3.1d"% sizeInBytes )

        return offset, sizeInBytes

    def __unpack_legacy_other_markers( self, data, packet_size, major, minor):
        offset = 0

        # Marker set count (4 bytes)
        other_marker_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
        offset += 4
        trace_mf( "Other Marker Count:", other_marker_count )

        # get data size (4 bytes)
        offset_tmp, unpackedDataSize = self.__unpack_data_size(data[offset:],major, minor)
        offset += offset_tmp

        other_marker_data = MoCapData.LegacyMarkerData()
        if(other_marker_count > 0):
            # get legacy_marker positions
            ### legacy_marker_data
            for j in range( 0, other_marker_count ):
                pos = Vector3.unpack( data[offset:offset+12] )
                offset += 12
                trace_mf( "\tMarker %3.1d : [x=%3.2f,y=%3.2f,z=%3.2f]"%( j, pos[0], pos[1], pos[2] ))
                other_marker_data.add_pos(pos)
 
        return offset, other_marker_data

    def __unpack_marker_set_data( self, data, packet_size, major, minor):
        marker_set_data=MoCapData.MarkerSetData()
        offset = 0
        # Marker set count (4 bytes)
        marker_set_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
        offset += 4
        trace_mf( "Marker Set Count:", marker_set_count )

        # get data size (4 bytes)
        offset_tmp, unpackedDataSize = self.__unpack_data_size(data[offset:],major, minor)
        offset += offset_tmp

        for i in range( 0, marker_set_count ):
            marker_data = MoCapData.MarkerData()
            # Model name
            model_name, separator, remainder = bytes(data[offset:]).partition( b'\0' )
            offset += len( model_name ) + 1
            trace_mf( "Model Name      : ", model_name.decode( 'utf-8' ) )
            marker_data.set_model_name(model_name)
            # Marker count (4 bytes)
            marker_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
            offset += 4
            if(marker_count < 0):
                print("WARNING: Early return.  Invalid marker count")
                offset = len(data)
                return offset, marker_set_data
            elif(marker_count > 10000):
                print("WARNING: Early return.  Marker count too high")
                offset = len(data)
                return offset, marker_set_data

            trace_mf( "Marker Count    : ", marker_count )
            for j in range( 0, marker_count ):
                if(len(data)<(offset+12)):
                    print("WARNING: Early return.  Out of data at marker ",j," of ", marker_count)
                    offset = len(data)
                    return offset, marker_set_data
                    break
                pos = Vector3.unpack( data[offset:offset+12] )
                offset += 12
                trace_mf( "\tMarker %3.1d : [x=%3.2f,y=%3.2f,z=%3.2f]"%( j, pos[0], pos[1], pos[2] ))
                marker_data.add_pos(pos)
            marker_set_data.add_marker_data(marker_data)

        # Unlabeled markers count (4 bytes)
        #unlabeled_markers_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
        #offset += 4
        #trace_mf( "Unlabeled Markers Count:", unlabeled_markers_count )

        #for i in range( 0, unlabeled_markers_count ):
        #    pos = Vector3.unpack( data[offset:offset+12] )
        #    offset += 12
        #    trace_mf( "\tMarker %3.1d : [%3.2f,%3.2f,%3.2f]"%( i, pos[0], pos[1], pos[2] ))
        #    marker_set_data.add_unlabeled_marker(pos)
        return offset, marker_set_data

    def __unpack_rigid_body_data( self, data, packet_size, major, minor):
        rigid_body_data = MoCapData.RigidBodyData()
        offset = 0
        # Rigid body count (4 bytes)
        rigid_body_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
        offset += 4
        trace_mf( "Rigid Body Count:", rigid_body_count )

        # get data size (4 bytes)
        offset_tmp, unpackedDataSize = self.__unpack_data_size(data[offset:],major, minor)
        offset += offset_tmp

        for i in range( 0, rigid_body_count ):
            offset_tmp, rigid_body = self.__unpack_rigid_body( data[offset:], major, minor, i )
            offset += offset_tmp
            rigid_body_data.add_rigid_body(rigid_body)

        return offset, rigid_body_data


    def __unpack_skeleton_data( self, data, packet_size, major, minor):
        skeleton_data = MoCapData.SkeletonData()

        offset = 0
        # Version 2.1 and later
        skeleton_count = 0
        if( ( major == 2 and minor > 0 ) or major > 2 ):
            skeleton_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
            offset += 4
            trace_mf( "Skeleton Count:", skeleton_count )
            
            # get data size (4 bytes)
            offset_tmp, unpackedDataSize = self.__unpack_data_size(data[offset:],major, minor)
            offset += offset_tmp
            if(skeleton_count >0):
                for skeleton_num in range( 0, skeleton_count ):
                    rel_offset, skeleton = self.__unpack_skeleton( data[offset:], major, minor, skeleton_num )
                    offset += rel_offset
                    skeleton_data.add_skeleton(skeleton)

        return offset, skeleton_data

    def __decode_marker_id(self, new_id):
        model_id = 0
        marker_id = 0
        model_id = new_id >> 16
        marker_id = new_id & 0x0000ffff
        return model_id, marker_id

    def __unpack_labeled_marker_data( self, data, packet_size, major, minor):
        labeled_marker_data = MoCapData.LabeledMarkerData()
        offset = 0
        # Labeled markers (Version 2.3 and later)
        labeled_marker_count = 0
        if( ( major == 2 and minor > 3 ) or major > 2 ):
            labeled_marker_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
            offset += 4
            trace_mf( "Labeled Marker Count:", labeled_marker_count )

            # get data size (4 bytes)
            offset_tmp, unpackedDataSize = self.__unpack_data_size(data[offset:],major, minor)
            offset += offset_tmp

            for lm_num in range( 0, labeled_marker_count ):
                model_id = 0
                marker_id = 0
                tmp_id = int.from_bytes( data[offset:offset+4], byteorder='little' )
                offset += 4
                model_id, marker_id = self.__decode_marker_id(tmp_id)
                pos = Vector3.unpack( data[offset:offset+12] )
                offset += 12
                size = FloatValue.unpack( data[offset:offset+4] )
                offset += 4
                trace_mf("%3.1d ID     : [MarkerID: %3.1d] [ModelID: %3.1d]"%(lm_num, marker_id,model_id))
                trace_mf("    pos  : [%3.2f, %3.2f, %3.2f]"%(pos[0],pos[1],pos[2]))
                trace_mf("    size : [%3.2f]"%size)


                # Version 2.6 and later
                param = 0
                if( ( major == 2 and minor >= 6 ) or major > 2):
                    param, = struct.unpack( 'h', data[offset:offset+2] )
                    offset += 2
                    #occluded = ( param & 0x01 ) != 0
                    #point_cloud_solved = ( param & 0x02 ) != 0
                    #model_solved = ( param & 0x04 ) != 0

                # Version 3.0 and later
                residual = 0.0
                if major >= 3 :
                    residual, = FloatValue.unpack( data[offset:offset+4] )
                    offset += 4
                    residual = residual * 1000.0
                    trace_mf( "    err  : [%3.2f]"% residual )

                labeled_marker = MoCapData.LabeledMarker(tmp_id,pos,size,param, residual)
                labeled_marker_data.add_labeled_marker(labeled_marker)

        return offset, labeled_marker_data

    def __unpack_force_plate_data( self, data, packet_size, major, minor):
        force_plate_data = MoCapData.ForcePlateData()
        n_frames_show_max = 4
        offset = 0
        # Force Plate data (version 2.9 and later)
        force_plate_count = 0
        if( ( major == 2 and minor >= 9 ) or major > 2 ):
            force_plate_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
            offset += 4
            trace_mf( "Force Plate Count:", force_plate_count )

            # get data size (4 bytes)
            offset_tmp, unpackedDataSize = self.__unpack_data_size(data[offset:],major, minor)
            offset += offset_tmp

            for i in range( 0, force_plate_count ):
                # ID
                force_plate_id = int.from_bytes( data[offset:offset+4], byteorder='little' )
                offset += 4
                force_plate = MoCapData.ForcePlate(force_plate_id)

                # Channel Count
                force_plate_channel_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
                offset += 4

                trace_mf( "\tForce Plate %3.1d ID: %3.1d Num Channels: %3.1d"% (i, force_plate_id, force_plate_channel_count ))

                # Channel Data
                for j in range( force_plate_channel_count ):
                    fp_channel_data = MoCapData.ForcePlateChannelData()
                    force_plate_channel_frame_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
                    offset += 4
                    out_string="\tChannel %3.1d: "%( j )
                    out_string+="  %3.1d Frames - Frame Data: "%(force_plate_channel_frame_count)

                    # Force plate frames
                    n_frames_show = min(force_plate_channel_frame_count, n_frames_show_max)
                    for k in range( force_plate_channel_frame_count ):
                        force_plate_channel_val = FloatValue.unpack( data[offset:offset+4] )
                        offset += 4
                        fp_channel_data.add_frame_entry(force_plate_channel_val)

                        if k < n_frames_show:
                            out_string += "%3.2f "%(force_plate_channel_val)
                    if n_frames_show < force_plate_channel_frame_count:
                        out_string += " showing %3.1d of %3.1d frames"%(n_frames_show, force_plate_channel_frame_count)
                    trace_mf( "%s"% out_string )
                    force_plate.add_channel_data(fp_channel_data)
                force_plate_data.add_force_plate(force_plate)
        return offset, force_plate_data

    def __unpack_device_data( self, data, packet_size, major, minor):
        device_data = MoCapData.DeviceData()
        n_frames_show_max = 4
        offset = 0
        # Device data (version 2.11 and later)
        device_count = 0
        if ( major == 2 and minor >= 11 ) or (major > 2) :
            device_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
            offset += 4
            trace_mf( "Device Count:", device_count )

            # get data size (4 bytes)
            offset_tmp, unpackedDataSize = self.__unpack_data_size(data[offset:],major, minor)
            offset += offset_tmp

            for i in range( 0, device_count ):

                # ID
                device_id = int.from_bytes( data[offset:offset+4], byteorder='little' )
                offset += 4
                device = MoCapData.Device(device_id)
                # Channel Count
                device_channel_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
                offset += 4

                trace_mf( "\tDevice %3.1d      ID: %3.1d Num Channels: %3.1d"% (i, device_id, device_channel_count ))

                # Channel Data
                for j in range( 0, device_channel_count ):
                    device_channel_data = MoCapData.DeviceChannelData()
                    device_channel_frame_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
                    offset += 4
                    out_string="\tChannel %3.1d "% (j)
                    out_string+="  %3.1d Frames - Frame Data: "%(device_channel_frame_count)

                    # Device Frame Data
                    n_frames_show = min(device_channel_frame_count, n_frames_show_max)
                    for k in range( 0, device_channel_frame_count ):
                        device_channel_val = int.from_bytes( data[offset:offset+4], byteorder='little' )
                        device_channel_val = FloatValue.unpack( data[offset:offset+4] )
                        offset += 4
                        if k < n_frames_show:
                            out_string += "%3.2f "%(device_channel_val)

                        device_channel_data.add_frame_entry(device_channel_val)
                    if n_frames_show < device_channel_frame_count:
                        out_string += " showing %3.1d of %3.1d frames"%(n_frames_show, device_channel_frame_count)
                    trace_mf( "%s"% out_string )
                    device.add_channel_data(device_channel_data)
                device_data.add_device(device)
        return offset, device_data

    def __unpack_frame_suffix_data( self, data, packet_size, major, minor):
        frame_suffix_data = MoCapData.FrameSuffixData()
        offset = 0

        # Timecode
        timecode = int.from_bytes( data[offset:offset+4], byteorder='little' )
        offset += 4
        frame_suffix_data.timecode = timecode

        timecode_sub = int.from_bytes( data[offset:offset+4], byteorder='little' )
        offset += 4
        frame_suffix_data.timecode_sub = timecode_sub

        param=0
        #check to see if there is enough data
        if((packet_size-offset) <= 0):
            print("ERROR: Early End of Data Frame Suffix Data")
            print("\tNo time stamp info available")
        else:
            # Timestamp (increased to double precision in 2.7 and later)
            if ( major == 2 and minor >= 7 ) or (major > 2 ):
                timestamp, = DoubleValue.unpack( data[offset:offset+8] )
                offset += 8
            else:
                timestamp, = FloatValue.unpack( data[offset:offset+4] )
                offset += 4
            trace_mf("Timestamp : %3.2f"%timestamp)
            frame_suffix_data.timestamp = timestamp

            # Hires Timestamp (Version 3.0 and later)
            if major >= 3 :
                stamp_camera_mid_exposure = int.from_bytes( data[offset:offset+8], byteorder='little' )
                trace_mf("Mid-exposure timestamp         : %3.1d"%stamp_camera_mid_exposure)
                offset += 8
                frame_suffix_data.stamp_camera_mid_exposure = stamp_camera_mid_exposure

                stamp_data_received = int.from_bytes( data[offset:offset+8], byteorder='little' )
                offset += 8
                frame_suffix_data.stamp_data_received = stamp_data_received
                trace_mf("Camera data received timestamp : %3.1d"%stamp_data_received)

                stamp_transmit = int.from_bytes( data[offset:offset+8], byteorder='little' )
                offset += 8
                trace_mf("Transmit timestamp             : %3.1d"%stamp_transmit)
                frame_suffix_data.stamp_transmit = stamp_transmit

            # Precision Timestamp (Version 4.1 and later) (defaults as 0 if N/A)
            if major >= 4:
                prec_timestamp_secs = int.from_bytes( data[offset:offset+4], byteorder='little' )
                hours = int(prec_timestamp_secs/3600)
                minutes=int(prec_timestamp_secs/60)%60
                seconds=prec_timestamp_secs%60
                out_string="Precision timestamp (h:m:s) - %4.1d:%2.2d:%2.2d"%(hours, minutes, seconds)
                trace_mf("%s"%out_string)
                trace_mf("Precision timestamp (sec)      : %3.1d"%prec_timestamp_secs)
                offset += 4
                frame_suffix_data.prec_timestamp_secs = prec_timestamp_secs

                prec_timestamp_frac_secs = int.from_bytes( data[offset:offset+4], byteorder='little' )
                trace_mf("Precision timestamp (frac sec) : %3.1d"%prec_timestamp_frac_secs)
                offset += 4
                frame_suffix_data.prec_timestamp_frac_secs = prec_timestamp_frac_secs

            # Frame parameters
            param, = struct.unpack( 'h', data[offset:offset+2] )
            offset += 2
        is_recording = ( param & 0x01 ) != 0
        tracked_models_changed = ( param & 0x02 ) != 0
        frame_suffix_data.param = param
        frame_suffix_data.is_recording = is_recording
        frame_suffix_data.tracked_models_changed = tracked_models_changed

        return offset, frame_suffix_data


    # Unpack data from a motion capture frame message
    def __unpack_mocap_data( self, data : bytes, packet_size, major, minor):
        mocap_data = MoCapData.MoCapData()
        data = memoryview( data )
        offset = 0
        rel_offset = 0
        
        #Frame Prefix Data
        rel_offset, frame_prefix_data = self.__unpack_frame_prefix_data(data[offset:])
        offset += rel_offset
        mocap_data.set_prefix_data(frame_prefix_data)
        frame_number = frame_prefix_data.frame_number

        #Marker Set Data
        rel_offset, marker_set_data =self.__unpack_marker_set_data(data[offset:], (packet_size - offset),major, minor)
        offset += rel_offset
        mocap_data.set_marker_set_data(marker_set_data)
        marker_set_count = marker_set_data.get_marker_set_count()
        unlabeled_markers_count = marker_set_data.get_unlabeled_marker_count()

        # Legacy Other Markers
        rel_offset, legacy_other_markers =self.__unpack_legacy_other_markers(data[offset:], (packet_size - offset),major, minor)
        offset += rel_offset
        mocap_data.set_legacy_other_markers(legacy_other_markers)
        marker_set_count = legacy_other_markers.get_marker_count()
        legacy_other_markers_count = marker_set_data.get_unlabeled_marker_count()

        # Rigid Body Data
        rel_offset, rigid_body_data = self.__unpack_rigid_body_data(data[offset:], (packet_size - offset),major, minor)
        offset += rel_offset
        mocap_data.set_rigid_body_data(rigid_body_data)
        rigid_body_count = rigid_body_data.get_rigid_body_count()

        # Skeleton Data
        rel_offset, skeleton_data = self.__unpack_skeleton_data(data[offset:], (packet_size - offset),major, minor)
        offset += rel_offset
        mocap_data.set_skeleton_data(skeleton_data)
        skeleton_count = skeleton_data.get_skeleton_count()

        # Assets ( Motive 3.1/NatNet 4.1 and greater)
        if (((major == 4) and (minor > 0)) or (major > 4)):
            rel_offset, asset_data = self.__unpack_asset_data(data[offset:], (packet_size - offset),major, minor)
            offset += rel_offset
            mocap_data.set_asset_data(asset_data)
            asset_rigid_body_count = asset_data.get_rigid_body_count()
            asset_marker_count = asset_data.get_marker_count()

        # Labeled Marker Data
        rel_offset, labeled_marker_data = self.__unpack_labeled_marker_data(data[offset:], (packet_size - offset),major, minor)
        offset += rel_offset
        mocap_data.set_labeled_marker_data(labeled_marker_data)
        labeled_marker_count = labeled_marker_data.get_labeled_marker_count()

        # Force Plate Data
        rel_offset, force_plate_data = self.__unpack_force_plate_data(data[offset:], (packet_size - offset),major, minor)
        offset += rel_offset
        mocap_data.set_force_plate_data(force_plate_data)

        # Device Data
        rel_offset,device_data = self.__unpack_device_data(data[offset:], (packet_size - offset),major, minor)
        offset += rel_offset
        mocap_data.set_device_data(device_data)

        # Frame Suffix Data
        #rel_offset, timecode, timecode_sub, timestamp, is_recording, tracked_models_changed = \
        rel_offset, frame_suffix_data =   self.__unpack_frame_suffix_data(data[offset:], (packet_size - offset),major, minor)
        offset += rel_offset
        mocap_data.set_suffix_data(frame_suffix_data)


        timecode = frame_suffix_data.timecode
        timecode_sub= frame_suffix_data.timecode_sub
        timestamp = frame_suffix_data.timestamp
        is_recording = frame_suffix_data.is_recording
        tracked_models_changed = frame_suffix_data.tracked_models_changed

        # Send information to any listener.
        if self.new_frame_listener is not None:
            data_dict={}
            data_dict["frame_number"]=frame_number
            data_dict[ "marker_set_count"] = marker_set_count
            data_dict[ "unlabeled_markers_count"] = unlabeled_markers_count
            data_dict[ "rigid_body_count"] = rigid_body_count
            data_dict[ "skeleton_count"] =skeleton_count
            data_dict[ "labeled_marker_count"] = labeled_marker_count
            data_dict[ "timecode"] = timecode
            data_dict[ "timecode_sub"] = timecode_sub
            data_dict[ "timestamp"] = timestamp
            data_dict[ "is_recording"] = is_recording
            data_dict[ "tracked_models_changed"] = tracked_models_changed

            self.new_frame_listener( data_dict )

        if self.rigid_body_list_listener is not None:
            self.rigid_body_list_listener(rigid_body_data, timestamp)
        
        if self.marker_set_listener is not None:
            self.marker_set_listener(marker_set_data)

        return offset, mocap_data


    # Unpack a marker set description packet
    def __unpack_marker_set_description( self, data, major, minor):
        ms_desc = DataDescriptions.MarkerSetDescription()

        offset = 0

        name, separator, remainder = bytes(data[offset:]).partition( b'\0' )
        offset += len( name ) + 1
        trace_dd( "Marker Set Name: %s" % (name.decode( 'utf-8' )) )
        ms_desc.set_name(name)

        marker_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
        offset += 4
        trace_dd( "Marker Count : %3.1d" % marker_count)
        if(marker_count > 0):
            for i in range( 0, marker_count ):
                name, separator, remainder = bytes(data[offset:]).partition( b'\0' )
                offset += len( name ) + 1
                trace_dd( "\t%2.1d Marker Name: %s"%(i, name.decode( 'utf-8' ) ))
                ms_desc.add_marker_name(name)

        return offset, ms_desc

    # Unpack a rigid body description packet
    def __unpack_rigid_body_description( self, data, major, minor):
        rb_desc=DataDescriptions.RigidBodyDescription()
        offset = 0

        # Version 2.0 or higher
        if (major >= 2) or (major == 0):
            name, separator, remainder = bytes(data[offset:]).partition( b'\0' )
            offset += len( name ) + 1
            rb_desc.set_name(name)
            trace_dd( "\tRigid Body Name   : ", name.decode( 'utf-8' ) )

        # ID
        new_id = int.from_bytes( data[offset:offset+4], byteorder='little' )
        offset += 4
        rb_desc.set_id(new_id)
        trace_dd( "\tID                : ", str(new_id))

        #Parent ID
        parent_id = int.from_bytes( data[offset:offset+4], byteorder='little' )
        offset += 4
        rb_desc.set_parent_id(parent_id)
        trace_dd( "\tParent ID         : ", parent_id)

        # Position Offsets
        pos = Vector3.unpack( data[offset:offset+12] )
        offset += 12
        rb_desc.set_pos(pos[0],pos[1],pos[2])

        trace_dd( "\tPosition          : [%3.2f, %3.2f, %3.2f]"% (pos[0], pos[1], pos[2] ))

        # Version 3.0 and higher, rigid body marker information contained in description
        if (major >= 3) or (major == 0) :
            # Marker Count
            marker_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
            offset += 4
            trace_dd( "\tNumber of Markers : ", marker_count )

            marker_count_range = range( 0, marker_count )
            offset1 = offset
            offset2 = offset1 + (12*marker_count)
            offset3 = offset2 + (4*marker_count)
            # Marker Offsets X,Y,Z
            marker_name=""
            for marker in marker_count_range:
                # Offset
                marker_offset = Vector3.unpack(data[offset1:offset1+12])
                offset1 +=12

                # Active Label
                active_label = int.from_bytes(data[offset2:offset2+4],byteorder = 'little')
                offset2 += 4

                #Marker Name
                if (major >= 4) or (major == 0):
                    # markername
                    marker_name, separator, remainder = bytes(data[offset3:]).partition( b'\0' )
                    marker_name = marker_name.decode( 'utf-8' )
                    offset3 += len( marker_name ) + 1

                rb_marker=DataDescriptions.RBMarker(marker_name,active_label,marker_offset)
                rb_desc.add_rb_marker(rb_marker)
                trace_dd( "\t%3.1d Marker Label: %s Position: [x=%3.2f,y=%3.2f,z=%3.2f] %s" % (marker,active_label,\
                   marker_offset[0], marker_offset[1], marker_offset[2],marker_name ))

            offset = offset3
        
        trace_dd("\tunpack_rigid_body_description processed bytes: ", offset)
        return offset, rb_desc

    # Unpack a skeleton description packet
    def __unpack_skeleton_description( self, data, major, minor):
        skeleton_desc = DataDescriptions.SkeletonDescription()
        offset = 0

        #Name
        name, separator, remainder = bytes(data[offset:]).partition( b'\0' )
        offset += len( name ) + 1
        skeleton_desc.set_name(name)
        trace_dd( "Name : %s"% name.decode( 'utf-8' ) )

        #ID
        new_id = int.from_bytes( data[offset:offset+4], byteorder='little' )
        offset += 4
        skeleton_desc.set_id(new_id)
        trace_dd( "ID : %3.1d"% new_id )

        # # of RigidBodies
        rigid_body_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
        offset += 4
        trace_dd( "Rigid Body (Bone) Count : %3.1d" % rigid_body_count)

        # Loop over all Rigid Bodies
        for i in range( 0, rigid_body_count ):
            trace_dd("Rigid Body (Bone) ", i)
            offset_tmp, rb_desc_tmp = self.__unpack_rigid_body_description( data[offset:], major, minor )
            offset+= offset_tmp
            skeleton_desc.add_rigid_body_description(rb_desc_tmp)
        return offset, skeleton_desc

    def __unpack_force_plate_description(self, data, major, minor):
        fp_desc = None
        offset = 0
        if major >= 3:
            fp_desc = DataDescriptions.ForcePlateDescription()
            # ID
            new_id = int.from_bytes( data[offset:offset+4], byteorder='little' )
            offset += 4
            fp_desc.set_id(new_id)
            trace_dd("\tID : ", str(new_id))

            # Serial Number
            serial_number, separator, remainder = bytes(data[offset:]).partition( b'\0' )
            offset += len( serial_number ) + 1
            fp_desc.set_serial_number(serial_number)
            trace_dd( "\tSerial Number : ", serial_number.decode( 'utf-8' ) )

            # Dimensions
            f_width = FloatValue.unpack( data[offset:offset+4])
            offset += 4
            trace_dd( "\tWidth  : %3.2f"% f_width)
            f_length = FloatValue.unpack( data[offset:offset+4])
            offset += 4
            fp_desc.set_dimensions(f_width[0], f_length[0])
            trace_dd( "\tLength : %3.2f"% f_length)

            # Origin
            origin = Vector3.unpack( data[offset:offset+12] )
            offset += 12
            fp_desc.set_origin(origin[0],origin[1],origin[2])
            trace_dd( "\tOrigin : %3.2f, %3.2f, %3.2f"%( origin[0], origin[1], origin[2] ))

            # Calibration Matrix 12x12 floats
            trace_dd("Cal Matrix:")
            cal_matrix_tmp= [[0.0 for col in range(12)] for row in range(12)]

            for i in range(0,12):
                cal_matrix_row=FPCalMatrixRow.unpack(data[offset:offset+(12*4)])
                trace_dd("\t%3.1d %3.3e %3.3e %3.3e %3.3e %3.3e %3.3e %3.3e %3.3e %3.3e %3.3e %3.3e %3.3e" % (i
                      , cal_matrix_row[0], cal_matrix_row[1], cal_matrix_row[2], cal_matrix_row[3]
                      , cal_matrix_row[4], cal_matrix_row[5], cal_matrix_row[6], cal_matrix_row[7]
                      , cal_matrix_row[8], cal_matrix_row[9], cal_matrix_row[10], cal_matrix_row[11]))
                cal_matrix_tmp[i] = copy.deepcopy(cal_matrix_row)
                offset += (12*4)
            fp_desc.set_cal_matrix(cal_matrix_tmp)
            # Corners 4x3 floats
            corners = FPCorners.unpack(data[offset:offset + (12*4)])
            offset += (12*4)
            o_2=0
            trace_dd("Corners:")
            corners_tmp = [[0.0 for col in range(3)] for row in range(4)]
            for i in range(0,4):
                trace_dd("\t%3.1d %3.3e %3.3e %3.3e"%(i, corners[o_2], corners[o_2+1], corners[o_2+2]))
                corners_tmp[i][0]=corners[o_2]
                corners_tmp[i][1]=corners[o_2+1]
                corners_tmp[i][2]=corners[o_2+2]
                o_2+=3
            fp_desc.set_corners(corners_tmp)

            # Plate Type int
            plate_type = int.from_bytes( data[offset:offset+4], byteorder='little' )
            offset+=4
            fp_desc.set_plate_type(plate_type)
            trace_dd ("Plate Type : ", plate_type)

            # Channel Data Type int
            channel_data_type = int.from_bytes( data[offset:offset+4], byteorder='little' )
            offset+=4
            fp_desc.set_channel_data_type(channel_data_type)
            trace_dd("Channel Data Type : ", channel_data_type)

            # Number of Channels int
            num_channels = int.from_bytes( data[offset:offset+4], byteorder='little' )
            offset+=4
            trace_dd("Number of Channels : ", num_channels)

            # Channel Names list of NoC strings
            for i in range(0, num_channels):
                channel_name, separator, remainder = bytes(data[offset:]).partition( b'\0' )
                offset += len( channel_name ) + 1
                trace_dd( "\tChannel Name %3.1d: %s"%(i, channel_name.decode( 'utf-8' ) ))
                fp_desc.add_channel_name(channel_name)

        trace_dd("unpackForcePlate processed ", offset, " bytes")
        return offset, fp_desc

    def __unpack_device_description(self, data, major, minor):
        device_desc=None
        offset = 0
        if major >= 3:
            # new_id
            new_id = int.from_bytes( data[offset:offset+4], byteorder='little' )
            offset += 4
            trace_dd("\tID : ", str(new_id))

            # Name
            name, separator, remainder = bytes(data[offset:]).partition( b'\0' )
            offset += len( name ) + 1
            trace_dd( "\tName : ", name.decode( 'utf-8' ) )

            # Serial Number
            serial_number, separator, remainder = bytes(data[offset:]).partition( b'\0' )
            offset += len( serial_number ) + 1
            trace_dd( "\tSerial Number : ", serial_number.decode( 'utf-8' ) )


            # Device Type int
            device_type = int.from_bytes( data[offset:offset+4], byteorder='little' )
            offset+=4
            trace_dd ("Device Type : ", device_type)

            # Channel Data Type int
            channel_data_type = int.from_bytes( data[offset:offset+4], byteorder='little' )
            offset+=4
            trace_dd("Channel Data Type : ", channel_data_type)

            device_desc = DataDescriptions.DeviceDescription(new_id,name,serial_number,device_type,channel_data_type)

            # Number of Channels int
            num_channels = int.from_bytes( data[offset:offset+4], byteorder='little' )
            offset+=4
            trace_dd("Number of Channels ", num_channels)

            # Channel Names list of NoC strings
            for i in range(0, num_channels):
                channel_name, separator, remainder = bytes(data[offset:]).partition( b'\0' )
                offset += len( channel_name ) + 1
                device_desc.add_channel_name(channel_name)
                trace_dd( "\tChannel ",i," Name : ", channel_name.decode( 'utf-8' ) )

        trace_dd("unpack_device_description processed ", offset, " bytes")
        return offset, device_desc

    def __unpack_camera_description(self, data, major, minor):
        offset = 0
        # Name
        name, separator, remainder = bytes(data[offset:]).partition( b'\0' )
        offset += len( name ) + 1
        trace_dd( "\tName       : %s"% name.decode( 'utf-8' ) )
        # Position
        position = Vector3.unpack( data[offset:offset+12] )
        offset += 12
        trace_dd( "\tPosition   : [%3.2f, %3.2f, %3.2f]"% (position[0], position[1], position[2] ))

        # Orientation
        orientation = Quaternion.unpack( data[offset:offset+16] )
        offset += 16
        trace_dd( "\tOrientation: [%3.2f, %3.2f, %3.2f, %3.2f]"% (orientation[0], orientation[1], orientation[2], orientation[3] ))
        trace_dd("unpack_camera_description processed %3.1d bytes"% offset)

        camera_desc=DataDescriptions.CameraDescription(name, position, orientation)
        return offset, camera_desc


    def __unpack_marker_description( self, data, major, minor ):
        offset = 0

        # Name
        name, separator, remainder = bytes(data[offset:]).partition( b'\0' )
        offset += len( name ) + 1
        trace_dd( "\tName       : %s"% name.decode( 'utf-8' ) )

        # ID
        marker_id =  data[offset:offset+4]
        offset += 4
        trace_dd( "\tID         : %d"% (marker_id ))

        # Initial Position
        initialPosition = Vector3.unpack( data[offset:offset+12] )
        offset += 12
        trace_dd( "\tPosition   : [%3.2f, %3.2f, %3.2f]"% (initialPosition[0], initialPosition[1], initialPosition[2] ))

        # Size
        marker_size = FloatValue.unpack( data[offset:offset+4] )
        offset += 4
        trace_mf( "\tMarker Size:", marker_size )

        # Params
        marker_params, = struct.unpack( 'h', data[offset:offset+2] )
        offset += 2
        trace_mf( "\tParams     :", marker_params )

        trace_dd("unpack_marker_description processed %3.1d bytes"% offset)

        # Package for return object
        marker_desc=DataDescriptions.MarkerDescription(name, marker_id, initialPosition, marker_size, marker_params)
        return offset, marker_desc

    def __unpack_asset_rigid_body_data( self, data, major, minor ):
        offset = 0
        # ID
        rbID =  data[offset:offset+4]
        offset += 4
        trace_dd( "\tID         : %d"% (rbID ))

        # Position: x,y,z
        pos = Vector3.unpack( data[offset:offset+12] )
        offset += 12
        trace_mf( "\tPosition    : [%3.2f, %3.2f, %3.2f]"% (pos[0], pos[1], pos[2] ))

        # Orientation: qx, qy, qz, qw
        rot = Quaternion.unpack( data[offset:offset+16] )
        offset += 16
        trace_mf( "\tOrientation : [%3.2f, %3.2f, %3.2f, %3.2f]"% (rot[0], rot[1], rot[2], rot[3] ))

        # Mean error
        mean_error, = FloatValue.unpack( data[offset:offset+4] )
        offset += 4
        trace_mf( "\tMean Error  : %3.2f"% mean_error )

        # Params
        marker_params, = struct.unpack( 'h', data[offset:offset+2] )
        offset += 2
        trace_mf( "\tParams      :", marker_params )

        trace_dd("unpack_marker_description processed %3.1d bytes"% offset)
                
        # Package for return object
        rigid_body_data=MoCapData.AssetRigidBodyData(rbID, pos, rot, mean_error, marker_params)

        return offset, rigid_body_data

    def __unpack_asset_marker_data( self, data, major, minor ):
        # ID
        marker_id =  data[offset:offset+4]
        offset += 4
        trace_dd( "\tID          : %d"% (marker_id ))

        # Position: x,y,z
        pos = Vector3.unpack( data[offset:offset+12] )
        offset += 12
        trace_mf( "\tPosition    : [%3.2f, %3.2f, %3.2f]"% (pos[0], pos[1], pos[2] ))

        # Size
        marker_size, = FloatValue.unpack( data[offset:offset+4] )
        offset += 4
        trace_mf( "\tMarker Size : %3.2f"% marker_size )

        # Params
        marker_params, = struct.unpack( 'h', data[offset:offset+2] )
        offset += 2
        trace_mf( "\tParams      :", marker_params )

        # Residual
        residual, = FloatValue.unpack( data[offset:offset+4] )
        offset += 4
        trace_mf( "\tResidual    : %3.2f"% residual )

        marker_data = DataDescriptions.AssetMarkerData(marker_id, pos, marker_size, marker_params, residual)
        return offset, marker_data

    def __unpack_asset_data( self, data, packet_size, major, minor):
        asset_data = MoCapData.AssetData()

        offset = 0

        # Asset Count
        asset_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
        offset += 4
        trace_mf( "Asset Count:", asset_count )

        # Get data size (4 bytes)
        offset_tmp, unpackedDataSize = self.__unpack_data_size(data[offset:],major, minor)
        offset += offset_tmp

        # Unpack assets 
        for _ in range( 0, asset_count ):
            # Asset ID 4 bytes
            assetID =  data[offset:offset+4]
            offset += 4
            trace_dd( "\tAsset ID     : %d"% (assetID ))
            asset_data.set_id(assetID)

            # # of RigidBodies
            numRBs =  data[offset:offset+4]
            offset += 4
            trace_dd( "\tRigid Bodies : %d" % (numRBs))
        
            offset1=0
            for rbNum in range(numRBs):
                # # of RigidBodies
                offset1,rigid_body = self.__unpack_asset_rigid_body_data(data+offset, major, minor)
                offset += offset1
                asset_data.add_rigid_body(rigid_body)


            # # of Markers
            numMarkers =  data[offset:offset+4]
            offset += 4
            trace_dd( "\tMarkers      : %d" % (numMarkers))
        
            for markerNum in range(numMarkers):
                # # of Markers
                offset1,marker = self.__unpack_asset_marker_data( data+offset, major, minor)
                offset += offset1
                asset_data.add_marker(marker)

        return offset, asset_data

    def __unpack_asset_description(self, data, major, minor):
        offset = 0

        # Name
        name, separator, remainder = bytes(data[offset:]).partition( b'\0' )
        offset += len( name ) + 1
        trace_dd( "\tName       : %s"% name.decode( 'utf-8' ) )

        # Asset Type 4 bytes
        assetType =  data[offset:offset+4]
        offset += 4
        trace_dd( "\tType       : %d"% (assetType ))

        # ID 4 bytes
        assetID =  data[offset:offset+4]
        offset += 4
        trace_dd( "\tID         : %d"% (assetID ))

        # # of RigidBodies
        numRBs =  data[offset:offset+4]
        offset += 4
        trace_dd( "\tNumber of RBs: %d" % (numRBs))
        
        rigidbodyArray=[]
        offset1=0
        for rbNum in range(numRBs):
            # # of RigidBodies
            offset1,rigidbody = self.__unpack_rigid_body_description(data+offset, major, minor)
            offset += offset1
            rigidbodyArray.append(rigidbody)


        # # of Markers
        numMarkers =  data[offset:offset+4]
        offset += 4
        trace_dd( "\tnumber of Markers: %d" % (numMarkers))
        
        markerArray=[]
        for markerNum in range(numMarkers):
            # # of Markers
            offset1,marker = self.__unpack_marker_description( data+offset, major, minor)
            offset += offset1
            markerArray.append(marker)

        trace_dd("unpack_asset_description processed %3.1d bytes"% offset)

        # package for output
        asset_desc=DataDescriptions.AssetDescription(name, assetType, assetID, rigidbodyArray, markerArray)
        return offset, asset_desc


    # Unpack a data description packet
    def __unpack_data_descriptions( self, data : bytes, packet_size, major, minor):
        data_descs = DataDescriptions.DataDescriptions()
        offset = 0
        # # of data sets to process
        dataset_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
        offset += 4
        trace_dd("Dataset Count : ", str(dataset_count))
        for i in range( 0, dataset_count ):
            trace_dd("Dataset ", str(i))
            data_type = int.from_bytes( data[offset:offset+4], byteorder='little' )
            offset += 4
            data_tmp=None
            if data_type == 0 :
                trace_dd("Type: 0 Markerset")
                offset_tmp, data_tmp = self.__unpack_marker_set_description( data[offset:], major, minor )
            elif data_type == 1 :
                trace_dd("Type: 1 Rigid Body")
                offset_tmp, data_tmp = self.__unpack_rigid_body_description( data[offset:], major, minor )
            elif data_type == 2 :
                trace_dd("Type: 2 Skeleton")
                offset_tmp, data_tmp = self.__unpack_skeleton_description( data[offset:], major, minor )
            elif data_type == 3 :
                trace_dd("Type: 3 Force Plate")
                offset_tmp, data_tmp = self.__unpack_force_plate_description(data[offset:], major, minor)
            elif data_type == 4 :
                trace_dd("Type: 4 Device")
                offset_tmp, data_tmp = self.__unpack_device_description(data[offset:], major, minor)
            elif data_type == 5 :
                trace_dd("Type: 5 Camera")
                offset_tmp, data_tmp = self.__unpack_camera_description(data[offset:], major, minor)
            elif data_type == 6 :
                trace_dd("Type: 6 Asset")
                offset_tmp, data_tmp = self.__unpack_asset_description(data[offset:], major, minor)
            else:
                print("Type: Unknown " + str(data_type))
                print("ERROR: Type decode failure" )
                print("\t"+ str(i + 1) +" datasets processed of " + str(dataset_count))
                print("\t "+ str(offset) +" bytes processed of " + str(packet_size) )
                print("\tPACKET DECODE STOPPED")
                return offset
            offset += offset_tmp
            data_descs.add_data(data_tmp)
            trace_dd("\t"+ str(i) +" datasets processed of " + str(dataset_count))
            trace_dd("\t "+ str(offset) +" bytes processed of " + str(packet_size) )

        return offset, data_descs

    # __unpack_server_info is for local use of the client
    # and will update the values for the versions/ NatNet capabilities
    # of the server.
    def __unpack_server_info(self, data, packet_size, major, minor):
        offset = 0
        # Server name
        #szName = data[offset: offset+256]
        self.__application_name, separator, remainder = bytes(data[offset: offset+256]).partition( b'\0' )
        self.__application_name=str(self.__application_name, "utf-8")
        offset += 256
        # Server Version info
        server_version = struct.unpack( 'BBBB', data[offset:offset+4] )
        offset += 4
        self.__server_version[0] = server_version[0]
        self.__server_version[1] = server_version[1]
        self.__server_version[2] = server_version[2]
        self.__server_version[3] = server_version[3]

        # NatNet Version info
        nnsvs = struct.unpack( 'BBBB', data[offset:offset+4] )
        offset += 4
        self.__nat_net_stream_version_server[0]=nnsvs[0]
        self.__nat_net_stream_version_server[1]=nnsvs[1]
        self.__nat_net_stream_version_server[2]=nnsvs[2]
        self.__nat_net_stream_version_server[3]=nnsvs[3]
        if (self.__nat_net_requested_version[0] == 0) and\
           (self.__nat_net_requested_version[1] == 0):
            trace_mf("resetting requested version to %d %d %d %d from %d %d %d %d"%(
                self.__nat_net_stream_version_server[0],
                self.__nat_net_stream_version_server[1],
                self.__nat_net_stream_version_server[2],
                self.__nat_net_stream_version_server[3],
                self.__nat_net_requested_version[0],
                self.__nat_net_requested_version[1],
                self.__nat_net_requested_version[2],
                self.__nat_net_requested_version[3]))

            self.__nat_net_requested_version[0] = self.__nat_net_stream_version_server[0]
            self.__nat_net_requested_version[1] = self.__nat_net_stream_version_server[1]
            self.__nat_net_requested_version[2] = self.__nat_net_stream_version_server[2]
            self.__nat_net_requested_version[3] = self.__nat_net_stream_version_server[3]
            # Determine if the bitstream version can be changed
            if (self.__nat_net_stream_version_server[0] >= 4) and (self.use_multicast == False):
                self.__can_change_bitstream_version = True



        trace_mf("Sending Application Name: ", self.__application_name)
        trace_mf("NatNetVersion " , str(self.__nat_net_stream_version_server[0]), " "
            , str(self.__nat_net_stream_version_server[1]), " "
            , str(self.__nat_net_stream_version_server[2]), " "
                , str(self.__nat_net_stream_version_server[3]))

        trace_mf("ServerVersion " , str(self.__server_version[0]), " "
            , str(self.__server_version[1]), " "
            , str(self.__server_version[2]), " "
                , str(self.__server_version[3]) )
        return offset

    # __unpack_bitstream_info is for local use of the client
    # and will update the values for the current bitstream
    # of the server.

    def __unpack_bitstream_info(self, data, packet_size, major, minor):
        nn_version=[]
        inString = data.decode('utf-8')
        messageList = inString.split(',')
        if( len(messageList) > 1 ):
            if( messageList[0] == 'Bitstream'):
                nn_version=messageList[1].split('.')
        return nn_version

    def __command_thread_function( self, in_socket, stop, gprint_level):
        message_id_dict={}
        if not self.use_multicast:
            in_socket.settimeout(2.0)
        data=bytearray(0)
        # 64k buffer size
        recv_buffer_size=64*1024
        while not stop():
            # Block for input
            try:
                data, addr = in_socket.recvfrom( recv_buffer_size )
            except socket.error as msg:
                if stop():
                    #print("ERROR: command socket access error occurred:\n  %s" %msg)
                    #return 1
                    print("shutting down")
            except  socket.herror:
                print("ERROR: command socket access herror occurred")
                return 2
            except  socket.gaierror:
                print("ERROR: command socket access gaierror occurred")
                return 3
            except  socket.timeout:
                if(self.use_multicast):
                    print("ERROR: command socket access timeout occurred. Server not responding")
                    #return 4

            if len( data ) > 0 :
                #peek ahead at message_id
                message_id = get_message_id(data)
                tmp_str="mi_%1.1d"%message_id
                if tmp_str not in message_id_dict:
                    message_id_dict[tmp_str]=0
                message_id_dict[tmp_str] += 1
                
                print_level = gprint_level()
                if message_id == self.NAT_FRAMEOFDATA:
                    if print_level > 0:
                        if (message_id_dict[tmp_str] % print_level) == 0:
                            print_level = 1
                        else:
                            print_level = 0
                message_id = self.__process_message( data , print_level)

                data=bytearray(0)

            if not self.use_multicast:
                if not stop():
                    self.send_keep_alive(in_socket, self.server_ip_address, self.command_port)
        return 0

    def __data_thread_function( self, in_socket, stop, gprint_level):
        message_id_dict={}
        data=bytearray(0)
        # 64k buffer size
        recv_buffer_size=64*1024

        while not stop():
            # Block for input
            try:
                data, addr = in_socket.recvfrom( recv_buffer_size )
            except socket.error as msg:
                if not stop():
                    print("ERROR: data socket access error occurred:\n  %s" %msg)
                    return 1
            except  socket.herror:
                print("ERROR: data socket access herror occurred")
                #return 2
            except  socket.gaierror:
                print("ERROR: data socket access gaierror occurred")
                #return 3
            except  socket.timeout:
                #if self.use_multicast:
                print("ERROR: data socket access timeout occurred. Server not responding")
                #return 4
            if len( data ) > 0 :
                #peek ahead at message_id
                message_id = get_message_id(data)
                tmp_str="mi_%1.1d"%message_id
                if tmp_str not in message_id_dict:
                    message_id_dict[tmp_str]=0
                message_id_dict[tmp_str] += 1
                
                print_level = gprint_level()
                if message_id == self.NAT_FRAMEOFDATA:
                    if print_level > 0:
                        if (message_id_dict[tmp_str] % print_level) == 0:
                            print_level = 1
                        else:
                            print_level = 0
                message_id = self.__process_message( data , print_level)

                data=bytearray(0)
        return 0

    def __process_message( self, data : bytes, print_level=0):
        #return message ID
        major = self.get_major()
        minor = self.get_minor()

        trace( "Begin Packet\n-----------------" )
        show_nat_net_version = False
        if show_nat_net_version:
            trace("NatNetVersion " , str(self.__nat_net_requested_version[0]), " "\
                , str(self.__nat_net_requested_version[1]), " "\
                , str(self.__nat_net_requested_version[2]), " "\
                , str(self.__nat_net_requested_version[3]))

        message_id = get_message_id(data)

        packet_size = int.from_bytes( data[2:4], byteorder='little' )

        #skip the 4 bytes for message ID and packet_size
        offset = 4
        if message_id == self.NAT_FRAMEOFDATA :
            trace( "Message ID  : %3.1d NAT_FRAMEOFDATA"% message_id )
            trace( "Packet Size : ", packet_size )

            offset_tmp, mocap_data = self.__unpack_mocap_data( data[offset:], packet_size, major, minor )
            offset += offset_tmp
            trace("MoCap Frame: %d\n"%(mocap_data.prefix_data.frame_number))
            # get a string version of the data for output
            mocap_data_str=mocap_data.get_as_string()
            if print_level >= 1:
                print("%s\n"%mocap_data_str)

        elif message_id == self.NAT_MODELDEF :
            trace( "Message ID  : %3.1d NAT_MODELDEF"% message_id )
            trace( "Packet Size : %d"% packet_size )
            offset_tmp, data_descs = self.__unpack_data_descriptions( data[offset:], packet_size, major, minor)
            offset += offset_tmp
            print("Data Descriptions:\n")
            # get a string version of the data for output
            data_descs_str=data_descs.get_as_string()
            if print_level>0:
                print("%s\n"%(data_descs_str))

        elif message_id == self.NAT_SERVERINFO :
            trace( "Message ID  : %3.1d NAT_SERVERINFO"% message_id )
            trace( "Packet Size : ", packet_size )
            offset += self.__unpack_server_info( data[offset:], packet_size, major, minor)

        elif message_id == self.NAT_RESPONSE :
            trace( "Message ID  : %3.1d NAT_RESPONSE"% message_id )
            trace( "Packet Size : ", packet_size )
            if packet_size == 4 :
                command_response = int.from_bytes( data[offset:offset+4], byteorder='little' )
                trace( "Command response: %d - %d %d %d %d"% (command_response,
                                                             data[offset],
                                                             data[offset+1],
                                                             data[offset+2],
                                                             data[offset+3]))
                offset += 4
            else:
                show_remainder = False
                message, separator, remainder = bytes(data[offset:]).partition( b'\0' )
                if(len(message) < 30):
                    tmpString = message.decode('utf-8')
                    # Decode bitstream version
                    if( tmpString.startswith('Bitstream') ):
                        nn_version = self.__unpack_bitstream_info(data[offset:],packet_size, major, minor)
                        # This is the current server version
                        if(len(nn_version)>1):
                            for i in range( len(nn_version) ):
                                self.__nat_net_stream_version_server[i] = int(nn_version[i])
                            for i in range( len(nn_version),4 ):
                                self.__nat_net_stream_version_server[i] = 0
                            
                offset += len( message ) + 1

                if(show_remainder):
                    trace( "Command response:", message.decode( 'utf-8' ),\
                        " separator:", separator, " remainder:",remainder )
                else:
                    trace( "Command response:", message.decode( 'utf-8' ))
        elif message_id == self.NAT_UNRECOGNIZED_REQUEST :
            trace( "Message ID  : %3.1d NAT_UNRECOGNIZED_REQUEST: "% message_id )
            trace( "Packet Size : ", packet_size )
            trace( "Received 'Unrecognized request' from server" )
        elif message_id == self.NAT_MESSAGESTRING :
            trace( "Message ID  : %3.1d NAT_MESSAGESTRING"% message_id)
            trace( "Packet Size : ", packet_size )
            message, separator, remainder = bytes(data[offset:]).partition( b'\0' )
            offset += len( message ) + 1
            trace( "Received message from server:", message.decode( 'utf-8' ) )
        else:
            trace( "Message ID  : %3.1d UNKNOWN"% message_id )
            trace( "Packet Size : ", packet_size )
            trace( "ERROR: Unrecognized packet type" )

        trace( "End Packet\n-----------------" )
        return message_id

    def send_request( self, in_socket, command, command_str, address ):
        # Compose the message in our known message format
        packet_size = 0
        if command == self.NAT_REQUEST_MODELDEF or command == self.NAT_REQUEST_FRAMEOFDATA :
            packet_size = 0
            command_str = ""
        elif command == self.NAT_REQUEST :
            packet_size = len( command_str ) + 1
        elif command == self.NAT_CONNECT :
            tmp_version=[4,1,0,0]
            print("NAT_CONNECT to Motive with %d %d %d %d"%(
                tmp_version[0],
                tmp_version[1],
                tmp_version[2],
                tmp_version[3]
            ))
            #allocate a byte array for 270 bytes
            # to connect with a specific version
            # The first 4 bytes spell out "Ping"
            command_str = []
            command_str = [0 for i in range(270)]
            command_str[0] =80
            command_str[1] =105
            command_str[2] =110
            command_str[3] =103
            command_str[264] =0
            command_str[265] =tmp_version[0]
            command_str[266] =tmp_version[1]
            command_str[267] =tmp_version[2]
            command_str[268] =tmp_version[3]
            packet_size = len( command_str ) + 1
        elif command == self.NAT_KEEPALIVE:
            packet_size = 0
            command_str = ""

        data = command.to_bytes( 2, byteorder='little' )
        data += packet_size.to_bytes( 2, byteorder='little' )

        if command == self.NAT_CONNECT :
            data+=bytearray(command_str)
        else:
            data += command_str.encode( 'utf-8' )
        data += b'\0'

        return in_socket.sendto( data, address )

    def send_command( self, command_str):
        #print("Send command %s"%command_str)
        nTries = 3
        ret_val = -1
        while nTries:
            nTries -= 1
            ret_val = self.send_request( self.command_socket, self.NAT_REQUEST, command_str,  (self.server_ip_address, self.command_port) )
            if (ret_val != -1):
                break;
        return ret_val

        #return self.send_request(self.data_socket,    self.NAT_REQUEST, command_str,  (self.server_ip_address, self.command_port) )

    def send_commands(self,tmpCommands, print_results: bool =True):
        for sz_command in tmpCommands:
            return_code = self.send_command(sz_command)
            if(print_results):
                print("Command: %s - return_code: %d"% (sz_command, return_code) )

    def send_keep_alive(self,in_socket, server_ip_address, server_port):
        return self.send_request(in_socket, self.NAT_KEEPALIVE, "", (server_ip_address, server_port))

    def get_command_port(self):
        return self.command_port

    def refresh_configuration(self):
        #query for application configuration
        #print("Request current configuration")
        sz_command = "Bitstream"
        return_code = self.send_command(sz_command)
        time.sleep(0.5)

    def get_application_name(self):
        return self.__application_name

    def get_nat_net_requested_version(self):
        return self.__nat_net_requested_version

    def get_nat_net_version_server(self):
        return self.__nat_net_stream_version_server

    def get_server_version(self):
        return self.__server_version



    def run( self ):
        # Create the data socket
        self.data_socket = self.__create_data_socket( self.data_port )
        if self.data_socket is None :
            print( "Could not open data channel" )
            return False

        # Create the command socket
        self.command_socket = self.__create_command_socket()
        if self.command_socket is None :
            print( "Could not open command channel" )
            return False
        self.__is_locked = True

        self.stop_threads = False
        # Create a separate thread for receiving data packets
        self.data_thread = Thread( target = self.__data_thread_function, args = (self.data_socket, lambda : self.stop_threads, lambda : self.print_level, ))
        self.data_thread.start()

        # Create a separate thread for receiving command packets
        self.command_thread = Thread( target = self.__command_thread_function, args = (self.command_socket, lambda : self.stop_threads, lambda : self.print_level,))
        self.command_thread.start()

        # Required for setup
        # Get NatNet and server versions
        self.send_request(self.command_socket, self.NAT_CONNECT, "",  (self.server_ip_address, self.command_port) )


        ##Example Commands
        ## Get NatNet and server versions
        #self.send_request(self.command_socket, self.NAT_CONNECT, "", (self.server_ip_address, self.command_port) )
        ## Request the model definitions
        #self.send_request(self.command_socket, self.NAT_REQUEST_MODELDEF, "",  (self.server_ip_address, self.command_port) )
        return True

    def shutdown(self):
        print("shutdown called")
        self.stop_threads = True
        # closing sockets causes blocking recvfrom to throw
        # an exception and break the loop
        self.command_socket.close()
        self.data_socket.close()
        # attempt to join the threads back.
        self.command_thread.join()
        self.data_thread.join()

