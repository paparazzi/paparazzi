#Copyright © 2021 Naturalpoint
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


# OptiTrack NatNet direct depacketization sample for Python 3.x
#


# Uses the Python NatNetClient.py library to establish a connection (by creating a NatNetClient),
# and receive data via a NatNet connection and decode it using the NatNetClient library.



import copy
import hashlib
import random

K_SKIP = [0,0,1]
K_FAIL = [0,1,0]
K_PASS = [1,0,0]


def get_tab_str(tab_str, level):
    out_tab_str=""
    loop_range = range(0,level)
    for _ in loop_range:
        out_tab_str+=tab_str
    return out_tab_str

def add_lists(totals, totals_tmp):
    totals[0]+=totals_tmp[0]
    totals[1]+=totals_tmp[1]
    totals[2]+=totals_tmp[2]
    return totals



def test_hash(test_name, test_hash_str, test_object):
    out_str = test_object.get_as_string()
    out_hash_str=hashlib.sha1(out_str.encode()).hexdigest()
    ret_value=True
    if test_hash_str == out_hash_str :
        print("[PASS]:%s"%test_name)
    else:
        print("[FAIL]:%s test_hash_str != out_hash_str"%test_name)
        print("test_hash_str=%s"%test_hash_str)
        print("out_hash_str=%s"%out_hash_str)
        print("out_str =\n%s"%out_str)
        ret_value=False
    return ret_value

def test_hash2(test_name, test_hash_str, test_object, run_test=True):
    ret_value = K_FAIL
    out_str = "FAIL"
    out_str2=""
    indent_string="       "
    if not run_test:
        ret_value = K_SKIP
        out_str = "SKIP"
    elif test_object == None:
        out_str = "FAIL"
        ret_value = K_FAIL
        out_str2 = "%sERROR: test_object was None"%indent_string
    else:
        obj_out_hash_str = ""
        if str(type(test_object)) != 'NoneType':
            obj_out_str = test_object.get_as_string()
            obj_out_hash_str=hashlib.sha1(obj_out_str.encode()).hexdigest()

        if test_hash_str == obj_out_hash_str:
            out_str = "PASS"
            ret_value = K_PASS
        else:
            out_str2+="%s%s test_hash_str != out_hash_str\n"%(indent_string,test_name)
            out_str2+="%stest_hash_str=%s\n"%(indent_string,test_hash_str)
            out_str2+="%sobj_out_hash_str=%s\n"%(indent_string,obj_out_hash_str)
            out_str2+="%sobj_out_str =\n%s"%(indent_string,obj_out_str)
            ret_value = K_FAIL
    print("[%s]:%s"%(out_str,test_name))

    if len(out_str2):
        print("%s"%out_str2)
    return ret_value



def get_as_string(input_str):
    if type(input_str) == str:
        return input_str
    else:
        return input_str.decode('utf-8')


def get_data_sub_packet_type(new_data):
    out_string=""
    data_type = type(new_data)
    if data_type == MarkerSetDescription:
        out_string="Type: 0 Markerset\n"
    elif data_type == RigidBodyDescription:
        out_string="Type: 1 Rigid Body\n"
    elif data_type == SkeletonDescription:
        out_string="Type: 2 Skeleton\n"
    elif data_type == ForcePlateDescription:
        out_string="Type: 3 Force Plate\n"
    elif data_type == DeviceDescription:
        out_string="Type: 4 Device\n"
    elif data_type == CameraDescription:
        out_string="Type: 5 Camera\n"
    elif data_type == AssetDescription:
        out_string="Type: 6 Asset\n"
    elif data_type == None:
        out_string="Type: None\n"
    else:
        out_string="Type: Unknown %s\n"%str(data_type)
    return out_string

# cMarkerSetDescription
class MarkerSetDescription:
    def __init__(self):
        self.marker_set_name="Not Set"
        self.marker_names_list=[]

    def set_name(self,new_name):
        self.marker_set_name=new_name

    def get_num_markers(self):
        return len(self.marker_names_list)

    def add_marker_name(self,marker_name):
        self.marker_names_list.append(copy.copy(marker_name))
        return self.get_num_markers()

    def get_as_string(self, tab_str="  ", level=0):
        out_tab_str = get_tab_str(tab_str, level)
        out_tab_str2 = get_tab_str(tab_str, level+1)
        out_tab_str3 = get_tab_str(tab_str, level+2)
        out_string=""
        out_string+="%sMarker Set Name: %s\n"%(out_tab_str,get_as_string(self.marker_set_name))
        num_markers = len(self.marker_names_list)
        out_string+="%sMarker Count   : %d\n"%(out_tab_str2, num_markers)
        for i in range(num_markers):
            out_string+="%s%3.1d Marker Name: %s\n"%(out_tab_str3,i, get_as_string(self.marker_names_list[i]))
        return out_string

class RBMarker:
    def __init__(self, marker_name="", active_label=0, pos=[0.0,0.0,0.0]):
        self.marker_name = marker_name
        self.active_label = active_label
        self.pos=pos

    def get_as_string(self, tab_str="  ", level=0):
        out_tab_str = get_tab_str(tab_str, level)
        out_string=""
        out_string += "%sMarker Label: %s Position: [%f %f %f] %s\n" % \
            (out_tab_str, self.active_label, self.pos[0],self.pos[1],self.pos[2],self.marker_name )
        return out_string


class RigidBodyDescription:
    def __init__(self,sz_name="", new_id=0, parent_id=0,pos=[0.0,0.0,0.0]):
        self.sz_name=sz_name
        self.id_num = new_id
        self.parent_id = parent_id
        self.pos=pos
        self.rb_marker_list=[]


    def set_name(self,new_name):
        self.sz_name=new_name

    def set_id(self, new_id):
        self.id_num = new_id

    def set_parent_id(self, parent_id):
        self.parent_id = parent_id

    def set_pos(self,p_x,p_y,p_z):
        self.pos=[p_x,p_y,p_z]

    def get_num_markers(self):
        return len(self.rb_marker_list)

    def add_rb_marker(self,new_rb_maker):
        self.rb_marker_list.append(copy.deepcopy(new_rb_maker))
        return self.get_num_markers()


    def get_as_string(self, tab_str="  ", level=0):
        out_tab_str = get_tab_str(tab_str, level)
        out_tab_str2 = get_tab_str(tab_str, level+1)
        out_string=""
        out_string += "%sRigid Body Name   : %s\n"%(out_tab_str, get_as_string(self.sz_name))
        out_string += "%sID                : %d\n"%(out_tab_str, self.id_num)
        out_string += "%sParent ID         : %d\n"%(out_tab_str, self.parent_id)
        out_string += "%sPosition          : [%3.2f, %3.2f, %3.2f]\n"%(out_tab_str, self.pos[0],self.pos[1],self.pos[2])
        num_markers= len(self.rb_marker_list)
        out_string += "%sNumber of Markers : %d\n"%(out_tab_str, num_markers )
        # loop over markers
        for i in range(num_markers):
            out_string += "%s%i %s"%(out_tab_str2,i,self.rb_marker_list[i].get_as_string(tab_str,0))
        return out_string



class SkeletonDescription:
    def __init__(self, name="", new_id=0):
        self.name = name
        self.id_num = new_id
        self.rigid_body_description_list=[]

    def set_name(self,new_name):
        self.name=new_name

    def set_id(self, new_id):
        self.id_num = new_id

    def add_rigid_body_description(self,rigid_body_description):
        self.rigid_body_description_list.append(copy.deepcopy(rigid_body_description))
        return len(self.rigid_body_description_list)

    def get_as_string(self, tab_str="  ", level=0):
        out_tab_str = get_tab_str(tab_str, level)
        out_tab_str2 = get_tab_str(tab_str, level+1)
        out_string = ""
        out_string += "%sName                    : %s\n"%(out_tab_str,get_as_string(self.name))
        out_string += "%sID                      : %d\n"% (out_tab_str, self.id_num)
        num_bones = len(self.rigid_body_description_list)
        out_string += "%sRigid Body (Bone) Count : %d\n" % (out_tab_str, num_bones)
        for i in range(num_bones):
            out_string += "%sRigid Body (Bone) %d\n"%(out_tab_str2, i)
            out_string += self.rigid_body_description_list[i].get_as_string(tab_str,level+2)
        return out_string


class ForcePlateDescription:
    def __init__(self, new_id=0, serial_number=""):
        self.id_num = new_id
        self.serial_number = serial_number
        self.width = 0
        self.length = 0
        self.position=[0.0,0.0,0.0]
        self.cal_matrix= [[0.0 for col in range(12)] for row in range(12)]
        self.corners = [[0.0 for col in range(3)] for row in range(4)]
        self.plate_type = 0
        self.channel_data_type = 0
        self.channel_list=[]

    def set_id(self, new_id):
        self.id_num = new_id

    def set_serial_number(self,serial_number):
        self.serial_number=serial_number

    def set_dimensions(self, width, length):
        self.width = width
        self.length = length

    def set_origin(self, p_x, p_y, p_z):
        self.position=[p_x,p_y,p_z]

    def set_cal_matrix(self, cal_matrix):
        self.cal_matrix = cal_matrix

    def set_corners(self, corners):
        self.corners = corners

    def set_plate_type(self, plate_type):
        self.plate_type=plate_type

    def set_channel_data_type(self, channel_data_type):
        self.channel_data_type = channel_data_type

    def add_channel_name(self,channel_name):
        self.channel_list.append(copy.deepcopy(channel_name))
        return len(self.channel_list)

    def get_cal_matrix_as_string(self, tab_str="", level=0):
        """Get force plate calibration matrix as string"""
        out_tab_str=get_tab_str(tab_str,level)
        out_tab_str2=get_tab_str(tab_str,level+1)
        out_string=""
        out_string+="%sCal Matrix:\n"%out_tab_str
        for i in range(0,12):
            out_string+="%s%2.1d %3.3e %3.3e %3.3e %3.3e %3.3e %3.3e %3.3e %3.3e %3.3e %3.3e %3.3e %3.3e\n" % \
                    (out_tab_str2,i,
                    self.cal_matrix[i][0], self.cal_matrix[i][1],
                    self.cal_matrix[i][2], self.cal_matrix[i][3],
                    self.cal_matrix[i][4], self.cal_matrix[i][5],
                    self.cal_matrix[i][6], self.cal_matrix[i][7],
                    self.cal_matrix[i][8], self.cal_matrix[i][9],
                    self.cal_matrix[i][10], self.cal_matrix[i][11])
        return out_string

    def get_corners_as_string(self, tab_str="", level=0):
        """Get force plate corner positions as a string"""
        # Corners 4x3 floats
        out_tab_str=get_tab_str(tab_str,level)
        out_tab_str2=get_tab_str(tab_str,level+1)
        out_string=""
        out_string+="%sCorners:\n"%out_tab_str
        for i in range(0,4):
            out_string+="%s%2.1d %3.3e %3.3e %3.3e\n" % \
                    (out_tab_str2,i,
                    self.corners[i][0], self.corners[i][1], self.corners[i][2])
        return out_string


    def get_as_string(self, tab_str="  ", level=0):
        """Get force plate description as a class"""
        out_tab_str = get_tab_str(tab_str, level)
        out_string = ""
        out_string += "%sID                      : %d\n"% (out_tab_str, self.id_num)
        out_string += "%sSerial Number           : %s\n"% (out_tab_str,\
            get_as_string(self.serial_number))
        out_string += "%sWidth                   : %3.2f\n"%(out_tab_str, self.width)
        out_string += "%sLength                  : %3.2f\n"%(out_tab_str, self.length)
        out_string += "%sOrigin                  : %3.2f, %3.2f, %3.2f\n"%(out_tab_str,
                                                                       self.position[0],
                                                                       self.position[1],
                                                                       self.position[2])
        out_string += self.get_cal_matrix_as_string(tab_str, level)
        out_string += self.get_corners_as_string(tab_str, level)

        out_string+="%sPlate Type                : %d\n"%(out_tab_str, self.plate_type)
        out_string+="%sChannel Data Type         : %d\n"%(out_tab_str, self.channel_data_type)
        num_channels = len(self.channel_list)
        out_string+="%sNumber of Channels        : %d\n"%(out_tab_str, num_channels)
        # Channel Names list of NoC strings
        out_tab_str2=get_tab_str(tab_str, level+1)
        for channel_num in range(num_channels):
            out_string += "%sChannel Name %d: %s\n"%(out_tab_str2, channel_num,\
                                                    get_as_string(self.channel_list[channel_num]) )

        return out_string

class DeviceDescription:
    """Device Description class"""
    def __init__(self,new_id,name, serial_number,device_type,channel_data_type):
        self.id_num=new_id
        self.name=name
        self.serial_number=serial_number
        self.device_type=device_type
        self.channel_data_type=channel_data_type
        self.channel_list=[]

    def set_id(self, new_id):
        """Set the device id"""
        self.id_num=new_id

    def set_name(self, name):
        """Set the Device name"""
        self.name=name

    def add_channel_name(self, channel_name):
        """Add channel name to channel_list"""
        self.channel_list.append(channel_name)
        return len(self.channel_list)

    def get_as_string(self, tab_str="  ", level=0):
        """Get Device Description as string"""
        out_tab_str = get_tab_str(tab_str, level)
        out_tab_str2 = get_tab_str(tab_str, level+1)
        out_string = ""
        out_string +="%sID                 : %5.1d\n"%(out_tab_str, self.id_num)
        out_string +="%sName               : %s\n"%(out_tab_str,get_as_string(self.name))
        out_string +="%sSerial Number      : %s\n"%(out_tab_str,get_as_string(self.serial_number))
        out_string +="%sDevice Type        : %d\n"%(out_tab_str,self.device_type)
        out_string +="%sChannel Data Type  : %d\n"%(out_tab_str, self.channel_data_type)
        num_channels = len(self.channel_list)
        out_string +="%sNumber of Channels : %d\n"%(out_tab_str, num_channels)
        for i in range(num_channels):
            out_string+="%sChannel %2.1d Name : %s\n"%(out_tab_str2, i,\
            get_as_string(self.channel_list[i]))
        return out_string

class CameraDescription:
    """Camera Description class"""
    def __init__(self, name, position_vec3, orientation_quat):
        self.name=name
        self.position=position_vec3
        self.orientation=orientation_quat

    def get_as_string(self, tab_str="..", level=0):
        """Get Camera Description as a string"""
        out_tab_str = get_tab_str(tab_str, level)
        out_string = ""
        out_string += "%sName        : %s\n"%(out_tab_str,get_as_string(self.name))
        out_string += "%sPosition    : [%3.2f, %3.2f, %3.2f]\n"% \
            (out_tab_str,self.position[0], self.position[1], self.position[2] )
        out_string += "%sOrientation : [%3.2f, %3.2f, %3.2f, %3.2f]\n"% \
            (out_tab_str,\
            self.orientation[0], self.orientation[1],\
            self.orientation[2], self.orientation[3] )
        return out_string

class MarkerDescription:
    """Marker Description class"""
    def __init__(self, name, marker_id, position, marker_size, marker_params):
        self.name=name
        self.marker_id=marker_id
        self.position=position
        self.marker_size=marker_size
        self.marker_params=marker_params

    def get_as_string(self, tab_str="..", level=0):
        """Get Marker Description as a string"""
        out_tab_str = get_tab_str(tab_str, level)
        out_string = ""
        out_string += "%sName        : %s\n"%(out_tab_str,get_as_string(self.name))
        out_string += "%sPosition    : [%3.2f, %3.2f, %3.2f]\n"% \
            (out_tab_str,self.position[0], self.position[1], self.position[2] )
        out_string += "Size          : %d\n"%(self.marker_size)
        out_string += "Params        : %d\n"%(self.marker_params)

        return out_string

class AssetDescription:
    """Asset Description class"""
    def __init__(self, name, assetType, assetID, rigidbodyArray, markerArray):
        self.name=name
        self.assetType=assetType
        self.assetID=assetID
        self.rigidbodyArray=rigidbodyArray
        self.markerArray=markerArray

    def get_as_string(self, tab_str="..", level=0):
        """Get Asset Description as a string"""
        out_tab_str = get_tab_str(tab_str, level)
        out_string = ""
        out_string += "Asset Description\n"
        out_string += "%sName        : %s\n"%(out_tab_str,get_as_string(self.name))
        out_string += "assetType     : %d\n"%(self.assetType)
        out_string += "assetID       : %d\n"%(self.assetID)
        out_string += "numRBs        : %d\n"%(self.rigidbodyArray.size())
        out_string += "numMarkers    : %d\n"%(self.markerArray.size())

        return out_string



# cDataDescriptions
# Full data descriptions
class DataDescriptions():
    """Data Descriptions class"""
    order_num = 0
    def __init__(self):
        self.data_order_dict={}
        self.marker_set_list=[]
        self.rigid_body_list=[]
        self.skeleton_list=[]
        self.force_plate_list=[]
        self.device_list=[]
        self.camera_list=[]

    def generate_order_name(self):
        """Generate the name for the order list based on the current length of the list"""
        # should be a one up counter instead of based on length of data_order_dict
        order_name="data_%3.3d"%self.order_num
        self.order_num += 1
        return order_name

    # Add Marker Set
    def add_marker_set(self, new_marker_set):
        """Add a marker set"""
        order_name = self.generate_order_name()

        # generate order entry
        pos = len(self.marker_set_list)
        self.data_order_dict[order_name]=("marker_set_list", pos)
        self.marker_set_list.append(copy.deepcopy(new_marker_set))

    # Add Rigid Body
    def add_rigid_body(self, new_rigid_body):
        """Add a rigid body"""
        order_name = self.generate_order_name()

        # generate order entry
        pos = len(self.rigid_body_list)
        self.data_order_dict[order_name]=("rigid_body_list", pos)
        self.rigid_body_list.append(copy.deepcopy(new_rigid_body))


    # Add a skeleton
    def add_skeleton(self, new_skeleton):
        """Add a skeleton"""
        order_name = self.generate_order_name()

        # generate order entry
        pos = len(self.skeleton_list)
        self.data_order_dict[order_name]=("skeleton_list", pos)
        self.skeleton_list.append(copy.deepcopy(new_skeleton))


    # Add a force plate
    def add_force_plate(self, new_force_plate):
        """Add a force plate"""
        order_name = self.generate_order_name()

        # generate order entry
        pos = len(self.force_plate_list)
        self.data_order_dict[order_name]=("force_plate_list", pos)
        self.force_plate_list.append(copy.deepcopy(new_force_plate))


    def add_device(self, newdevice):
        """ add_device - Add a device"""
        order_name = self.generate_order_name()

        # generate order entry
        pos = len(self.device_list)
        self.data_order_dict[order_name]=("device_list", pos)
        self.device_list.append(copy.deepcopy(newdevice))


    def add_camera(self, newcamera):
        """ Add a new camera """
        order_name = self.generate_order_name()

        # generate order entry
        pos = len(self.camera_list)
        self.data_order_dict[order_name]=("camera_list", pos)
        self.camera_list.append(copy.deepcopy(newcamera))

    def add_data(self, new_data):
        """Add data based on data type"""
        data_type = type(new_data)
        if data_type == MarkerSetDescription:
            self.add_marker_set(new_data)
        elif data_type == RigidBodyDescription:
            self.add_rigid_body(new_data)
        elif data_type == SkeletonDescription:
            self.add_skeleton(new_data)
        elif data_type == ForcePlateDescription:
            self.add_force_plate(new_data)
        elif data_type == DeviceDescription:
            self.add_device(new_data)
        elif data_type == CameraDescription:
            self.add_camera(new_data)
        elif data_type is None:
            data_type = None
        else:
            print("ERROR: Type %s unknown"%str(data_type))

    def get_object_from_list(self, list_name, pos_num):
        """Determine list name and position of the object"""
        ret_value = None
        if (list_name =="marker_set_list") and \
           (pos_num < len(self.marker_set_list)):
            ret_value = self.marker_set_list[pos_num]

        elif (list_name =="rigid_body_list") and \
            (pos_num < len(self.rigid_body_list)):
            ret_value = self.rigid_body_list[pos_num]

        elif (list_name =="skeleton_list") and \
            (pos_num < len(self.skeleton_list)):
            ret_value = self.skeleton_list[pos_num]

        elif (list_name =="force_plate_list") and \
            (pos_num < len(self.force_plate_list)):
            ret_value = self.force_plate_list[pos_num]

        elif (list_name =="device_list") and \
            (pos_num < len(self.device_list)):
            ret_value = self.device_list[pos_num]

        elif (list_name =="camera_list") and \
            (pos_num < len(self.camera_list)):
            ret_value = self.camera_list[pos_num]

        else:
            ret_value = None

        return ret_value

    def get_as_string(self, tab_str="  ", level = 0):
        """Ensure data comes back as a string"""
        out_tab_str = get_tab_str(tab_str,level)
        out_tab_str2 = get_tab_str(tab_str,level+1)
        out_tab_str3 = get_tab_str(tab_str,level+2)
        out_string=""
        num_data_sets=len(self.data_order_dict)
        out_string+="%sNumber of Data Sets: %d\n"%(out_tab_str, num_data_sets)
        i=0
        for tmp_key,tmp_value in self.data_order_dict.items():
            #tmp_name,tmp_num=self.data_order_dict[data_set]
            tmp_name=tmp_value[0]
            tmp_num =tmp_value[1]
            tmp_object = self.get_object_from_list(tmp_name, tmp_num)
            out_string += "%sDataset %3.1d\n"%(out_tab_str2, i)
            tmp_string = get_data_sub_packet_type(tmp_object)
            if tmp_string != "":
                out_string += "%s%s"%(out_tab_str2, tmp_string)
            #out_string += "%s%s %s %d\n"%(out_tab_str2, data_set, tmp_name,tmp_num)
            out_string += "%s%s %s %s\n"%(out_tab_str2,tmp_key, tmp_name,tmp_num)
            if tmp_object is not None:
                out_string += tmp_object.get_as_string(tab_str,level+2)
            else:
                out_string += "%s%s %s %s not found\n"%(out_tab_str3,tmp_key, tmp_name,tmp_num)
            out_string += "\n"
            i+=1

        return out_string

# cDataDescriptions END

def generate_marker_set_description(set_num=0):
    """generate_marker_set_description - Testing functions"""
    marker_set_description = MarkerSetDescription()
    marker_set_description.set_name("MarkerSetName%3.3d"%set_num)
    marker_set_description.add_marker_name("MarkerName%3.3d_0"% set_num)
    marker_set_description.add_marker_name("MarkerName%3.3d_1"% set_num)
    marker_set_description.add_marker_name("MarkerName%3.3d_2"% set_num)
    marker_set_description.add_marker_name("MarkerName%3.3d_3"% set_num)
    return marker_set_description

def generate_rb_marker(marker_num=0):
    """generate_rb_marker - Generate rigid body marker based on marker number"""
    marker_num_mod = marker_num % 4
    marker_name="RBMarker_%3.3d"%marker_num
    marker_active_label = marker_num+10000
    marker_pos=[1.0,4.0,9.0]
    if marker_num_mod == 1:
        marker_pos=[1.0, 8.0, 27.0]
    elif marker_num_mod == 2:
        marker_pos=[3.1, 4.1, 5.9]
    elif marker_num_mod == 3:
        marker_pos=[1.0, 3.0, 6.0]

    return RBMarker(marker_name, marker_active_label, marker_pos)

def generate_rigid_body_description(rbd_num=0):
    """generate_rigid_body_description - Generate Rigid Body Description Data"""
    rbd=RigidBodyDescription()
    rbd.set_name("rigidBodyDescription_%3.3d"%rbd_num)
    rbd.set_id(3141)
    rbd.set_parent_id(314)
    rbd.set_pos(1,4,9)
    rbd.add_rb_marker(generate_rb_marker(0))
    rbd.add_rb_marker(generate_rb_marker(1))
    rbd.add_rb_marker(generate_rb_marker(2))

    return rbd

def generate_skeleton_description(skeleton_num=0):
    """generate_skeleton_description -Generate Test SkeletonDescription Data"""
    skel_desc=SkeletonDescription("SkeletonDescription_%3.3d"%skeleton_num,skeleton_num)
    #generate some rigid bodies to add
    skel_desc.add_rigid_body_description(generate_rigid_body_description(0))
    skel_desc.add_rigid_body_description(generate_rigid_body_description(1))
    skel_desc.add_rigid_body_description(generate_rigid_body_description(2))
    skel_desc.add_rigid_body_description(generate_rigid_body_description(3))
    skel_desc.add_rigid_body_description(generate_rigid_body_description(5))
    skel_desc.add_rigid_body_description(generate_rigid_body_description(7))
    return skel_desc

def generate_force_plate_description(force_plate_num=0):
    """generate_force_plate_description - Generate Test ForcePlateDescription Data"""
    fp_id=force_plate_num
    random.seed(force_plate_num)

    serial_number="S/N_%5.5d"%random.randint(0,99999)
    width=random.random()*10
    length=random.random()*10
    origin=[(random.random()*100),(random.random()*100),(random.random()*100)]
    corners = [[0.0, 0.0, 0.0],
               [0.0, 1.0, 0.0],
               [1.0, 1.0, 0.0],
               [1.0, 0.0, 0.0]]

    fp_desc=ForcePlateDescription(fp_id, serial_number)
    fp_desc.set_dimensions(width, length)
    fp_desc.set_origin(origin[0],origin[1],origin[2])
    #fp_desc.set_cal_matrix(cal_matrix)
    fp_desc.set_corners(corners)
    for i in range(3):
        fp_desc.add_channel_name("channel_%3.3d"%i)
    return fp_desc


def generate_device_description(dev_num=0):
    """generate_device_description- Generate Test DeviceDescription Data"""
    new_id=0
    name="Device%3.3d"%dev_num
    serial_number="SerialNumber%3.3d"%dev_num
    device_type=dev_num%4
    channel_data_type=dev_num%5
    dev_desc = DeviceDescription(new_id,name, serial_number,device_type,channel_data_type)
    for i in range(channel_data_type+3):
        dev_desc.add_channel_name("channel_name_%2.2d"%i)
    return dev_desc


def generate_camera_description(cam_num=0):
    """generate_camera_description - Generate Test CameraDescription data"""
    pos_vec3=[1,2,3]
    orientation_quat=[1,2,3,4]
    return CameraDescription("Camera_%3.3d"%cam_num, pos_vec3, orientation_quat)


#generate_data_descriptions - Generate Test DataDescriptions
def generate_data_descriptions(data_desc_num=0):
    """Generate data descriptions"""
    data_descs = DataDescriptions()

    data_descs.add_data(generate_marker_set_description(data_desc_num+0))
    data_descs.add_data(generate_marker_set_description(data_desc_num+1))

    data_descs.add_data(generate_rigid_body_description(data_desc_num+0))
    data_descs.add_data(generate_rigid_body_description(data_desc_num+1))

    data_descs.add_skeleton(generate_skeleton_description(data_desc_num+3))
    data_descs.add_skeleton(generate_skeleton_description(data_desc_num+9))
    data_descs.add_skeleton(generate_skeleton_description(data_desc_num+27))

    data_descs.add_force_plate(generate_force_plate_description(data_desc_num+123))
    data_descs.add_force_plate(generate_force_plate_description(data_desc_num+87))
    data_descs.add_force_plate(generate_force_plate_description(data_desc_num+21))

    data_descs.add_device(generate_device_description(data_desc_num+0))
    data_descs.add_device(generate_device_description(data_desc_num+2))
    data_descs.add_device(generate_device_description(data_desc_num+4))

    data_descs.add_camera(generate_camera_description(data_desc_num+0))
    data_descs.add_camera(generate_camera_description(data_desc_num+10))
    data_descs.add_camera(generate_camera_description(data_desc_num+3))
    data_descs.add_camera(generate_camera_description(data_desc_num+7))
    return data_descs


# test_all - Test all the major classes
def test_all(run_test=True):
    """Test all the Data Description classes"""
    totals=[0,0,0]
    if run_test is True:
        test_cases=[["Test Marker Set Description 0",  "754fe535286ca84bd054d9aca5e9906ab9384d92",
                        "generate_marker_set_description(0)",True],
                    ["Test RB Marker 0",               "0f2612abf2ce70e479d7b9912f646f12910b3310",
                        "generate_rb_marker(0)",True],
                    ["Test Rigid Body Description 0",  "7a4e93dcda442c1d9c5dcc5c01a247e4a6c01b66",
                        "generate_rigid_body_description(0)",True],
                    ["Test Skeleton Description 0",    "b4d1a031dd7c323e3d316b5312329881a6a552ca",
                        "generate_skeleton_description(0)",True],
                    ["Test Force Plate Description 0", "b385dd1096bdd9f521eb48bb9cbfb3414ea075bd",
                        "generate_force_plate_description(0)",True],
                    ["Test Device Description 0",      "39b4fdda402bc73c0b1cd5c7f61599476aa9a926",
                        "generate_device_description(0)",True],
                    ["Test Camera Description 0",      "614602c5d290bda3b288138d5e25516dd1e1e85a",
                        "generate_camera_description(0)",True],
                    ["Test Data Description 0",        "e5f448d10087ac818a65934710a85fc7ebfdf89e",
                        "generate_data_descriptions(0)",True],
                    ]
        num_tests = len(test_cases)
        for i in range(num_tests):
            data = eval(test_cases[i][2])
            totals_tmp = test_hash2(test_cases[i][0],test_cases[i][1],data,test_cases[i][3])
            totals=add_lists(totals, totals_tmp)

    print("--------------------")
    print("[PASS] Count = %3.1d"%totals[0])
    print("[FAIL] Count = %3.1d"%totals[1])
    print("[SKIP] Count = %3.1d"%totals[2])

    return totals



#
if __name__ == "__main__":
    test_all(True)
