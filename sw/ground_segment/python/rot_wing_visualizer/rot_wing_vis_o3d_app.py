#
# Copyright (C) 2023 TUDelft
#
# This file is part of paparazzi.
#
# paparazzi is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi.  If not, see <http://www.gnu.org/licenses/>.
#

import sys

import os
import open3d as o3d
import numpy as np
import time
import platform
import random
import threading
import copy

isMacOS = (platform.system() == "Darwin")

PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))

PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))

sys.path.append(PPRZ_HOME + "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface


class AHRSRefQuatMessage(object):
    def __init__(self, msg):
        self.body_qi = msg['body_qi']
        self.body_qx = msg['body_qx']
        self.body_qy = msg['body_qy']
        self.body_qz = msg['body_qz']

        self.ref_qi = msg['ref_qi']
        self.ref_qx = msg['ref_qx']
        self.ref_qy = msg['ref_qy']
        self.ref_qz = msg['ref_qz']
    
    def get_body_quaternion(self):
        body_qi = float(self.body_qi) / 2**15
        body_qx = float(self.body_qx) / 2**15
        body_qy = float(self.body_qy) / 2**15
        body_qz = float(self.body_qz) / 2**15
        return np.array([[body_qi], [body_qx], [body_qy], [body_qz]])
    
    def get_ref_quaternion(self):
        ref_qi = float(self.ref_qi) / 2**15
        ref_qx = float(self.ref_qx) / 2**15
        ref_qy = float(self.ref_qy) / 2**15
        ref_qz = float(self.ref_qz) / 2**15
        return np.array([[ref_qi], [ref_qx], [ref_qy], [ref_qz]])
    
class RotWingControllerMessage(object):
    def __init__(self, msg):
        self.wing_angle_deg = msg['wing_angle_deg']
        self.wing_angle_deg_sp = msg['wing_angle_deg_sp']

    def get_wing_angle(self):
        return np.deg2rad(float(self.wing_angle_deg))
    
    def get_wing_angle_sp(self):
        return np.deg2rad(float(self.wing_angle_deg_sp))
    
class ActuatorsMessage(object):
    def __init__(self, msg):
        self.values = msg['values']
    
    def get_actuator_values(self):
        return np.array(self.values).astype(dtype = float)
    
    def get_actuator_value(self, idx):
        return float(self.values[idx])
    
class EscMessage(object):
    def __init__(self, msg):
        self.id = int(msg['motor_id'])
        self.amp = float(msg['amps'])
        self.rpm = float(msg['rpm'])
        self.volt_b = float(msg['bat_volts'])
        self.volt_m = float(msg['motor_volts'])
        self.temperature = float(msg['temperature']) - 273.15
        self.energy = float(msg['energy'])

    def get_id(self):
        return self.id
    
    def get_temp(self):
        return self.temperature

class Viewer3D(object):
    MENU_SPHERE = 1
    MENU_RANDOM = 2
    MENU_QUIT = 3
    
    def __init__(self, title):
        self._id = 0

        app = o3d.visualization.gui.Application.instance
        app.initialize()

        self.w = app.create_window(title)

        self.main_vis = o3d.visualization.gui.SceneWidget()
        self.main_vis.scene = o3d.visualization.rendering.Open3DScene(self.w.renderer)
        self.main_vis.scene.set_background(np.array([[0],[0],[0],[0]], dtype=np.float32))
        self.main_vis.scene.scene.set_sun_light(
            [1, 1, 1],  # direction
            [1, 1, 1],  # color
            100000)  # intensity
        self.main_vis.scene.scene.enable_sun_light(True)

        #self.main_vis.scene.view.set_post_processing(False)
        
        self.mat = o3d.visualization.rendering.MaterialRecord()
        self.mat.shader = "defaultLit"
        self.mat.point_size = 50 * self.w.scaling

        self.mat_ref = o3d.visualization.rendering.MaterialRecord()
        self.mat_ref.shader = 'defaultLitSSR'
        self.mat_ref.transmission = 1.0
        self.mat_ref.point_size = 50 * self.w.scaling

        self.add_meshes()

        bbox = self.main_vis.scene.bounding_box
        #bbox = o3d.geometry.AxisAlignedBoundingBox([-100, -100, -100],
        #                                           [100, 100, 100])
        self.main_vis.setup_camera(60.0, bbox, bbox.get_center())
        self.main_vis.look_at(bbox.get_center(), np.array([[-2500],[0],[-500]]), np.array([[0],[0],[-1]]))
        self.w.add_child(self.main_vis)

        # # The menu is global (because the macOS menu is global), so only create
        # # it once, no matter how many windows are created
        # if o3d.visualization.gui.Application.instance.menubar is None:
        #     if isMacOS:
        #         app_menu = o3d.visualization.gui.Menu()
        #         app_menu.add_item("Quit", Viewer3D.MENU_QUIT)
        #     debug_menu = o3d.visualization.gui.Menu()
        #     debug_menu.add_item("Add Sphere", Viewer3D.MENU_SPHERE)
        #     debug_menu.add_item("Add Random Spheres", Viewer3D.MENU_RANDOM)
        #     if not isMacOS:
        #         debug_menu.add_separator()
        #         debug_menu.add_item("Quit", Viewer3D.MENU_QUIT)

        #     menu = o3d.visualization.gui.Menu()
        #     if isMacOS:
        #         # macOS will name the first menu item for the running application
        #         # (in our case, probably "Python"), regardless of what we call
        #         # it. This is the application menu, and it is where the
        #         # About..., Preferences..., and Quit menu items typically go.
        #         menu.add_menu("Example", app_menu)
        #         menu.add_menu("Debug", debug_menu)
        #     else:
        #         menu.add_menu("Debug", debug_menu)
        #     o3d.visualization.gui.Application.instance.menubar = menu

        # # The menubar is global, but we need to connect the menu items to the
        # # window, so that the window can call the appropriate function when the
        # # menu item is activated.
        # self.w.set_on_menu_item_activated(Viewer3D.MENU_SPHERE,
        #                                        self._on_menu_sphere)
        # self.w.set_on_menu_item_activated(Viewer3D.MENU_RANDOM,
        #                                        self._on_menu_random)
        # self.w.set_on_menu_item_activated(Viewer3D.MENU_QUIT,
        #                                        self._on_menu_quit)

        test_label = self.main_vis.add_3d_label([0,0,-400], 'TEST')
        test_label.color = o3d.visualization.gui.Color(1.0, 1.0, 1.0, 1.0)

        self.interface = IvyMessagesInterface("rotwingviewer")
        self.interface.subscribe(self.message_recv)

        self.ahrs_ref_quat = None
        self.wing_rotation_controller = None
        self.actuators = None

        self.T_rotation = np.matrix(np.eye(4))
        self.T_rotation_ref = np.matrix(np.eye(4))
        self.T_rotation_wing = np.matrix(np.eye(4))
        self.T_rotation_wing_ref = np.matrix(np.eye(4))
        self.T_rotating_wing_final = np.matrix(np.eye(4))
        self.T_rotating_wing_final_ref = np.matrix(np.eye(4))
    
    def add_meshes(self):
        R_standard = o3d.geometry.get_rotation_matrix_from_zxy(np.array([[0],[np.deg2rad(-90)],[np.deg2rad(-90)]]))
        T_standard = np.array([[680],[0],[0]])

        self.mesh_wing = o3d.io.read_triangle_mesh("sw/ground_segment/python/rot_wing_visualizer/import_ply_model/RotatingWingV3/WingQuad.ply")
        self.mesh_wing.compute_vertex_normals()
        self.mesh_wing.rotate(R_standard, np.array([[0],[0],[0]]))
        self.mesh_wing.translate(T_standard)
        self.mesh_fuselage = o3d.io.read_triangle_mesh("sw/ground_segment/python/rot_wing_visualizer/import_ply_model/RotatingWingV3/RWV4 NonMoving.ply")
        self.mesh_fuselage.compute_vertex_normals()
        self.mesh_fuselage.rotate(R_standard, np.array([[0],[0],[0]]))
        self.mesh_fuselage.translate(T_standard)
        self.mesh_vert_tail = o3d.io.read_triangle_mesh("sw/ground_segment/python/rot_wing_visualizer/import_ply_model/RotatingWingV3/RudderSolid.ply")
        self.mesh_vert_tail.compute_vertex_normals()
        self.mesh_vert_tail.rotate(R_standard, np.array([[0],[0],[0]]))
        self.mesh_vert_tail.translate(T_standard)
        self.mesh_hor_tail = o3d.io.read_triangle_mesh("sw/ground_segment/python/rot_wing_visualizer/import_ply_model/RotatingWingV3/HoriTail.ply")
        self.mesh_hor_tail.compute_vertex_normals()
        self.mesh_hor_tail.rotate(R_standard, np.array([[0],[0],[0]]))
        self.mesh_hor_tail.translate(T_standard)
        self.mesh_motor_F = o3d.io.read_triangle_mesh("sw/ground_segment/python/rot_wing_visualizer/import_ply_model/RotatingWingV3/15 inch prop Disc.ply")
        self.mesh_motor_F.compute_vertex_normals()
        self.mesh_motor_F.rotate(R_standard, np.array([[0],[0],[0]]))
        self.mesh_motor_F.translate(np.array([[-232],[0],[124]]))
        self.mesh_motor_F.translate(T_standard)
        self.mesh_motor_B = o3d.io.read_triangle_mesh("sw/ground_segment/python/rot_wing_visualizer/import_ply_model/RotatingWingV3/15 inch prop Disc.ply")
        self.mesh_motor_B.compute_vertex_normals()
        self.mesh_motor_B.rotate(R_standard, np.array([[0],[0],[0]]))
        self.mesh_motor_B.translate(np.array([[-1129],[0],[124]]))
        self.mesh_motor_B.translate(T_standard)
        self.mesh_motor_R = o3d.io.read_triangle_mesh("sw/ground_segment/python/rot_wing_visualizer/import_ply_model/RotatingWingV3/15 inch prop Disc.ply")
        self.mesh_motor_R.compute_vertex_normals()
        self.mesh_motor_R.rotate(R_standard, np.array([[0],[0],[0]]))
        self.mesh_motor_R.translate(np.array([[-1050],[0],[-152]]))
        self.mesh_motor_R.translate(T_standard)
        self.mesh_motor_L = o3d.io.read_triangle_mesh("sw/ground_segment/python/rot_wing_visualizer/import_ply_model/RotatingWingV3/15 inch prop Disc.ply")
        self.mesh_motor_L.compute_vertex_normals()
        self.mesh_motor_L.rotate(R_standard, np.array([[0],[0],[0]]))
        self.mesh_motor_L.translate(np.array([[-310],[0],[-152]]))
        self.mesh_motor_L.translate(T_standard)
        # self.mesh_motor_pusher = o3d.io.read_triangle_mesh("sw/ground_segment/python/rot_wing_visualizer/import_ply_model/RotatingWingV3/15 inch prop Disc.ply")
        # self.mesh_motor_pusher.compute_vertex_normals()
        # self.mesh_motor_pusher.rotate(R_standard, np.array([[0],[0],[0]]))
        # self.mesh_motor_pusher.translate(np.array([[-1050],[0],[-152]]))
        # self.mesh_motor_pusher.translate(T_standard)
        # self.mesh_rudder = o3d.io.read_triangle_mesh("sw/ground_segment/python/rot_wing_visualizer/import_ply_model/RotatingWingV3/Rudder.ply")
        # self.mesh_rudder.compute_vertex_normals()
        # self.mesh_rudder.rotate(R_standard, np.array([[0],[0],[0]]))
        # self.mesh_rudder.translate(np.array([[-806.6],[0],[-152]]))
        # self.mesh_rudder.translate(T_standard)
        # self.mesh_l_flap = o3d.io.read_triangle_mesh("sw/ground_segment/python/rot_wing_visualizer/import_ply_model/RotatingWingV3/LeftFlap.ply")
        # self.mesh_l_flap.rotate(R_standard, np.array([[0],[0],[0]]))
        # self.mesh_l_flap.compute_vertex_normals()
        # self.mesh_l_aileron = o3d.io.read_triangle_mesh("sw/ground_segment/python/rot_wing_visualizer/import_ply_model/RotatingWingV3/LeftAileron.ply")
        # self.mesh_l_aileron.rotate(R_standard, np.array([[0],[0],[0]]))
        # self.mesh_l_aileron.compute_vertex_normals()
        # self.mesh_r_flap = o3d.io.read_triangle_mesh("sw/ground_segment/python/rot_wing_visualizer/import_ply_model/RotatingWingV3/RightFlap.ply")
        # self.mesh_r_flap.rotate(R_standard, np.array([[0],[0],[0]]))
        # self.mesh_r_flap.compute_vertex_normals()
        # self.mesh_r_aileron = o3d.io.read_triangle_mesh("sw/ground_segment/python/rot_wing_visualizer/import_ply_model/RotatingWingV3/RightAileron.ply")
        # self.mesh_r_aileron.rotate(R_standard, np.array([[0],[0],[0]]))
        # self.mesh_r_aileron.compute_vertex_normals()

        mesh_coordinate = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1000, origin=np.array([0.0, 0.0, 0.0]))

        self.main_vis.scene.add_geometry("wing", self.mesh_wing, self.mat)
        self.main_vis.scene.add_geometry("fuselage", self.mesh_fuselage, self.mat)
        self.main_vis.scene.add_geometry("coordinate", mesh_coordinate, self.mat)
        self.main_vis.scene.add_geometry("vert_tail", self.mesh_vert_tail, self.mat)
        self.main_vis.scene.add_geometry("ḧor_tail", self.mesh_hor_tail, self.mat)
        self.main_vis.scene.add_geometry("motor_F", self.mesh_motor_F, self.mat)
        self.main_vis.scene.add_geometry("motor_B", self.mesh_motor_B, self.mat)
        self.main_vis.scene.add_geometry("motor_R", self.mesh_motor_R, self.mat)
        self.main_vis.scene.add_geometry("motor_L", self.mesh_motor_L, self.mat)
        # self.main_vis.scene.add_geometry("rudder", self.mesh_rudder, self.mat)
        # self.main_vis.scene.add_geometry("l_flap", self.mesh_l_flap, self.mat)
        # self.main_vis.scene.add_geometry("l_aileron", self.mesh_l_aileron, self.mat)
        # self.main_vis.scene.add_geometry("r_flap", self.mesh_r_flap, self.mat)
        # self.main_vis.scene.add_geometry("r_aileron", self.mesh_r_aileron, self.mat)

        # Add reference
        self.main_vis.scene.add_geometry("fuselage_ref", self.mesh_fuselage, self.mat_ref)
        self.main_vis.scene.add_geometry("wing_ref", self.mesh_wing, self.mat_ref)
    
    def add_sphere(self):
        self._id += 1
        mat = o3d.visualization.rendering.MaterialRecord()
        mat.base_color = [
            random.random(),
            random.random(),
            random.random(), 1.0
        ]
        mat.shader = "defaultLit"
        sphere = o3d.geometry.TriangleMesh.create_sphere(0.5)
        sphere.compute_vertex_normals()
        sphere.translate([
            10.0 * random.uniform(-1.0, 1.0), 10.0 * random.uniform(-1.0, 1.0),
            10.0 * random.uniform(-1.0, 1.0)
        ])
        self.main_vis.scene.add_geometry("sphere" + str(self._id), sphere, mat)

    def _on_menu_sphere(self):
        # GUI callbacks happen on the main thread, so we can do everything
        # normally here.
        self.add_sphere()

    def _on_menu_random(self):
        # This adds spheres asynchronously. This pattern is useful if you have
        # data coming in from another source than user interaction.
        def thread_main():
            for _ in range(0, 20):
                # We can only modify GUI objects on the main thread, so we
                # need to post the function to call to the main thread.
                o3d.visualization.gui.Application.instance.post_to_main_thread(
                    self.w, self.add_sphere)
                time.sleep(1)

        threading.Thread(target=thread_main).start()

    def _on_menu_quit(self):
        o3d.visualization.gui.Application.instance.quit()

    def update_actuator_color(self, name, min_cmd, max_cmd, cmd, transparant = True):
        cmd_fraction = (cmd - min_cmd) / (max_cmd - min_cmd)

        # Color min cmd = Blue
        # Color neutral = Green
        # Color max cmd = red

        color_rgb = np.ones(4)
        if cmd_fraction > 0.5:
            color_rgb[0] = (cmd_fraction - 0.5) * 2.
            color_rgb[1] = 1. - color_rgb[0]
            color_rgb[2] = 0.
        else:
            color_rgb[0] = 0.
            color_rgb[2] = (0.5 - cmd_fraction) * 2.
            color_rgb[1] = 1. - color_rgb[2]

        mat = o3d.visualization.rendering.MaterialRecord()
        mat.base_color = color_rgb
        if transparant:
            mat.shader = 'defaultLitSSR'
            mat.transmission = 0.5
        else:
            mat.shader = "defaultLit"
        self.main_vis.scene.modify_geometry_material(name, mat)

    def update_actuator_colors(self):
        # update motors
        self.update_actuator_color("motor_F", 1000, 8191, self.actuators.get_actuator_value(3))
        self.update_actuator_color("motor_R", 1000, 8191, self.actuators.get_actuator_value(4))
        self.update_actuator_color("motor_B", 1000, 8191, self.actuators.get_actuator_value(5))
        self.update_actuator_color("motor_L", 1000, 8191, self.actuators.get_actuator_value(6))
        # update elevator
        self.update_actuator_color("ḧor_tail", 8191, -1395, self.actuators.get_actuator_value(8), transparant=False)

    def update_fuselage(self):
        self.main_vis.scene.set_geometry_transform("fuselage", self.T_rotation)
        self.main_vis.scene.set_geometry_transform("fuselage_ref", self.T_rotation_ref)

    def update_vert_tail(self):
        self.main_vis.scene.set_geometry_transform("vert_tail", self.T_rotation)
    
    def update_wing(self):
        if self.wing_rotation_controller == None:
            return

        wing_rotation_matrix = np.matrix(o3d.geometry.get_rotation_matrix_from_axis_angle(np.array([[0],[0],[self.wing_rotation_controller.get_wing_angle() - np.pi/2]])))
        wing_rotation_matrix_ref = np.matrix(o3d.geometry.get_rotation_matrix_from_axis_angle(np.array([[0],[0],[self.wing_rotation_controller.get_wing_angle_sp() - np.pi/2]])))

        self.T_rotation_wing = np.matrix(np.eye(4))
        self.T_rotation_wing[:3,:3] = wing_rotation_matrix
        self.T_rotating_wing_final = self.T_rotation * self.T_rotation_wing

        self.T_rotation_wing_ref = np.matrix(np.eye(4))
        self.T_rotation_wing_ref[:3,:3] = wing_rotation_matrix_ref
        self.T_rotating_wing_final_ref = self.T_rotation * self.T_rotation_wing_ref

        self.main_vis.scene.set_geometry_transform("wing", self.T_rotating_wing_final)
        self.main_vis.scene.set_geometry_transform("wing_ref", self.T_rotating_wing_final_ref)

        def thread_main():
            o3d.visualization.gui.Application.instance.post_to_main_thread(
                    self.w, self.update_motor_R)
            o3d.visualization.gui.Application.instance.post_to_main_thread(
                    self.w, self.update_motor_L)

        threading.Thread(target=thread_main).start()

    def update_elevator(self):
        cmd_elevator = self.actuators.get_actuator_value(8)
        min_angle = np.deg2rad(-36.6) 
        max_angle = np.deg2rad(10.3)
        min_cmd = 8191.
        max_cmd = -1395.

        # Calc angle from command
        angle_elevator = cmd_elevator * (max_angle - min_angle) / (max_cmd - min_cmd)

        elevator_rotation_matrix = np.matrix(o3d.geometry.get_rotation_matrix_from_axis_angle(np.array([[0],[-angle_elevator],[0]])))

        T_offset_hinge = np.matrix(np.eye(4))
        T_offset_hinge[:3,3] = np.matrix([[1486.6-680.],[8.3],[-32]])
        T_elevator = np.eye(4)
        T_elevator[:3,:3] =  np.matrix(elevator_rotation_matrix)

        T_elevator_final = self.T_rotation * np.linalg.inv(T_offset_hinge) * T_elevator * T_offset_hinge
        self.main_vis.scene.set_geometry_transform("ḧor_tail", T_elevator_final)

    def update_motor_F(self):
        self.main_vis.scene.set_geometry_transform("motor_F", self.T_rotation)
    
    def update_motor_R(self):
        if self.wing_rotation_controller == None:
            return
        
        self.main_vis.scene.set_geometry_transform("motor_R", self.T_rotating_wing_final)

    def update_motor_B(self):
        self.main_vis.scene.set_geometry_transform("motor_B", self.T_rotation)
    
    def update_motor_L(self):
        if self.wing_rotation_controller == None:
            return
        
        self.main_vis.scene.set_geometry_transform("motor_L", self.T_rotating_wing_final)

    def update_actuators(self):
        if self.actuators == None:
            return
        
        def thread_main():
            o3d.visualization.gui.Application.instance.post_to_main_thread(
                    self.w, self.update_elevator)
            o3d.visualization.gui.Application.instance.post_to_main_thread(
                    self.w, self.update_motor_F)
            o3d.visualization.gui.Application.instance.post_to_main_thread(
                    self.w, self.update_motor_B)
            o3d.visualization.gui.Application.instance.post_to_main_thread(
                    self.w, self.update_motor_R)
            o3d.visualization.gui.Application.instance.post_to_main_thread(
                    self.w, self.update_motor_L)
            o3d.visualization.gui.Application.instance.post_to_main_thread(
                    self.w, self.update_actuator_colors)

        threading.Thread(target=thread_main).start()

    def update_orientation_from_quat(self):
        if self.ahrs_ref_quat == None:
            return
        
        # Calculate drone rotation matrix
        rotation_matrix = np.matrix(o3d.geometry.get_rotation_matrix_from_quaternion(self.ahrs_ref_quat.get_body_quaternion()))
        rotation_matrix_ref = np.matrix(o3d.geometry.get_rotation_matrix_from_quaternion(self.ahrs_ref_quat.get_ref_quaternion()))

        self.T_rotation = np.matrix(np.eye(4))
        self.T_rotation[:3,:3] = rotation_matrix
        
        self.T_rotation_ref = np.matrix(np.eye(4))
        self.T_rotation_ref[:3,:3] = rotation_matrix_ref

        def thread_main():
            o3d.visualization.gui.Application.instance.post_to_main_thread(
                    self.w, self.update_fuselage)
            o3d.visualization.gui.Application.instance.post_to_main_thread(
                    self.w, self.update_vert_tail)
            o3d.visualization.gui.Application.instance.post_to_main_thread(
                    self.w, self.update_wing)

        threading.Thread(target=thread_main).start()

        self.update_actuators()
        
    def message_recv(self, ac_id, msg):
        if msg.name == "AHRS_REF_QUAT":
            self.ahrs_ref_quat = AHRSRefQuatMessage(msg)
            self.update_orientation_from_quat()

        if msg.name == "ROT_WING_CONTROLLER":
            self.wing_rotation_controller = RotWingControllerMessage(msg)

        if msg.name == "ACTUATORS":
            self.actuators = ActuatorsMessage(msg)
            self.update_actuators()
        

def main():
    o3d.visualization.gui.Application.instance.initialize()
    Viewer3D("Rot wing visualizer")
    o3d.visualization.gui.Application.instance.run()

if __name__ == "__main__":
    main()