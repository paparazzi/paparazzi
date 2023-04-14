import pybullet as p
import pybullet_data
import numpy as np
import time
import os


PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.getcwd())
PYBULLET_CONF_PATH = os.path.join(PPRZ_HOME, "conf/simulator/pybullet")


class BulletFDM():
    def __init__(self, dt=0.02, GUI=True, debug=False, urdf="robobee.urdf"):
        print(f"Hello from PyBullet ! dt={dt}, GUI={GUI} debug={debug}")

        pprz_src = os.getenv("PAPARAZZI_SRC")
        print(f"pprz_src = {pprz_src}")

        # self.last_time_print = 0
        
        # dt : Time step
        self.dt=dt
        if GUI:
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)

        # Add path for pybullet assests e.g : plane and grid...
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        self.planeId = p.loadURDF("plane.urdf")
        self.textureId = p.loadTexture("checker_grid.jpg")

        self.vehicle_start_pos = [0, 0, 0.2]
        self.vehicle_start_orientation = p.getQuaternionFromEuler([0, 0, 0])

        vehicule_urdf = os.path.join(PYBULLET_CONF_PATH, urdf)
        self.vehicle = p.loadURDF(vehicule_urdf, self.vehicle_start_pos, self.vehicle_start_orientation)

        # Bullet physics uses ENU frame
        p.setGravity(0, 0, -9.81,  physicsClientId=self.physicsClient)
        # Real time simulation should be off in order to apply external force and moments !
        p.setRealTimeSimulation(0,   physicsClientId=self.physicsClient)
        # dt : Time step
        p.setTimeStep(self.dt, physicsClientId=self.physicsClient)
        # Orient the camera if needed
        p.resetDebugVisualizerCamera( cameraDistance=3.5, cameraYaw=-80, cameraPitch=-40, cameraTargetPosition=[0.0, 0.0, 0.0])
        # Vehicle properties
        self.KF = 0.01 ; self.KM = 0.001
        # Initialte velocity and acceleration vectors
        self.vel = np.zeros(3)
        self.accel = np.zeros(3)
        # Initialte angular velocity and acceleration vectors
        self.ang_vel = np.zeros(3)
        self.ang_accel = np.zeros(3)

        # State
        self.observation = {}

        # For debug purposes
        if debug:
            p.setGravity(0, 0, 0,  physicsClientId=self.physicsClient)
            self.roll_Id = p.addUserDebugParameter("roll_cmd", -0.01, 0.01, 0)
            self.pitch_Id = p.addUserDebugParameter("pitch_cmd", -0.01, 0.01, 0)
            self.yaw_Id = p.addUserDebugParameter("yaw_cmd", -0.01, 0.01, 0)

    def apply_force_and_moments(self,rpm):
        ''' rpm = [ quadrotor prop rpms ]'''
        forces  = np.array(rpm**2)*self.KF
        torques = np.array(rpm**2)*self.KM

        f_noise = np.random.normal(0, 0.01, len(rpm))
        m_noise = np.random.normal(0, 0.001,len(rpm))

        # f_noise = np.zeros(4)
        # m_noise = np.zeros(4)

        forces  += f_noise
        torques += m_noise

        z_torque = (torques[0] - torques[1] + torques[2] - torques[3])
        for i in range(4): #FIXME : decide according to commands length
            p.applyExternalForce(self.vehicle,
                                    i,
                                    forceObj=[f_noise[0], f_noise[1], forces[i]],
                                    posObj=[0, 0, 0],
                                    flags=p.LINK_FRAME,
                                    physicsClientId=self.physicsClient
                                    )
        p.applyExternalTorque(self.vehicle,
                                -1,  # FIXME : this is not correct , use center of mass !!! 
                                torqueObj=[m_noise[0], m_noise[1], z_torque],
                                flags=p.LINK_FRAME,
                                physicsClientId=self.physicsClient
                                )

    def step(self,commands):
        
        if not isinstance(commands, np.ndarray):
          commands = np.array(commands)
        
        # little hack to scale commands. It should probably be in the URDF file.
        commands *= 30

        # t = time.time()
        # if (t - self.last_time_print) > 0.5:
        #     self.last_time_print = t
        #     print(f"{commands[0]:.2f}  {commands[1]:.2f}  {commands[2]:.2f}  {commands[3]:.2f}")

        self.apply_force_and_moments(commands)
        p.stepSimulation(physicsClientId=self.physicsClient)
        return self.get_observation()

    def step_debug(self,commands):
        # commands are not used...
        roll_cmd = p.readUserDebugParameter(self.roll_Id)
        pitch_cmd = p.readUserDebugParameter(self.pitch_Id)
        yaw_cmd = p.readUserDebugParameter(self.yaw_Id)

        # p.applyExternalForce(self.vehicle,
        #                         -1,
        #                         forceObj=[0, 0, 9 + yaw_cmd*100],
        #                         posObj=[0, 0, 0],
        #                         flags=p.LINK_FRAME,
        #                         physicsClientId=self.physicsClient
        #                         )

        p.applyExternalTorque(self.vehicle,
                                -1,  # FIXME : this is not correct , use center of mass !!! 
                                torqueObj=[roll_cmd, pitch_cmd, yaw_cmd],
                                flags=p.LINK_FRAME,
                                physicsClientId=self.physicsClient
                                )
        
        p.stepSimulation(physicsClientId=self.physicsClient)
        return self.get_observation()

    def get_observation(self):
        # Get observation
        v_pos, v_quat = p.getBasePositionAndOrientation(self.vehicle)
        # print(v_Pos, type(v_Pos))
        v_rpy = p.getEulerFromQuaternion(v_quat)
        v_vel, v_ang_v = p.getBaseVelocity(self.vehicle)

        self.accel = (np.array(v_vel) - self.vel)/self.dt
        self.vel = np.array(v_vel)
        self.ang_accel = (np.array(v_ang_v) - self.ang_vel)/self.dt
        self.ang_vel = np.array(v_ang_v)

        self.observation = {'pos':v_pos,
                            'quat':v_quat,
                            'rpy':v_rpy,
                            'vel':tuple(self.vel),
                            'ang_v':tuple(self.ang_vel),
                            'accel':tuple(self.accel),
                            'ang_accel':tuple(self.ang_accel)
        }
        return self.observation

    def reset(self):
        # Reset the simulation and the state
        # p.resetSimulation(physicsClientId=self.physicsClient) # FIXME : this is not the correct way to reset the simulation
        p.resetBasePositionAndOrientation(self.vehicle, self.vehicle_start_pos, self.vehicle_start_orientation)
        p.resetBaseVelocity(self.vehicle, [0, 0, 0], [0, 0, 0])

        return self.get_observation()



if __name__ == "__main__":
    from time import sleep
    debug = True
    m = BulletFDM(GUI=True, debug=debug)

    # An example simulation loop with random commands generation
    while 1:
        for i in range(200):
            # commands = np.array([10., 10., 10., 10.]) # fixed rpms
            commands = np.random.normal(13., 0.5, 4) # random rpms
            if debug:
                m.step_debug(commands)
            else:
                m.step(commands)
            sleep(0.05) # FIXME : checck the computation time and sleep to sync realtime...
        m.reset()
