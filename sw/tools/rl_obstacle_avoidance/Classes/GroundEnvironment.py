import numpy as np
import math
import pandas as pd
import bisect
import random
import matplotlib
import matplotlib.pyplot as plt
from collections import Counter
from scipy.signal import butter, lfilter, lfilter_zi, lfiltic

# Define default settings for the environment
dt_factor = 1
default_env_settings = {
    'dt': 0.001953125*dt_factor,
    'quadrotor_mass': 0.42,  # kg
    'rpm_bounds': (0, 9800),
    'termination': True,
    'save_timesteps': round(512/dt_factor),
    'no_action_timesteps': round(1/dt_factor),
    'hover_timesteps': round(256/dt_factor),
    'descend_speed': (0.3, 0.3),
    'controller_gains': {
        'tau': 0.35, # On bebop: 0.25
        'P': 0.0,  # Proportional gain, on bebop: 263/(2**7)
        'I': 0.0,  # Integral gain, on bebop: 20 / (2**16)
        'D': 5*600/(2**16),   # Derivative gain, on bebop: 600 / (2**16)
    },
    'ground_pos': 0.0,  # Z-position of the ground
    'ground_termination_distance': 0.05,  # Distance from the ground after which we consider it a crash
    'min_save_height': 0.25,
    'actuator_cutoff': 15,
    'delay_cutoff': 15,
    # 'action_every_x_timesteps': 10,
}


class GroundEnvironment:
    """This is the environment simulating the ground effect"""

    def __init__(self, dt, controller_gains, quadrotor_mass, rpm_bounds, ground_pos, ground_termination_distance,
                 min_save_height, termination, descend_speed,
                 actuator_cutoff, delay_cutoff, rolling_window_size=None,
                 save_timesteps=0, no_action_timesteps=0, hover_timesteps=0, **kwargs):
        # Define main environment parameters
        self.dt = dt
        self.quadrotor_mass = quadrotor_mass
        self.termination = termination
        self.terminal = False
        self.save_timesteps = save_timesteps
        self.no_action_timesteps = no_action_timesteps
        self.hover_timesteps = hover_timesteps
        self.ground_pos = ground_pos
        self.ground_termination_distance = ground_termination_distance
        self.min_save_height = min_save_height
        self.prev_action = 1.0
        self.rolling_window_size = rolling_window_size

        # Rotors
        self.rpm_bounds = rpm_bounds
        self.k_over_m = [3.7352494607033004E-06, 3.7352494607033004E-06, 3.7352494607033004E-06, 3.7352494607033004E-06]
        self.thrust_hover = math.sqrt(9.81 / sum(self.k_over_m)) / (self.rpm_bounds[1]*2*math.pi / 60)
        # print(self.thrust_hover)

        # Define environment statistics
        self.env_statistics = {
            'init_state': None,
            'crashed': False,
            'false_save': False,
            'saved': False,
            'save_height': False,
            'save_timestep': None,
            'result': 'Timeout'
        }

        # Define conventional controller gains
        self.controller_gains = controller_gains


        # Create working variables for the conventional PID controller
        self.controller_last_error = 0.0 # For the derivative controller
        self.controller_integral_error = 0.0 # For the integral controller
        self.gv_zd_ref = self.get_starting_state()['dz'] # For the the integrator in the zd controller
        self.gv_z_ref = self.get_starting_state()['z']

        #  Delay introduced by optitrack etc
        self.delay_filter_b, self.delay_filter_a = self.get_butter_lowpass_coef(cutoff=delay_cutoff, fs=int(1 / dt), order=2)
        self.delay_filter_zi = None  # [lfilter_zi(self.actuators_b, self.actuators_a) for k in range(0, 4)]

        # Set actuator state
        self.actuators_b, self.actuators_a = self.get_butter_lowpass_coef(cutoff=actuator_cutoff, fs=int(1/dt), order=2)
        self.actuators_zi = [None] * 4  # [lfilter_zi(self.actuators_b, self.actuators_a) for k in range(0, 4)]

        # Set noise state
        self.noise_b, self.noise_a = self.get_butter_lowpass_coef(cutoff=1.8, fs=int(1/dt), order=2)
        # self.noise_zi = lfiltic(self.noise_b, self.noise_a, 0.0, 0.0)
        self.noise_zi = [0.0, 0.0]

        self.descend_speed = np.random.uniform(low=descend_speed[0], high=descend_speed[1])

        return

        # # Load the ground measurements
        # self.F_ext_path = F_ext_path
        # F_ext_df = pd.read_csv(self.F_ext_path)
        #
        # # Create a hashtable for F_ext
        # self.z_heights = np.linspace(ground_termination_distance, 1.2, 100)
        # z_hashtable = {}
        # F_ext_df['bin'] = np.digitize(F_ext_df['ENU position z'], self.z_heights)
        #
        #
        # for bin_id, height in enumerate(self.z_heights):
        #     z_hashtable[bin_id] = F_ext_df[F_ext_df['bin'] == bin_id]['F_ext_over_m'].values
        # # Add final value for everything above the highest measurement point
        # z_hashtable[bin_id+1] = [0.0]
        #
        # for key, value in z_hashtable.items():
        #     if len(value) == 0:
        #         print(self.z_heights[key])
        #
        # self.F_ext_hash = z_hashtable


    def get_starting_state(self):
        state = {
            't': 0.0,
            'z': 1.0,
            'dz': 0.0,
            'ddz': 0.0,
            'rpm_obs': [0.0, 0.0, 0.0, 0.0],
            'F_ext': 0.0,
            'F_ext_rolling': 0.0,
            'F_ext_hist': [],
            'ref': 0.0,
            'rpm_cmd': [],
            'rpm_actual': [],
            'prev_action': 1.0,

        }
        return state

    def get_flight_plan(self, t):
        """"The reference from the flight plan"""
        vmode = 'climb'
        if t < 0.1:
            zd_ref = 0.0
        else:
            zd_ref = -self.descend_speed
        return vmode, zd_ref

    def get_hover_ref(self):
        vmode = 'climb'
        zd_ref = 0.0

        return vmode, zd_ref

    def get_save_ref(self):
        vmode = 'climb'
        zd_ref = 1.0

        return vmode, zd_ref

    def step(self, old_state, action):
        """"Take the environment one timestep forward"""

        # Speed up learning by not allowing action 2 above 0.25m
        real_action = {**action}
        # if 'intervene' in action and action['intervene'] == 2 and old_state['z'] > 0.25:
        #     real_action['intervene'] = 1

        # Calculate new state based on the chosen action and system dynamics
        new_state = self.step_dynamics(old_state, real_action)

        # Calculate reward for the action and resulting new state
        reward = self.get_reward(new_state, action)

        # Check for termination
        terminal = self.check_for_termination(new_state, action)
        if terminal and self.termination is True:
            self.terminal = True

        return reward, new_state

    def step_dynamics(self, old_state, action):
        """Calculate new state based on the chosen action and system dynamics"""
        # Calculate new time
        t = old_state['t'] + self.dt

        # |The switch|
        # Check which reference is passed to the PID controller
        if 'intervene' in action and action['intervene'] == 2:
            # The RL agent has intervened and triggered a save
            vmode, ref = self.get_save_ref()
        elif 'intervene' in action and action['intervene'] == 3:
            # The RL agent has intervened and trigger a hover
            vmode, ref = self.get_hover_ref()
        else:
            # Follow the flight plan
            vmode, ref = self.get_flight_plan(t)

        # |The PID controller|
        rpm_commanded = self.get_conv_controller_action(old_state, vmode, ref, True)

        # |Actuator dynamics|
        rpm_actual = self.actuator_dynamics(old_state, rpm_commanded)

        # |Get external force (both from measurements, and from theory)|
        F_ext = self.estimate_F_ext(old_state['z'])
        F_ext_theory = self.F_ext_from_theory(old_state['z'])

        # |Simplified quadrotor dynamics|
        ddz = self.quadrotor_dynamics(rpm_actual, F_ext[0])

        # |Integrate acceleration|
        # Calculate speed
        dz = old_state['dz'] + (old_state['ddz'] + ddz)/2*self.dt

        # Calculate position
        z = old_state['z'] + (old_state['dz'] + dz)/2*self.dt

        # |State estimator|
        F_ext_hist = old_state['F_ext_hist']+[F_ext]
        if self.rolling_window_size is not None:
            if len(F_ext_hist) > self.rolling_window_size:
                F_ext_hist.pop(0)
            F_ext_rolling = sum(F_ext_hist) / len(F_ext_hist)
        else:
            F_ext_rolling = 0.0

        # Update new state
        new_state = {
            't': t,
            'z': z,
            'dz': dz,
            'ddz': ddz,
            'rpm_obs': rpm_actual,
            'F_ext': F_ext,
            'F_ext_rolling': F_ext_rolling,
            'F_ext_hist': F_ext_hist,
            'ref': ref,
            'rpm_cmd': rpm_commanded,
            'rpm_actual': rpm_actual,
            'prev_action': self.prev_action
        }

        self.prev_action = action['intervene']

        return new_state

    def actuator_dynamics(self, old_state, rpm_commanded):
        """This function mimics the actuator dynamics, from commanded RPM to actual RPM"""
        rpm_actual = []
        # Loop over rotors
        for rotor_id, rotor_cmd in enumerate(rpm_commanded):

            # Get previous filter state
            zi = self.actuators_zi[rotor_id]

            # If first time, set initial conditions
            if zi is None:
                zi = lfiltic(self.actuators_b, self.actuators_a, [rotor_cmd]*100, [rotor_cmd]*100)

            # Calculate actual rotor speed and new rotor state
            rpm_filtered, zf = lfilter(self.actuators_b, self.actuators_a, [rotor_cmd], zi=zi)

            # Save actual RPM and filter state
            rpm_actual.append(rpm_filtered[0])
            self.actuators_zi[rotor_id] = zf

        return rpm_actual

    def get_delayed_speed(self, state):
        # Get previous filter state
        zi = self.delay_filter_zi

        # If first time, set initial conditions
        if zi is None:
            zi = lfiltic(self.delay_filter_b, self.delay_filter_a, [0] * 100, [0] * 100)

        speed_filt, zf = lfilter(self.delay_filter_b, self.delay_filter_a, [state['dz']], zi=zi)

        self.delay_filter_zi = zf

        return speed_filt[0]

    def quadrotor_dynamics(self, rpm_actual, F_ext):

        # Calculate produced thrust
        rotor_speeds = [rpm*2*math.pi/60 for rpm in rpm_actual]
        k = [k*self.quadrotor_mass for k in self.k_over_m]
        F_thrust = 0.0
        for id, omega in enumerate(rotor_speeds):
            F_thrust += k[id]*(omega**2)

        # Calculate net force (downward is positive)
        F_gravity = 9.81*self.quadrotor_mass
        F_ext_actual = F_ext * self.quadrotor_mass
        F_result = -F_thrust + F_gravity + F_ext_actual
        # print("{0} + {1} + {2} = {3}".format(-F_thrust, F_gravity, F_ext_actual, F_result))

        # Calculate acceleration (upward is positive)
        ddz = -F_result/self.quadrotor_mass

        return ddz


    def get_reward(self, new_state, action):
        """Determine the numerical reward for a action and resulting new state"""

        # Start of with a 0 reward
        reward = 0

        if 'intervene' in action:
            # Negative reward for action 2 when height > 0.25 meter
            if action['intervene'] == 2 and self.env_statistics['save_height'] is False:
                # Init of save
                self.env_statistics['save_height'] = new_state['z']
                self.env_statistics['save_timestep'] = new_state['t']

                # Only give it if the save was started above the minimum save height
                if self.env_statistics['save_height'] > self.min_save_height:
                    reward += -500
                else:
                    # Reward for correct save depends on height above the ground (0 is perfect, 1 is worst)
                    dist_factor = (self.min_save_height - self.env_statistics['save_height']) / (self.min_save_height - self.ground_termination_distance)
                    # dist_factor = (self.env_statistics['save_height'] - self.ground_termination_distance) / (self.min_save_height - self.ground_termination_distance)

                    reward += dist_factor*-50

            # Small negative reward for hovering
            if action['intervene'] == 3:
                in_hover = True
                if new_state['z'] > self.min_save_height:
                    reward += -100/self.hover_timesteps
                else:
                    reward += -25 / self.hover_timesteps

            # Negative reward for crash
            if new_state['z'] <= self.ground_pos + self.ground_termination_distance:
                reward += -2000

        return reward

    def check_number_of_timesteps_actions(self, state, action):
        # Save
        if 'intervene' in action and action['intervene'] == 2:
            if state['z'] < 0.25:
                return self.save_timesteps
            else:
                return self.save_timesteps
                # return 0

        # No-action
        if 'intervene' in action and action['intervene'] == 1:
            return self.no_action_timesteps

        # Hover
        if 'intervene' in action and action['intervene'] == 3:
            return self.hover_timesteps

    def check_for_termination(self, new_state, action):

        # Check if crashed
        if new_state['z'] <= self.ground_pos + self.ground_termination_distance:
            self.env_statistics['crashed'] = True
            self.env_statistics['crash_height'] = new_state['z']
            self.env_statistics['result'] = 'Crash'
            return True

        # Check if we're in a save
        if self.env_statistics['save_height'] is not False:
            # Check if this is the end of the save
            if new_state['t'] > self.env_statistics['save_timestep']+(self.save_timesteps-1)*self.dt:
                # Check if it was a good save, or a false save
                if self.env_statistics['save_height'] <= self.min_save_height:
                    self.env_statistics['saved'] = True
                    self.env_statistics['result'] = 'Correct Save'
                else:
                    self.env_statistics['false_save'] = True
                    self.env_statistics['result'] = 'False Save'
                # Either way, the episode has ended
                return True

        return False

    def get_conv_controller_action(self, state, vmode, ref, save_errors=False):
        """Get the force proposed by the conventional PID controller"""

        if vmode == 'climb':
            # Constants
            gv_ref_inv_thau = 1/self.controller_gains['tau']
            max_accel_down = -0.8*9.81
            max_accel_up = 2*9.81
            max_speed_down = -0.5
            max_speed_up = 0.8

            # Reference generator
            current_dz = self.get_delayed_speed(state)
            tracking_error_dz = ref - current_dz
            gv_zdd_sp = tracking_error_dz*gv_ref_inv_thau

            # Boundaries on acceleration
            gv_zdd_ref = np.clip(a=gv_zdd_sp, a_min=max_accel_down, a_max=max_accel_up)

            # Reference speed
            gv_zd_ref = self.gv_zd_ref+gv_zdd_ref*self.dt
            self.gv_zd_ref = np.clip(a=gv_zd_ref, a_min=max_speed_down, a_max=max_speed_up)

            # Reference height
            self.gv_z_ref = self.gv_z_ref + self.gv_zd_ref*self.dt

            # Vertical loop
            # Position tracking error P & I
            z_error = self.gv_z_ref-state['z']
            z_error = np.clip(a=z_error, a_min=-10.0, a_max=10.0)

            integral_error = self.controller_integral_error + z_error * self.dt
            integral_error = min(integral_error, 2000000)

            # Velocity tracking error
            derivative_error = self.gv_zd_ref-current_dz


            # Calculate P,I & D contribution
            thrust_P = self.controller_gains['P'] * z_error
            thrust_I = self.controller_gains['I'] * integral_error
            thrust_D = self.controller_gains['D'] * derivative_error

            # Sum P, I & D forces
            guidance_v_fb_cmd = thrust_P + thrust_I + thrust_D

            # Feedforward acceleration
            guidance_v_ff_cmd = (9.81 + gv_zdd_ref) * (self.thrust_hover / 9.81)

            # Calculate total required thrust
            thrust_commanded = guidance_v_fb_cmd + guidance_v_ff_cmd
            # print("Feedback: P={0}, I={1}, D={2}. Feedforward: (9.81 + {3}) * ({4} / 9.81) = {5}".format(
            #     thrust_P, thrust_I, thrust_D, gv_zdd_ref, self.thrust_hover, thrust_commanded))

            # Thrust to RPM
            rpm_commanded = thrust_commanded*(self.rpm_bounds[1]-self.rpm_bounds[0])+self.rpm_bounds[0]

            # Clip RPM to bounds and divide over the 4 rotors
            rpm_commanded = np.clip(a=rpm_commanded, a_min=self.rpm_bounds[0], a_max=self.rpm_bounds[1])
            rpm_commanded_list = [rpm_commanded for a in range(4)]

        # Save derivative and integral
        if save_errors:
            # self.controller_last_error = tracking_error
            self.controller_integral_error = integral_error

        return rpm_commanded_list


    def estimate_F_ext(self, z):
        """"Get the external force from the ground surface"""
        # F_ext = 0.0
        # window_size = 0.01
        # lower_boundary = z - window_size
        # upper_boundary = z + window_size
        #
        # lower_index = bisect.bisect_left(self.F_ext_df['ENU position z'], lower_boundary)
        # upper_index = bisect.bisect_right(self.F_ext_df['ENU position z'], upper_boundary)
        #
        # distr = self.F_ext_df.loc[lower_index:upper_index]
        # distr = self.F_ext_df[(self.F_ext_df['ENU position z'] - window_size < z) or
        #                       (self.F_ext_df['ENU position z'] + window_size < z)]

        # Pick random point from cloud
        # if len(distr) > 0:
        #     F_ext = distr.sample()['F_ext_over_m'].values[0]
        # else:
        #     F_ext = 0.0

        # z_bin = np.digitize(z, self.z_heights).item(0)
        # distr = self.F_ext_hash[z_bin]
        # if len(distr) > 0:
        #     F_ext = random.choice(distr)
        # else:
        #     print(self.z_heights[z_bin])
        #     F_ext = 0.0

        # F_ext from the ground effect
        F_ext_theory = -9.81 * (1 / (1 - (0.05 / (4 * z)) ** 2) - 1)
        F_ext_fit = -5.58364265e-12*1/(z+5.03702366e-01)-1.26883748e-02*1/(z+7.44916373e-02)**2-1.40903748e-01

        # Calculate low-pass filtered noise
        F_ext_noise_new = np.random.normal(loc=0.055, scale=0.348*4)
        F_ext_noise, self.noise_zi = lfilter(self.noise_b, self.noise_a, [F_ext_noise_new], zi=self.noise_zi)

        # Sum it to get F_ext
        F_ext = F_ext_noise + F_ext_fit
        # F_ext = np.array([F_ext_fit])

        return F_ext

    def F_ext_from_theory(self, z):
        prop_radius = 0.05 # Rotor radius
        F_ext_theory = -9.81 * (1 / (1 - (prop_radius/ (4 * z)) ** 2) - 1)

        return F_ext_theory

    def start_episode(self, state=None):
        if state is not None and self.env_statistics['init_state'] is None:
            self.env_statistics['init_state'] = {i:state[i] for i in state if i!='F_ext_hist'}

        """"This function is invoked by the training environment at the start of an episode"""
        return True

    def is_terminal(self):
        return self.terminal

    def end_episode(self):
        """"This function is invoked by the training environment at the end of an episode"""
        return True

    def visualize_episode(self,episode_history, variables=None, fig_passed=None):
        """This function creates a graphical representation of the environment in a certain episode"""
        # Default variables to plot
        if variables is None:
            variables = ['z', 'dz', 'ref', 'F_ext', 'F_ext_rolling', 'reward']

        # Get timeline used in all
        sim_t = episode_history['t']

        # Create figure if no handle is passed
        if fig_passed is None:
            fig, ax = plt.subplots()
        else:
            fig = fig_passed

        # Get and clear axis
        if len(fig.axes) == 0:
            ax = fig.add_subplot(1, 1, 1)
            ax_right = ax.twinx()
        elif len(fig.axes) == 1:
            ax = fig.axes[0]
            ax.clear()
            ax_right = ax.twinx()
        else:
            ax = fig.axes[0]
            ax_right = fig.axes[1]
            ax.clear()
            ax_right.clear()

        # Plot z
        if 'z' in variables:
            ax.plot(sim_t, episode_history['z'], label='Quadrotor', zorder=10)

        # Plot dz
        if 'dz' in variables:
            ax.plot(sim_t, episode_history['dz'], linestyle='--', label='dz')

        # Plot the speed reference
        if 'ref' in variables:
            ax.plot(sim_t, episode_history['ref'], linestyle='-', label='dz_ref')

        # Plot F_ext
        if 'F_ext' in variables:
            ax.plot(sim_t, episode_history['F_ext'], linestyle='-', label='F_ext')

        # Plot F_ext
        if 'F_ext_rolling' in variables:
            ax.plot(sim_t, episode_history['F_ext_rolling'], linestyle='--', label='F_ext_rolling')

        # Plot the rewards
        if 'reward' in variables:

            ax_right.plot(sim_t, episode_history['reward'], linestyle='-', label='reward', color='m')
            ax_right.set_xlabel('Reward')

        # Create the legend
        ax.legend()

        # Plot the ground surface
        ax.hlines(y=self.ground_pos, xmin=min(sim_t), xmax=max(sim_t))
        ax.text(x=np.mean(sim_t), y=self.ground_pos-1, s='Wall')

        # Depending on whether an axis was passed, show plot and return
        if fig_passed is None:
            plt.show()

        return fig, ax

    @staticmethod
    def calculate_statistics(episode_history, episode_results):
        number_of_episodes = len(episode_results)
        crashed = Counter(env['crashed'] for env in episode_results)[True]
        saved = Counter(env['saved'] for env in episode_results)[True]
        mean_save_height = sum([episode['save_height'] for episode in episode_results]) / number_of_episodes
        sum_of_rewards = sum([episode['episode_reward'] for episode in episode_results])
        statistics = {
            'percentage_crashed': crashed * 1.0 /number_of_episodes*100,
            'percentage_saved': saved * 1.0 / number_of_episodes * 100,
            'percentage_continued': (number_of_episodes-crashed-saved) / number_of_episodes * 100,
            'average_total_reward': sum_of_rewards/number_of_episodes,
            'mean_save_height':  mean_save_height,
        }

        return statistics

    @staticmethod
    def get_butter_lowpass_coef(cutoff, fs, order=2):
        # Get filter
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        return b, a

