import numpy as np
from scipy.interpolate import interp1d
from enum import Enum


def extract_csv(filepath, transpose=False):
    """
    Reads trajectory data from a CSV file

    Assumes the data is structured in columns as: time, x, y, z, psi
    """
    
    try:
        if filepath.lower().endswith(('.csv', '.txt')):
            data = np.loadtxt(filepath, delimiter=',')
        elif filepath.lower().endswith('.npz'):
            data = np.load(filepath)
        else:
            raise Exception('Unsupported file format')
        if transpose:
            data = data.T
        return data
    except Exception as e:
        print(f"Error while loding {filepath} [{e}]")
        return None

class FrameType(Enum):
    ENU = 1
    NED = 2

class TrajectoryInterpolator:
    """
    Pre-calculates derivatives and provides an interpolated full state vector
    for any given time `t`.

    Assumes the data is structured in columns as either:
     - time, x, y, z, psi
     - time, x, y, z, psi, vx, vy, vz, psi_d
     - time, x, y, z, psi, vx, vy, vz, psi_d, ax, ay, az, psi_dd
    Converted to NED frame if needed
    """
    
    def __init__(self, data, frame:FrameType, verbose=True):

        self.time_vector = data[:, 0]
        state_matrix = np.empty((data.shape[0], data.shape[1]-1))
        world_position_data = np.empty((data.shape[0], 3))
        world_velocity_data = np.empty((data.shape[0], 3))
        world_acceleration_data = np.empty((data.shape[0], 3))
        world_yaw_data = np.empty((data.shape[0], 3))

        if frame == FrameType.NED:
            state_matrix = data[:, 1:]
        else:
            state_matrix[:,0] = data[:,2]
            state_matrix[:,1] = data[:,1]
            state_matrix[:,2] = -data[:,3]
            state_matrix[:,3] = data[:,4]
            if data.shape[1] >= 9:
                state_matrix[:,4] = data[:,5]
                state_matrix[:,5] = data[:,6]
                state_matrix[:,6] = -data[:,7]
                state_matrix[:,7] = data[:,8]
            if data.shape[1] == 13:
                state_matrix[:,8] = data[:,9]
                state_matrix[:,9] = data[:,10]
                state_matrix[:,10] = -data[:,11]
                state_matrix[:,11] = data[:,12]

        
        if not np.all(np.diff(self.time_vector) > 0):
            raise ValueError("The time vector must be strictly increasing.")

        # --- Create interpolator for position and yaw ---
        world_position_data = state_matrix[:, [0, 1, 2]]
        self.position_interpolator = interp1d(self.time_vector, world_position_data,
                kind='linear', axis=0, bounds_error=False, fill_value="extrapolate")

        world_yaw_data = state_matrix[:, 3] # only yan angle for now
        self.yaw_interpolator = interp1d(self.time_vector, world_yaw_data,
                kind='linear', axis=0, bounds_error=False, fill_value="extrapolate")
        
        # --- Pre-calculate derivatives from the original data if needed ---
        
        if data.shape[1] >= 9:
            world_velocity_data = state_matrix[:, [4, 5, 6]]
        else:
            # Calculate world velocity using np.gradient
            world_velocity_data = np.gradient(world_position_data, self.time_vector, axis=0)
        
        if data.shape[1] == 13:
            world_acceleration_data = state_matrix[:, [9, 10, 11]]
        else:
            # Calculate world acceleration by taking the derivative of the velocity
            world_acceleration_data = np.gradient(world_velocity_data, self.time_vector, axis=0)
        
        # --- Create dedicated interpolators for the derivatives ---
        self.velocity_interpolator = interp1d(self.time_vector, world_velocity_data,
                kind='linear', axis=0, bounds_error=False, fill_value="extrapolate")
        self.acceleration_interpolator = interp1d(self.time_vector, world_acceleration_data,
                kind='linear', axis=0, bounds_error=False, fill_value="extrapolate")
        
        if verbose:
            print(f"Time range: {self.time_vector.min():.2f}s to {self.time_vector.max():.2f}s")

    def get_initial_time(self):
        return self.time_vector[0]

    def get_final_time(self):
        return self.time_vector[-1]

    def get_full_state(self, t):
        """
        Gets the interpolated state and kinematic derivatives at a specific time in NED frame

        Returns:
            A tuple containing:
            (position, world_velocity, world_acceleration, yaw_angle)
        """
        # Get position and yaw angle
        position = self.position_interpolator(t)
        yaw_angle = self.yaw_interpolator(t)
        
        # Get the pre-calculated derivatives
        world_velocity = self.velocity_interpolator(t)
        world_acceleration = self.acceleration_interpolator(t)

        if position.ndim == 1:
            # Handle a single time 't'
            return position, world_velocity, world_acceleration, yaw_angle
        else:
            # Handle an array of times 't'
            # Transpose to match shape convention (components, time)
            return position.T, world_velocity.T, world_acceleration.T, yaw_angle.T

