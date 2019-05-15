import numpy as np
import random
import matplotlib.pyplot as plt
import collections

class ReinforcementLearningAgent():
    """General class for a reinforcement learning agent"""

    def __init__(self, state_space, action_space):
        # Define state and action spaces
        self.state_space_variables = [key for key, value in state_space.items()]
        self.state_space = state_space
        self.action_space_variables = [key for key, value in action_space.items()]
        self.action_space = action_space
        self.number_of_state_variables = len(self.state_space)
        self.number_of_action_variables = len(self.action_space)

        # Loop through all state and action variables and give each a specific axis
        self.states = collections.OrderedDict()
        for variable_id, (variable_name, variable_space) in enumerate(self.state_space.items()):
            self.states[variable_name] = {'axis': variable_id,
                                          'values': variable_space,
                                          'value_to_index': {value: index for index, value in enumerate(variable_space)},
                                          'index_to_value': {index: value for index, value in enumerate(variable_space)},
                                          'size': len(variable_space)}

        self.actions = collections.OrderedDict()
        for variable_id, (variable_name, variable_space) in enumerate(self.action_space.items()):
            self.actions[variable_name] = {'axis': variable_id + self.number_of_state_variables,
                                           'values': variable_space,
                                           'value_to_index': {value: index for index, value in enumerate(variable_space)},
                                           'index_to_value': {index: value for index, value in enumerate(variable_space)},
                                           'size': len(variable_space)}

        # Generate a default matrix structure for (state) or (state, action) matrices
        self.data_structure = self.generate_data_structure()

        # Generate a matrix used to store a count of (s,a) visitations
        self.s_a_visits = self.generate_s_a_matrix()

        # Set the parameters required for each reinforcement learning agent
        self.in_training = True

    def generate_data_structure(self):
        """Generate dictionary that saves the default structure of state or state action matrices"""
        data_structure = {
            'states_shape': (),
            'states_action_shape': (),
            'actions_shape': (),
            'states_actions_shape': (),
        }

        # Calculate size for a matrix containing a cell for each potential state
        states_shape = [value['size'] for key, value in self.states.items()]
        data_structure['states_shape'] = tuple(states_shape)

        # Calculate size for a matrix with a cell for each potential state and each action variable (like a policy)
        states_action_shape = list(states_shape)
        states_action_shape.extend([self.number_of_action_variables])
        data_structure['states_action_shape'] = tuple(states_action_shape)

        # Calculate size for a matrix containing a cell for each potential action
        actions_shape = [value['size'] for key, value in self.actions.items()]
        data_structure['actions_shape'] = tuple(actions_shape)

        # Calculate size for a matrix containing a cell for each potential state and all actions (like Q-matrix)
        states_actions_shape = states_shape + actions_shape
        data_structure['states_actions_shape'] = tuple(states_actions_shape)

        return data_structure

    def get_action_axis(self):
        action_axis = tuple(
            value['axis'] for key, value in self.actions.items()
        )
        return action_axis

    def get_state_axis(self):
        state_axis = tuple(
            value['axis'] for key, value in self.states.items()
        )
        return state_axis

    def get_s_index(self, state):
        """This function returns an (s) tuple with indexes based on a state with values"""

        # s_index = tuple(self.states[var_name]['value_to_index'][value] for var_name, value in state.items())
        s_index = tuple(
            var_dict['value_to_index'][state[var_name]]
            for var_name, var_dict in self.states.items()
        )

        return s_index

    def get_a_index(self, action):
        """This function returns an (a) tuple with indexes based on an action with values"""
        # a_index = tuple(
        #     self.actions[var_name]['value_to_index'][value] for var_name, value in action.items())
        a_index = tuple(
            var_dict['value_to_index'][action[var_name]]
            for var_name, var_dict in self.actions.items())

        return a_index

    def get_s_a_index(self, state, action):
        """This function returns an (s,a) tuple with indexes based on a state and action dictionary with values"""
        s_index = self.get_s_index(state)
        a_index = self.get_a_index(action)

        # Combine s and a into one tuple
        s_a_index = s_index + a_index

        return s_a_index

    def s_a_to_s_index(self, s_a_index):
        """This functions takes an (s,a) tuple and returns an (s) tuple"""
        s_index = s_a_index[:self.number_of_state_variables]
        return s_index

    def s_a_to_a_index(self, s_a_index):
        """This functions takes an (s,a) tuple and returns an (a) tuple"""
        a_index = s_a_index[self.number_of_state_variables:]
        return a_index

    def split_s_a_index(self, s_a_index):
        s_index = self.s_a_to_s_index(s_a_index)
        a_index = self.s_a_to_a_index(s_a_index)
        return s_index, a_index

    def get_state_from_s_index(self, s_index):
        """This function returns an state dictionary with values based on an (s) tuple with indexes"""
        state = {}
        for state_dim_index, (state_name, action_prop) in enumerate(self.states.items()):
            state_index = s_index[state_dim_index]
            state[state_name] = action_prop['index_to_value'][state_index]

        return state

    def get_action_from_action_index(self, a_index):
        """This function returns an action dictionary with values based on an (a) tuple with indexes"""
        action = {}
        for action_dim_index, (action_name, action_prop) in enumerate(self.actions.items()):
            action_index = a_index[action_dim_index]
            action[action_name] = action_prop['index_to_value'][action_index]

        return action

    def get_s_a_value(self, s_a_index):
        """This function returns an state and action dictionary with values based on an (s,a) tuple with indexes"""
        # Get states
        s_index = self.s_a_to_s_index(s_a_index)
        state = self.get_state_from_s_index(s_index)

        # Get actions
        a_index = self.s_a_to_a_index(s_a_index)
        action = self.get_action_from_action_index(a_index)

        return state, action

    def get_Q_index(self, action_dim, action_value):
        action_discr = self.action_space[action_dim].index(action_value)
        return action_discr

    def generate_s_a_matrix(self, init=None):
        """Generate a (s,a) matrix, filled with zeros"""

        # The size should be able to contain all states and all action-dimensions
        size = self.data_structure['states_actions_shape']

        # Create a Q matrix filled with zeros
        if type(init) is float or type(init) is int:
            Q = np.full(size,init, dtype=np.float64)
        elif init is None:
            Q = np.zeros(size)
        else:
            raise Exception('Invalid initialization of this matrix')

        return Q


    def generate_policy(self, initialization):
        """Generate a policy with random, or integer initialization."""

        # The policy should contain all states and all action-dimensions
        policy_size = self.data_structure['states_action_shape']

        # Generate empty policy matrix
        policy = np.empty(policy_size)

        # Fill policy matrix
        if initialization == 'random':
            for action_dim_index, (action_name, action_prop) in enumerate(self.actions.items()):
                policy[..., action_dim_index] = np.random.choice(action_prop['values'], policy_size).squeeze()
        elif type(initialization) is list:
            for action_dim_index, (action_name, action_prop) in enumerate(self.actions.items()):
                policy[..., action_dim_index] = initialization[action_dim_index]

        return policy

    def start_episode(self):
        """Function that is executed when the episode is started"""
        return True

    def get_greedy_action_from_Q(self, s_index):
        """This function returns a tuple with the greedy action (ID not value) for a specific state, based on the Q-matrix"""
        # Find maximum Q for this state
        max_Q_value = self.Q[s_index].max()

        # Get list of actions with this Q-value
        max_Q_indices = np.where(max_Q_value == self.Q[s_index])[0]

        # Out of these actions with maximum Q-value pick one randomly
        greedy_action_tuple = tuple([random.choice(max_Q_indices)])

        return greedy_action_tuple, max_Q_value

    def get_all_actions_with_Q(self, s_index):
        """This function returns of a list of all actions (ID not value) and their Q value for this specific state"""
        # The list contains the following tuples (action_id, Q_value)
        list = []
        for Q_index, Q_value in np.ndenumerate(self.Q[s_index]):
            list.append((Q_index, Q_value))

        return list

    def process_step(self, old_state_rl, action, reward, new_state_rl):
        """Function that is executed after each step, should be overwritten by child class"""

        # Get state index
        s_a_index = self.get_s_a_index(old_state_rl, action)
        s_index = self.s_a_to_s_index(s_a_index)

        # Count (s,a) visit
        self.count_s_a_visit(s_a_index)

        return True

    def count_s_a_visit(self, s_a_index):
        """Count this visit"""
        self.s_a_visits[s_a_index] += 1

        return True

    def end_episode(self):
        """Function that is executed when the episode is started, should be overwritten by child class"""
        return True

    def plot_Q_for_state(self, state):
        state_index = self.get_s_index(state)
        fig, ax = self.plot_Q_for_state_index(state_index)

        return fig, ax

    def plot_Q_for_state_index(self, state_index):
        """Create a plot that shows the Q-value for all actions in a specific state"""
        # Create figure
        number_of_actions = self.number_of_action_variables
        fig, ax = plt.subplots(number_of_actions, squeeze=False)

        # Loop through each of the actions and create a subplot
        for i, (action_name, action_prop) in enumerate(self.actions.items()):
            ax[i, 0].plot(action_prop['values'], self.get_Q()[state_index])
        fig.show()

        return fig, ax

    def get_state_visits(self, filter=None):
        """Get a matrix cointaining the state visits (summed over all actions)"""
        # Get the (s,a) visits matrix and sum over all actions
        action_axis = self.get_action_axis()
        states_visited = np.sum(self.s_a_visits,axis=action_axis)

        if filter is not None:
            for key, value in filter.items():
                state_variable_axis = self.states[key]['axis']
                state_variable_index = self.states[key]['value_to_index'][value]
                states_visited = np.take(states_visited, state_variable_index, axis=state_variable_axis)

            state_variables = [variable for variable in self.state_space_variables if variable not in filter]
        else:
            state_variables = self.state_space_variables

        return states_visited, state_variables

    def get_policy(self):
        """Should be implemented by the child class"""

        return False