import numpy as np
import random

from Classes.ReinforcementLearningAgent import ReinforcementLearningAgent


class TemporalDifferenceAgent(ReinforcementLearningAgent):
    """Reinforcement Learning agent using Temporal Difference method for training"""

    def __init__(self, epsilon, gamma, alpha, algorithm, elig_lambda, state_space, action_space, Q_init=None, exploration='epsilon_greedy'):

        # Call parent class, which saves the state & action spaces and generates the data structure
        ReinforcementLearningAgent.__init__(self, state_space, action_space)

        # Define learning parameters
        self.gamma = gamma
        self.alpha = alpha
        self.epsilon = epsilon

        # Define algorithm
        self.algorithm = algorithm
        self.exploration = exploration

        # Generate Q matrix
        self.Q = self.generate_s_a_matrix(Q_init)

        if elig_lambda is not None:
            self.elig_lambda = elig_lambda
            # self.eligibility = self.generate_s_a_matrix()
            self.eligibility_traces = []

        # # Create variables that are used to store results from each timestep
        # self.states_visited = []
        # self.rewards = []
        #
        # # Create variables that are used to store results from each episode
        # self.returns = np.array(np.float)

    def choose_actions(self, state):
        """This function picks the next action(s) based on the current state (returns dict)"""
        actions = []

        # Find the index of the current state
        state_index = self.get_s_index(state)

        # Get greedy action
        greedy_action_tuple, _ = self.get_greedy_action_from_Q(state_index)

        # Loop through all available actions
        # for idx, action_type in enumerate(sorted(self.action_space)):
        actions = {}
        for action_dim_index, (action_name, action_prop) in enumerate(self.actions.items()):
            if self.exploration == 'epsilon_greedy':
                # Epsilon soft, so first determine if we go for the greedy action, as determined by the policy
                # Or for a a random action with chance epsilon
                if random.random() < self.epsilon: # Random action
                        action = random.choice(action_prop['values'])
                        actions[action_name] = action
                        # Reset eligibility traces if Q-learning (Watkins)
                        if self.algorithm == 'Q-learning':
                            self.reset_eligibility_traces()

                else: # Action determined by the policy (usually greedy)
                    action = greedy_action_tuple[action_dim_index]
                    actions[action_name] = self.action_space[action_name][action]
            elif self.exploration == 'epsilon_greedy_alt':
                action_prop['values']
            else:
                raise NotImplementedError('Unknown exploration method')

        return actions

    def process_step(self, old_state_rl, action, reward, new_state_rl):
        """Process each step, add state to the list of states visited and save the reward"""
        # print('Epsilon: {0:.2f} Alpha: {1:.2f}'.format(self.epsilon, self.alpha))
        # Get state indices
        old_s_a_index = self.get_s_a_index(old_state_rl, action)
        old_s_index = self.s_a_to_s_index(old_s_a_index)
        new_s_a_index = self.get_s_a_index(new_state_rl, action)
        new_s_index = self.s_a_to_s_index(new_s_a_index)

        # Count (s,a) visit
        self.count_s_a_visit(old_s_a_index)

        # Get Q position
        Q_pos = old_s_a_index

        # if reward > 0:
        #     print(reward)


        # Update eligibility traces
        if self.elig_lambda is not None:
            self.add_to_eligibility_traces(old_s_a_index, update_method='replacing')

        if self.algorithm == 'Q-learning':
            # Q-Learning, an off-policy TD control algorithm

            # Get the Q value of the next greedy action
            greedy_action_tuple, greedy_action_Q = self.get_greedy_action_from_Q(new_s_index)

            # Calculate the delta
            delta = reward + self.gamma * greedy_action_Q - self.Q[Q_pos]

            # No eligibility traces, update only last states Q
            if self.elig_lambda is None or self.elig_lambda == 0.0:
                self.Q[Q_pos] = self.Q[Q_pos] + self.alpha * delta
                # if action['intervene'] == 3:
                #     print('{} + {}*{} = {}'.format(self.Q[Q_pos], self.alpha, delta, self.Q[Q_pos]))

            else:
                for trace in self.eligibility_traces:
                    # Update Q
                    self.Q[trace[0]] = self.Q[trace[0]] + self.alpha * (delta) * trace[1]

            # if self.Q[Q_pos] < -1000:
                # print('')

        elif self.algorithm == 'SARSA':
            # SARSA, an on-policy TD control algorithm

            # Get the next action (on-policy)
            on_policy_action = self.choose_actions(new_state_rl)

            # Get the s_a index of this new state action combination
            on_policy_s_a = self.get_s_a_index(new_state_rl, on_policy_action)

            # Calculate the Q value for this new state & action combination
            on_policy_action_Q = self.Q[on_policy_s_a]

            # Calculate the delta
            delta = reward + self.gamma * on_policy_action_Q - self.Q[Q_pos]

            # No eligibility traces, update only last states Q
            if self.elig_lambda is None or self.elig_lambda == 0.0:
                self.Q[Q_pos] = self.Q[Q_pos] + self.alpha * delta

            else:
                for trace in self.eligibility_traces:
                    # Update Q
                    self.Q[trace[0]] = self.Q[trace[0]] + self.alpha * (delta) * trace[1]
        else:
            raise ValueError('Unknown TD learning algorithm')

        # Update eligibility traces
        self.update_eligibility_traces()

        return delta

    def get_policy(self, exploration_in_policy=None):
        """"Function to generate a policy matrix for a TD agent"""

        # The policy should contain all states and all action-dimensions
        policy_size = self.data_structure['states_action_shape']

        # Generate empty policy matrix
        policy = np.empty(policy_size)

        # Loop through the policy and determine greedy actions
        policy_it = np.nditer(policy, flags=['multi_index'], op_flags=['writeonly'])
        exploratory_state_actions = []
        while not policy_it.finished:
            # Convert policy index to state and action indices
            policy_index = policy_it.multi_index
            s_index, a_index = self.split_s_a_index(policy_index)

            if exploration_in_policy is None:
                # Get the greedy action and convert into a greedy action value
                action_tuple, _ = self.get_greedy_action_from_Q(s_index)

            elif exploration_in_policy == 'epsilon_greedy':

                if random.random() < self.epsilon:  # Random action
                    all_actions = self.get_all_actions_with_Q(s_index)
                    action_tuple = random.choice(all_actions)[0]
                    exploratory_state_actions.append(policy_index)
                else:  # Action determined by the policy (usually greedy)
                    action_tuple, _ = self.get_greedy_action_from_Q(s_index)
            else:
                raise ValueError('Unknown exploration method for the policy:'+exploration_in_policy)

            # Get action value from action tuple
            action = self.get_action_from_action_index(action_tuple)
            action_list = [value for key, value in action.items()]
            action_value = action_list[a_index[0]]

            # Save the action value to the policy
            policy_it[0] = action_value

            # Move on to the next element in the policy matrix
            policy_it.iternext()

        if exploration_in_policy is None:
            return policy
        else:
            return policy, exploratory_state_actions

    def add_to_eligibility_traces(self, s_a_index, update_method='replacing'):
        indices = [trace[0] for trace in self.eligibility_traces]
        if s_a_index in indices:
            pos = indices.index(s_a_index)

            # Increase value of trace
            if update_method == 'accumulating':
                self.eligibility_traces[pos][1] = self.eligibility_traces[pos][1] + 1
            elif update_method == 'dutch':
                self.eligibility_traces[pos][1] = (1-self.alpha) * self.eligibility_traces[pos][1] + 1
            elif update_method == 'replacing':
                self.eligibility_traces[pos][1] = 1
            else:
                raise ValueError("Unknown method for updating the eligibility traces")
        else:
            # Add to trace with value 1
            self.eligibility_traces.append([s_a_index,1])

        return True

    def update_eligibility_traces(self):
        for idx, trace in enumerate(self.eligibility_traces):
            self.eligibility_traces[idx][1] = self.eligibility_traces[idx][1] * self.gamma * self.elig_lambda

        # Remove small values
        self.eligibility_traces = [trace for trace in self.eligibility_traces if trace[1] > 0.0001]

        return True

    def reset_eligibility_traces(self):
        self.eligibility_traces = []
        return True

    def get_Q(self):
        return self.Q

