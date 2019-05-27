import random

from Classes.ReinforcementLearningAgent import ReinforcementLearningAgent


class RLActor(ReinforcementLearningAgent):
    def __init__(self, state_space, action_space, exploration, epsilon, critic, critic_settings):
        """Agent that can be used in an actor-critic setup as the actor"""

        # Call parent class, which saves the state & action spaces and generates the data structure
        super().__init__(state_space, action_space)

        self.exploration = exploration

        # Create critic
        self.critic = critic(state_space=state_space, action_space=action_space, **critic_settings)

        # Get initial policy
        self.policy = None
        self.update_policy()

        # Set exploration rate
        self.epsilon = epsilon

    def choose_actions(self, state):
        """This function picks the next action(s) based on the current state (returns dict) and policy"""
        actions = []

        # Find the index of the current state
        state_index = self.get_s_index(state)

        # Get policy action
        policy_action_tuple = self.get_policy_action(state_index)

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
                    # Reset eligibility traces of critic if Q-learning (Watkins)
                    if self.critic.algorithm == 'Q-learning':
                        self.critic.reset_eligibility_traces()

                else: # Action determined by the policy (usually greedy)
                    action = policy_action_tuple[action_dim_index]
                    actions[action_name] = action
            else:
                raise NotImplementedError('Unknown exploration method')

        return actions

    def get_policy_action(self, state_index):
        """Get the action according to the current policy"""
        action = self.policy[state_index]
        return action

    def update_policy(self):
        """Update the actor policy based on the latest info from the critic"""
        self.policy = self.critic.update_policy()
        return

    def get_state_visits(self, filter_apl=None):
        return self.critic.get_state_visits(filter_apl)

    def get_policy(self, exploration_in_policy=None):
        """Function that gets the current policy from the actor"""
        return self.policy

    def process_step(self, old_state_rl, action, reward, new_state_rl):
        """Pass to the critic"""
        delta_Q = self.critic.process_step(old_state_rl, action, reward, new_state_rl)
        return delta_Q

    def end_episode(self):
        """Function called at the end of every episode, use it to update the policy"""
        self.update_policy()
        return

    def get_Q(self):
        return self.critic.get_Q()