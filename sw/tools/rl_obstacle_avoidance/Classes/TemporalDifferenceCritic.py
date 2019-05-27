from Classes.TemporalDifferenceAgent import TemporalDifferenceAgent

class TemporalDifferenceCritic(TemporalDifferenceAgent):
    """Reinforcement Learning critic using Temporal Difference method for training"""

    def __init__(self, gamma, alpha, algorithm, elig_lambda, state_space, action_space, Q_init=None,
                 epsilon=0.0, exploration_in_policy=None):

        super().__init__(epsilon, gamma, alpha, algorithm, elig_lambda, state_space, action_space, Q_init)

        self.critic = True
        self.exploration_in_policy = exploration_in_policy
        self.policy = self.update_policy()
        self.exploratory_state_actions = []

    def choose_actions(self, state):
        raise NotImplementedError('This is a critic, an actor or agent should choose the actions, not the critic')

    def process_step(self, old_state_rl, action, reward, new_state_rl):
        # Check if the action that was taken was an exploratory one
        old_s_a_index = self.get_s_a_index(old_state_rl, action)
        if old_s_a_index in self.exploratory_state_actions:
            # If so, reset the eligibility traces
            self.reset_eligibility_traces()

        # Update Q matrix & state-visited list
        return super().process_step(old_state_rl, action, reward, new_state_rl)

    def update_policy(self):
        if self.exploration_in_policy is None:
            self.exploratory_state_actions = []
            self.policy = super().get_policy()
            return self.policy
        else:
            self.policy, self.exploratory_state_actions = super().get_policy(self.exploration_in_policy)
            return self.policy

    def get_policy(self, exploration_in_policy=None):
        return self.policy

