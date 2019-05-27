import numpy as np
import math
import pandas as pd
import random
import seaborn as sns
# import timeit
import bisect
import matplotlib.pyplot as plt
import time
import datetime
import pickle
import copy

class ReinforcementLearningTraining:
    """"Wrapper for reinforcement learning problems"""

    def __init__(self, env_class, agent_class, agent_settings, state_space, action_space, log_level='full',
                 mode='offline'):
        # Set properties provided to the init function
        self.env_class = env_class
        self.agent_class = agent_class
        self.agent_settings = agent_settings
        self.training_history = []
        self.log_level = log_level
        self.state_space_variables = [key for key, value in state_space.items()]
        self.state_space = state_space
        self.action_space_variables = [key for key, value in action_space.items()]
        self.action_space = action_space
        self.state_space_length = {key: len(space) for key,space in state_space.items()}
        self.states_available_to_rl = [name for name in self.state_space]
        self.state_space_bin_bounds = {var_name : (var_space[:-1]+var_space[1:])/2
                                       for var_name, var_space in state_space.items()}
        self.mode = mode

        # Initialize other properties
        self.epsilon_decrease = False
        self.alpha_decrease = False
        self.epsilon_critic_decrease = False
        self.alpha_critic_decrease = False
        self.train_settings = {}
        self.retain_plot_handles = {}
        self.Q_statistics = {}
        self.episodes_in_which_policy_changes = []

        # Variables used during a run
        self.training_history_list = []
        self.episode_result_list = []
        self.old_policy = None
        self.state_visits = None


    def train(self, episode_length, number_of_episodes, save_full_history=False, initial_agent=None, **episode_settings):
        """"Train a Reinforcement Learning for a set environment and return the trained agent

        Args:
            episode_length (int)            :   Number of timesteps in each episode
            number_of_episodes (int)        :   Number of episodes
            save_full_history (bool)        :   Bool determining whether to save the full training history
            initial_agent (obj)             :   If there is already an agent, this gives the ability to continue training
            episode_settings (kwargs)       :   See the run_episodes() function

        Returns:
            agent : The instance of a trained agent
        """

        # Save the training settings
        self.train_settings = {'episode_length':episode_length,
                               'number_of_episodes':number_of_episodes}

        # Check if the agent settings contain an epsilon that should be decreasing
        if 'epsilon' in self.agent_settings and isinstance(self.agent_settings['epsilon'], tuple):
            # Save the epsilon settings tuple (start value, end value, number of episodes)
            self.epsilon_decrease = self.agent_settings['epsilon']

            # Set epsilon to its starting point
            self.agent_settings['epsilon'] = self.epsilon_decrease[0]

        # Check if the agent settings contain an alpha that should be decreasing
        if 'alpha' in self.agent_settings and isinstance(self.agent_settings['alpha'], tuple):
            # Save the epsilon settings tuple (start value, end value, number of episodes)
            self.alpha_decrease = self.agent_settings['alpha']

            # Set epsilon to its starting point
            self.agent_settings['alpha'] = self.alpha_decrease[0]

        # Check if the critic contains an epsilon that should be decreasing
        if 'critic_settings' in self.agent_settings and 'epsilon' in self.agent_settings['critic_settings']:
            if isinstance(self.agent_settings['critic_settings']['epsilon'], tuple):
                # Save the epsilon settings tuple (start value, end value, number of episodes)
                self.epsilon_critic_decrease = self.agent_settings['critic_settings']['epsilon']

                # Set epsilon to its starting point
                self.agent_settings['critic_settings']['epsilon'] = self.epsilon_critic_decrease[0]

        # Check if the critic contains an alpha that should be decreasing
        if 'critic_settings' in self.agent_settings and 'alpha' in self.agent_settings['critic_settings']:
            if isinstance(self.agent_settings['critic_settings']['alpha'], tuple):
                # Save the alpha settings tuple (start value, end value, number of episodes)
                self.alpha_critic_decrease = self.agent_settings['critic_settings']['alpha']

                # Set alpha to its starting point
                self.agent_settings['critic_settings']['alpha'] = self.alpha_critic_decrease[0]

        # Create the agent, based on the provided settings
        if initial_agent is not None:
            agent = initial_agent

            # Set learning parameters for the actor
            if hasattr(agent, 'epsilon') and 'epsilon' in self.agent_settings:
                agent.epsilon = self.agent_settings['epsilon']
            if hasattr(agent, 'alpha') and 'alpha' in self.agent_settings:
                agent.alpha = self.agent_settings['alpha']
            if hasattr(agent, 'elig_lambda') and 'elig_lambda' in self.agent_settings:
                agent.critic.elig_lambda = self.agent_settings['critic_settings']['elig_lambda']

            # Set learning parameters for the critic
            if hasattr(agent.critic, 'epsilon') and 'epsilon' in self.agent_settings['critic_settings']:
                agent.critic.epsilon = self.agent_settings['critic_settings']['epsilon']
            if hasattr(agent.critic, 'alpha') and 'alpha' in self.agent_settings['critic_settings']:
                agent.critic.alpha = self.agent_settings['critic_settings']['alpha']
            if hasattr(agent.critic, 'elig_lambda') and 'elig_lambda' in self.agent_settings['critic_settings']:
                agent.critic.elig_lambda = self.agent_settings['critic_settings']['elig_lambda']
        else:
            agent = self.agent_class(state_space=self.state_space,
                                     action_space=self.action_space,
                                     **self.agent_settings)

        # Set the training state
        agent.in_training = True

        # Run the episodes
        training_history, _ = self.run_episodes(agent=agent,
                          episode_length=episode_length,
                          number_of_episodes=number_of_episodes,
                          save_full_history=save_full_history,
                          **episode_settings)

        if save_full_history:
            self.training_history = training_history

        # Return the trained agent
        return agent

    def run_episodes(self, agent, episode_length, number_of_episodes, save_full_history, env_settings,
                     starting_state_func=False, starting_action_func=False,
                     show_state_transitions=False, progress_updates=False, visualize_episode=False, plot_Q=False,
                     fast_forward=None, show_live_graphs=None):
        """Main loop for running episodes

        Args:
            agent (obj)                     :   Agent that is placed in the environment
            number_of_episodes (int)        :   Number of episodes
            save_full_history (bool)        :   Boolean determining whether to save the full training history
            starting_state_func (func)      :   Function that provides the starting state for each episode
            starting_action_func (func)     :   Function that provides the starting action for each episode
            episode_length (int)            :   Number of timesteps in each episode
            show_state_transitions (bool)   :   If True, the state transition, action and reward of each timestep will
                                                be printed to the console
            progress_updates (bool)         :   If True, after each 100th episode the current progress wil be printed
            visualize_episode (bool)        :   If True, each episode will be visually shown
            plot_Q (bool)                   :   If True the Q function for a state will be plotted every 100 episodes
            fast_forward (func)             :   Function that allows for exploring starts
            show_live_graphs [str]          :   List of which live graphs to show (dashboard, live_episode)

        Returns:
            # agent : The instance of a trained agent
            training_history_list: A list containing the history of all episodes and steps
        """
        # Start run
        self.start_run(agent)

        # Main loop
        for episode in range(0, number_of_episodes):
            # Time the first 10 episodes to provide a time estimate of the full training
            if episode is 0 and number_of_episodes >=100 :
                start = time.time()
            elif episode is 10 and number_of_episodes >=100 :
                end = time.time()
                time_episode = end-start
                expected_total_time = datetime.timedelta(seconds=(time_episode*number_of_episodes/10))
                print("The  10 episodes took {} seconds,"
                      " therefore it is expected that running all {} episodes will take {}"
                      .format(time_episode, number_of_episodes, expected_total_time))

            # Create an empty list to store the episode results
            episode_history = []

            # Create an empty list to store all delta_Q's from one episode
            delta_Q_history = []

            # Store the total episode reward
            episode_reward = 0.0

            # Generate environment
            env = self.env_class(**env_settings)

            # Get starting state
            if starting_state_func is not False:
                old_state_full = starting_state_func()
            else:
                old_state_full = env.get_starting_state()

            # If a fast forward function is defined, forward the environment
            if fast_forward is not None:
                # Get action to perform while forwarding the environment
                action, internal_state = fast_forward(old_state_full)
                state_full = old_state_full

                # Continue untill the function is positive
                while action is not False:
                    # Forward environment
                    reward, state_full = env.step(state_full, action)

                    # Get new action
                    action, _ = fast_forward(state_full, internal_state)

                # Save forwarded state
                old_state_full = state_full

            # Select states available to RL, discretize and convert to list
            old_state_rl = self.to_discrete_available_state(old_state_full)

            agent.start_episode()
            env.start_episode(old_state_full)

            # Counter used for frozen agents
            agent_frozen_for = 0
            agent_frozen_reward = 0

            # Loop over episode steps
            for step in range(0, episode_length):

                # If the agent is not frozen, pick a new action
                if agent_frozen_for <= 0:
                    # Pick an action
                    if step == 0 and starting_action_func is not False:
                        action = starting_action_func()
                    else:
                        # Get action
                        action = agent.choose_actions(old_state_rl)

                    # Check if action is a multiple step-action
                    agent_frozen_for = env.check_number_of_timesteps_actions(old_state_full, action)
                    if agent_frozen_for > 0:
                        state_on_freeze = old_state_rl
                else:
                    # Use previous action, but decrease the number of timesteps the agent is still frozen
                    action = action

                # Take environment 1 step further Take action a, observe r,s
                reward, new_state_full = env.step(old_state_full, action)

                # Decrease the # of timesteps frozen
                agent_frozen_for += -1

                # Select states available to RL and discretize, returns a dictionary
                new_state_rl = self.to_discrete_available_state(new_state_full)

                if agent_frozen_for == 0 or (agent_frozen_for > 0 and env.is_terminal()):
                    # Last timestep in freeze, process previous rewards
                    agent_frozen_reward += reward
                    delta_Q = agent.process_step(state_on_freeze, action, agent_frozen_reward, new_state_rl)
                    agent_frozen_reward = 0
                elif agent_frozen_for > 0:
                    # Agent frozen, save rewards for later processing
                    agent_frozen_reward += reward
                    delta_Q = None
                else:
                    # No freezing, process directly
                    delta_Q = agent.process_step(old_state_rl, action, reward, new_state_rl)

                # Save results of this timestep for later analysis
                episode_history_line = {}
                episode_history_line['episode'] = episode
                if self.log_level=='full':
                    episode_history_line['step'] = step
                    episode_history_line.update(old_state_full)
                    episode_history_line.update(action)
                    episode_history_line['reward'] = reward
                    episode_history_line['delta_Q'] = delta_Q
                else:
                    episode_history_line['step'] = step
                    episode_history_line['reward'] = reward

                # On the first step also save the environment
                if step == 0:
                    episode_history_line['env'] = vars(env)
                else:
                    episode_history_line['env'] = None

                # Save delta Q
                delta_Q_history.append(delta_Q)

                # Save the reward
                episode_reward += reward

                # Save the history of this timestep (dict) to the episode (list)
                episode_history.append(episode_history_line)

                # Print results if show_state_transitions is True
                if show_state_transitions is True:
                    print("Step {}: {} ---[{}]---> {}     {}".format(
                        step, old_state_full['z'], action, new_state_full['z'], reward))

                # Update state
                old_state_full = new_state_full
                old_state_rl = new_state_rl

                # Check if not terminal
                if env.is_terminal():
                    # End episode
                    break

            # Process the episode by the agent and the environment
            agent.end_episode()
            env.end_episode()

            # Print progress update if applicable
            if progress_updates is True and (episode+1) % 100 == 0:
                percentage = (episode+1)/number_of_episodes*100
                print("Finished episode {} of the {}, {} percent complete.".format(
                    episode+1, number_of_episodes, percentage))

            # Visualize episode if applicable
            if visualize_episode is True:
                episode_history_df = pd.DataFrame(episode_history)
                episode_history_df.set_index(['episode', 'step'], inplace=True)
                env.visualize_episode(episode_history_df)

            # Only update epsilon and alpha if in training
            if agent.in_training:
                # Decrease epsilon if applicable
                if self.epsilon_decrease is not False and hasattr(agent, 'epsilon') and episode < self.epsilon_decrease[2]:
                    decrease = (self.epsilon_decrease[0] - self.epsilon_decrease[1]) / self.epsilon_decrease[2]
                    agent.epsilon = agent.epsilon - decrease

                if self.epsilon_critic_decrease is not False and hasattr(agent.critic, 'epsilon'):
                    decrease = (self.epsilon_critic_decrease[0] - self.epsilon_critic_decrease[1]) / self.epsilon_critic_decrease[2]
                    agent.critic.epsilon = agent.critic.epsilon - decrease

                # Decrease alpha if applicable
                if self.alpha_decrease is not False and hasattr(agent, 'alpha'):
                    agent.alpha = self.alpha_decrease[0]* (self.alpha_decrease[1]/ (self.alpha_decrease[1] + episode))

                if self.alpha_critic_decrease is not False and hasattr(agent.critic, 'alpha'):
                    agent.critic.alpha = self.alpha_critic_decrease[0] * (self.alpha_critic_decrease[1] / (self.alpha_critic_decrease[1] + episode))

            # Save the history of this episode (list) to the training (list)
            if save_full_history is True:
                self.training_history_list.extend(episode_history)

            # Get new policy and determine whether it has changed
            new_policy_full = np.squeeze(agent.get_policy())

            states_visited_curr, _ = agent.get_state_visits()
            if self.state_visits is not None:
                # Set policy to 0 for all states that haven't been visited the past episode
                states_visited_diff = states_visited_curr - self.state_visits
                new_policy_compare = np.where(states_visited_diff >= 1, np.squeeze(agent.get_policy()), 0.0)
                old_policy_compare = np.where(states_visited_diff >= 1, self.old_policy, 0.0)
            else:
                new_policy_compare = np.where(states_visited_curr >= 1, np.squeeze(agent.get_policy()), 0.0)
                old_policy_compare = np.where(states_visited_curr >= 1, self.old_policy, 0.0)

            # Q_max = np.amax(agent.get_Q(), axis=tuple(action['axis'] for key, action in agent.actions.items()))
            # new_policy = np.where(Q_max < -1.0, new_policy, 0.0)

            policy_change = not(np.allclose(old_policy_compare, new_policy_compare))


            if(policy_change):
                self.episodes_in_which_policy_changes.append(episode)

            # Save some basic statistics about Q
            self.Q_statistics[episode] = {
                'mean': agent.get_Q().mean(),
                'net_delta': sum(filter_apl(None, delta_Q_history)),
                'abs_delta': sum([abs(Q) for Q in filter_apl(None, delta_Q_history)]),
                'policy_change': policy_change,
                'episode_reward': episode_reward,
            }

            # Save the environment statistics
            if hasattr(env, 'env_statistics'):
                self.episode_result_list.append({**env.env_statistics, **self.Q_statistics[episode]})

            # Update the retaining plots, if applicable
            if show_live_graphs is not None:
                self.update_live_graphs(graphs=show_live_graphs, number_of_episodes=number_of_episodes, current_episode=episode, agent=agent,
                                        episode_history=episode_history, env=env)
                self.flush_live_graphs()

            # New policy -> old policy
            self.old_policy = new_policy_full
            self.state_visits = states_visited_curr

        training_history, episode_results = self.end_run()

        # If no live graphs, show the dashboard at the end
        # if show_live_graphs is None:
        #     self.update_live_graphs(graphs=['dashboard'], number_of_episodes=number_of_episodes, agent=agent)

        return training_history, episode_results

    def start_run(self, agent):
        # Store the training history in a list
        self.training_history_list = []
        self.episode_result_list = []

        new_policy_full = np.squeeze(agent.get_policy())

        self.old_policy = new_policy_full

        return True

    def end_run(self):
        # Convert the complete training history (list) to a pandas dataframe
        if len(self.training_history_list) > 0:
            training_history = pd.DataFrame(self.training_history_list)
            training_history.set_index(['episode', 'step'], inplace=True)

            # Clear training_history_list
            self.training_history_list = []
        else:
            training_history = None

        # Pass the episode_result along without processing
        episode_result = self.episode_result_list

        return training_history, episode_result

    def evaluate_agent(self, agent, episode_length, number_of_episodes, **episode_settings):
        # Set the training state
        agent.in_training = False
        if hasattr(agent.critic, 'in_training'):
            agent.critic.in_training = False

        # Set epsilon to zero if applicable
        if hasattr(agent, 'epsilon'):
            old_value_epsilon = agent.epsilon
            agent.epsilon = 0.0

        if hasattr(agent.critic, 'epsilon'):
            old_value_critic_epsilon = agent.critic.epsilon
            agent.critic.epsilon = 0.0

        # Set alpha to zero if applicable
        if hasattr(agent, 'alpha'):
            old_value_alpha = agent.alpha
            agent.alpha = 0.0

        if hasattr(agent.critic, 'alpha'):
            old_value_critic_alpha = agent.critic.alpha
            agent.critic.alpha = 0.0

        try:
            # Run the episodes
            evaluation_history, episode_results = self.run_episodes(agent=agent,
                                                 episode_length=episode_length,
                                                 number_of_episodes=number_of_episodes,
                                                 save_full_history=False,
                                                 **episode_settings)

            # Get statistics from these evaluation episodes
            evaluation_statistics = self.env_class.calculate_statistics(evaluation_history, episode_results)

        except Exception as e:
            # Pass on exception to the caller
            raise
        finally:
            # Restore settings
            if hasattr(agent, 'epsilon'):
                agent.epsilon = old_value_epsilon

            if hasattr(agent.critic, 'epsilon'):
                agent.critic.epsilon = old_value_critic_epsilon

            if hasattr(agent, 'alpha'):
                agent.alpha = old_value_alpha

            if hasattr(agent.critic, 'alpha'):
                agent.critic.alpha = old_value_critic_alpha

        return evaluation_history, evaluation_statistics

    def to_discrete_available_state(self, state_full):
        """Takes a full state, selects states available to the agent, discretizes and returns a dictionary"""

        state_rl = {
            key: self.state_space[key][
                bisect.bisect(self.state_space_bin_bounds[key], state_full[key]) # Right side not included!
            ]
            for key in self.states_available_to_rl
        }
        return state_rl

    def df_to_discrete(self, df):
        for state in self.states_available_to_rl:
            if state in df.columns:
                df[state+'_discr'] = df[state].apply(lambda x: bisect.bisect(self.state_space_bin_bounds[state], x))

        return df

    def visualize_episode(self, episode_numbers, history=None, variables=None):
        """Call the environment to generate a visual representation of the episode based on the episode history"""

        # If episode_numbers is a single integer, convert to a list
        if isinstance(episode_numbers, int):
            episode_numbers = [episode_numbers]

        # Empty dictionary for storing figures and axes
        list_of_figures = {}

        # If no (evaluation) history is provided, use the training history
        if history is None:
            history = self.training_history

        # Loop over the list of episode numbers
        for episode_number in episode_numbers:
            # Get environment history
            env = self.get_environment(episode_number)
            episode_history = history.xs(episode_number, level='episode')

            # Call environment to provide visual representation
            fig, ax = env.visualize_episode(episode_history, variables)

            # Add a title to show which episode it was
            ax.set_title('Episode: {}'.format(episode_number))
            fig.show()

            # Save to dictionary
            list_of_figures[episode_number] = (fig, ax)

        return list_of_figures

    def get_environment(self, episode_number):
        """Return the environment object used in a certain episode"""

        # Get the environment properties from the history
        episode_history = self.training_history.xs(episode_number,level='episode')
        env_vars = episode_history.loc[0]['env']

        # Create a new environment with these same settings
        env = self.env_class(**env_vars)

        return env

    def plot_rewards(self, running_average=None, remove_zero_rewards=False, fig=None, ax=None, label=None, plotargs={}):
        """Create a plot of the rewards for each of the episodes"""

        # Calculate the rewards
        if hasattr(self, 'episode_result_list'):
            df = pd.DataFrame(self.episode_result_list)
            df['total_reward'] = df['episode_reward']
            df['episode'] = df.index
        else:
            rewards = self.training_history['reward'].groupby(['episode']).sum()
            episodes = self.training_history.index.get_level_values('episode').unique()

            # Convert to pandas dataframe
            df = pd.DataFrame({'episode':episodes,'total_reward':rewards})


        # If remove_zero_rewards is True remove zero rewards (no action and no save)
        if remove_zero_rewards is True:
            df = df[df['total_reward'] < 0]

        # Create plot
        if fig is None or ax is None:
            fig, ax = plt.subplots()

        # Plot rewards
        if running_average is None:
            ax.plot(df['episode'], df['total_reward'])
        else:
            # Plot running average
            ax.plot(df['episode'],
                    df['total_reward'].rolling(window=running_average, min_periods=1,center=False).mean(),
                    label=label,
                    **plotargs
                    )
        # Finalize plot
        plt.ylim(-2000, -10)
        plt.yscale('symlog')
        ax.legend()
        fig.show()

        return fig, ax

    def check_bounds_plot(self):
        """"Generate a plot to check whether the discretization and bounds off the state/action space"""

        # First plot the state bounds
        number_of_states = len(self.state_space)

        # Create figure
        fig_states, ax_states = plt.subplots(number_of_states, squeeze=False)

        # Loop through the states and plot each in a subplot
        for i, (name, bounds) in enumerate(self.state_space.items()):
            # Find number of visits for this state
            historical_state_visits = self.training_history[name]

            # Find internal edges
            state_space_bounds = self.state_space_bin_bounds[name]

            # Add outside edges
            bin_edges = state_space_bounds
            bin_edges = np.insert(bin_edges, 0, bounds[0])
            bin_edges = np.append(bin_edges, bounds[-1])

            if min(historical_state_visits) < bin_edges[0]:
                bin_edges = np.insert(bin_edges, 0, min(historical_state_visits), axis=0)
            if max(historical_state_visits) > bin_edges[-1]:
                bin_edges = np.append(bin_edges,max(historical_state_visits))

            # Plot histogram
            n, bins, patches = ax_states[i,0].hist(x=historical_state_visits, bins=bin_edges)
            ax_states[i, 0].axvline(x=min(bounds), color='r')
            ax_states[i, 0].axvline(x=max(bounds), color='r')


            # Add labels
            ax_states[i,0].set_xlabel(name)
            ax_states[i,0].set_ylabel('Number of visits')

        # Show figure for the state bounds
        fig_states.suptitle('Discretization of the state space bounds')
        fig_states.show()

        # Now do the same thing for the actions
        number_of_actions = len(self.action_space)

        # Create figure
        fig_actions, ax_actions = plt.subplots(number_of_actions, squeeze=False)

        # Loop through the actions and plot each in a subplot
        for i, (name, bounds) in enumerate(self.action_space.items()):
            # Find number of visits for this action type

            historical_action_visits = self.training_history[name]

            # Find & count the unique values in this list of actions
            values, counts = np.unique(historical_action_visits, return_counts=True)

            # Plot histogram
            ax_actions[i, 0].bar(values, counts)
            ax_actions[i, 0].axvline(x=min(bounds), color='r')
            ax_actions[i, 0].axvline(x=max(bounds), color='r')

            # Add labels
            ax_actions[i,0].set_xlabel(name)
            ax_actions[i,0].set_ylabel('Number of visits')

        # Show figure for the action bounds
        fig_actions.suptitle('Discretization of the action space bounds')
        fig_actions.show()

        return fig_states, ax_states, fig_actions, ax_actions

    def visualize_episode_state_visits(self, agent, episode_numbers, flip=False, minimum_visits=None, history=None,
                                       loop_over=None, action=None):
        """Creates a plot showing the state transitions for a single episode"""
        # If episode_numbers is a single integer, convert to a list
        if isinstance(episode_numbers, int):
            episode_numbers = [episode_numbers]

        # If no history is set use the training history
        if history is None:
            history = self.training_history

        figures = []
        for episode_number in episode_numbers:
            for value in self.state_space[loop_over]:
                # Create policy heatmap
                filter_apl = {loop_over: value}
                fig, ax = self.show_policy(agent=agent, flip=flip, minimum_visits=minimum_visits, filter_apl=filter_apl,
                                           action=action)
                figures.append((episode_number,value,fig,ax))

                # get episode history and filter
                state_variables = [variable for variable in self.state_space_variables if variable not in filter_apl]
                episode_history = history.xs(episode_number, level='episode').copy()
                episode_history = episode_history[episode_history[loop_over] == value]

                # Convert state variables to discrete indices
                state_var_one = state_variables[0]
                state_var_two = state_variables[1]
                self.df_to_discrete(episode_history)


                ax.plot(episode_history[state_var_one+'_discr']+0.5, episode_history[state_var_two+'_discr']+0.5)

                # ax.set_title("Final agent policy for {} = {}".format(loop_over, value))

        plt.show()

        return figures


    def visualize_states_visited_loop(self, agent, loop_over):
        for value in self.state_space[loop_over]:
            filter_apl = {loop_over: value}
            fig, ax = self.visualize_states_visited(agent, filter_apl)
            ax.set_title("Number of state visits (summed over all actions) for {}={}.".format(loop_over, value))
        return fig,ax

    def visualize_states_visited(self, agent, filter_apl=None):
        """Create a plot to show whether all states have been visited"""

        # Get the (s,a) visits matrix and sum over all actions
        states_visited, state_variables = agent.get_state_visits(filter_apl)

        # Determine ticks
        ticks = [self.state_space[state_variable] for state_variable in state_variables]
        ticks = tuple([self.format_ticks(ticks_list) for ticks_list in ticks])

        # Determine labels
        labels = tuple(state_variables)

        # Create a heatmap based on the values, ticks and labels previously found
        fig, ax = self.create_heatmap(states_visited, ticks, labels, annot=True)
        ax.set_title("Number of state visits (summed over all actions)")

        plt.show()
        return fig, ax

    def format_ticks(self, ticks):
        """ This function takes an iterable of floats and convert it into a list of strings with a set format """

        ticks_str = []
        # Loop through all thicks
        for i, tick in enumerate(ticks):
            try:
                ticks_str.append('%.2f' % tick)
            except ValueError:
                ticks_str.append(tick)

        return ticks_str

    def show_policies(self, agent, action, loop_over=False, flip=False, minimum_visits=0):
        if loop_over:
            number_of_plots = len(self.state_space['prev_action'])
            cols = max(round(math.sqrt(number_of_plots)), 1)
            rows = max(math.ceil(number_of_plots / cols), 1)
            fig, axarr = plt.subplots(rows, cols, squeeze=False)

            for idx, value in enumerate(self.state_space[loop_over]):
                ax = axarr[idx // cols][idx % cols]
                filter_apl = {loop_over: value}
                _, ax = self.show_policy(agent=agent, flip=flip, minimum_visits=minimum_visits, filter_apl=filter_apl, action=action, ax=ax)
                ax.set_title("Final agent policy for {} = {}".format(loop_over, value))

            fig.tight_layout()
        else:
            fig, ax = self.show_policy(agent=agent, flip=flip, minimum_visits=minimum_visits, filter_apl=None,
                                       action=action)
        return fig, ax

    def show_policy(self, agent,flip=False, minimum_visits=0, filter_apl=None, action=None, ax=None, cbar=False):
        # Get state variables
        state_variables = self.state_space_variables

        # Determine which action we should look at
        if action is None:
            action = self.action_space_variables[0]

        # Get policy
        if hasattr(agent, 'policy'):
            policy = agent.policy[:]
        elif isinstance(agent, np.ndarray):
            policy = agent
        else:
            policy = agent.get_policy()

        # Filter policy
        if filter_apl is not None:
            for key, value in filter_apl.items():
                state_variable_id = self.state_space_variables.index(key)
                state_index = agent.states[key]['value_to_index'][value]
                policy = np.take(policy, state_index, axis=state_variable_id)

            state_variables = [variable for variable in self.state_space_variables if variable not in filter_apl]

        # If dimensions of policy are greater then 2, try to squeeze it
        for l in policy.shape:
            if l == 1:
                policy = np.squeeze(policy)

        # Set states to None for which the minimum state visits has not been reached (no real policy determined here)
        minimum_visits = float(minimum_visits)
        try:
            states_visited, _ = agent.get_state_visits(filter_apl)
            policy = np.where(states_visited >= minimum_visits, policy, None)
        except AttributeError:
            print('Could not determine state visits')
            pass

        # If flip is true transpose the policy matrix and reverse the state variables
        if flip is True:
            print('flipping')
            policy = policy.T
            state_variables = state_variables[::-1]

        # Determine ticks & labels
        ticks = [self.state_space[state_variable] for state_variable in state_variables]
        ticks = tuple([self.format_ticks(ticks_list) for ticks_list in ticks])
        labels = tuple(state_variables)

        # Get vmin and vmax (exclude action 99)
        potential_actions = self.action_space[action]
        # potential_actions[potential_actions < 99]
        vmin = min(potential_actions[potential_actions < 99])
        vmax = max(potential_actions[potential_actions < 99])

        # Create a heatmap based on the values, ticks and labels previously found
        if ax is None:
            fig, ax = self.create_heatmap(policy, ticks, labels, annot=True, vmin=vmin,vmax=vmax, cbar=cbar)
            ax.set_title("Agent policy")
            plt.show()
        else:
            fig, ax = self.create_heatmap(policy, ticks, labels, annot=True, vmin=vmin, vmax=vmax, ax=ax, cbar=cbar)
            ax.set_title("Agent policy")

        return fig, ax

    def request_new_live_grapsh(self):
        self.retain_plot_handles = {}

        return True

    def update_live_graphs(self, graphs, retain=True, number_of_episodes=None, current_episode=None, agent=None,
                           episode_history=None, env=None):
        # If the number of episodes is not set, use the current number of episodes stored
        if number_of_episodes is None:
            number_of_episodes = len(self.Q_statistics)
        elif current_episode is not None and number_of_episodes <= current_episode:
            number_of_episodes = 1.2*current_episode

        # Create or retrieve Live dashboard
        if 'dashboard' in graphs:
            if retain is False or 'dashboard' not in self.retain_plot_handles:
                dashboard_handle = self.create_live_dashboard(number_of_episodes, agent)
            else:
                # Retrieve existing plot handles
                dashboard_handle = self.retain_plot_handles['dashboard']

                # See if the figure still exists
                if not plt.fignum_exists(dashboard_handle['fig'].number):
                    dashboard_handle = self.create_live_dashboard(number_of_episodes, agent)

            # Update Live dashboard
            dashboard_handle = self.update_live_dashboard(number_of_episodes, agent, current_episode, **dashboard_handle)
            if retain is True:
                self.retain_plot_handles['dashboard'] = dashboard_handle

        if 'live_episode' in graphs:
            # Create or retrieve Live episode plot
            if episode_history is not None and env is not None:
                if retain is False or 'live_episode' not in self.retain_plot_handles:
                    live_episode_handle = self.create_live_episode_plot(env, episode_history)
                else:
                    live_episode_handle = self.retain_plot_handles['live_episode']

                # Update live episode plot
                live_episode_handle = self.update_live_episode_plot(env, episode_history, current_episode, **live_episode_handle)
                if retain is True:
                    self.retain_plot_handles['live_episode'] = live_episode_handle

        return True

    def create_live_dashboard(self, number_of_episodes, agent):
        # Create new plots
        plt.ion()
        fig = plt.figure(figsize=(9.6, 9.58))
        axarr = fig.subplots(3, 2)

        # Locate plots
        ax_mean_Q = axarr[0, 0]
        ax_policy = axarr[0, 1]
        ax_delta_Q = axarr[1, 0]
        ax_q_1 = axarr[1, 1]
        ax_q_2 = axarr[2, 1]
        ax_rewards = axarr[2, 0]

        # Create mean Q subfigure
        ax_mean_Q.set_xlim([0, number_of_episodes])
        ax_mean_Q.set_xlabel('Episode')
        ax_mean_Q.set_ylim([-10, 0])
        ax_mean_Q.set_ylabel('Mean Q value')

        mean_Q_line = plt.Line2D([], [], color='black')
        ax_mean_Q.add_line(mean_Q_line)

        # Create delta Q subfigure
        ax_delta_Q.set_xlim([0, number_of_episodes])
        ax_delta_Q.set_xlabel('Episode')
        ax_delta_Q.set_ylim([-100, 1000])
        ax_delta_Q.set_ylabel('Absolute delta Q')

        delta_Q_line = plt.Line2D([], [], color='black')
        ax_delta_Q.add_line(delta_Q_line)

        # Create policy subfigure
        if agent is not None:
            self.show_policy(agent, flip=False, minimum_visits=1, action='intervene', ax=ax_policy, cbar=False)

        # Create average reward subfigure
        ax_rewards.set_xlim([0, number_of_episodes])
        ax_rewards.set_xlabel('Episode')
        ax_rewards.set_ylim([-1000, 50])
        ax_rewards.set_ylabel('Rewards')

        reward_line = plt.Line2D([], [], color='black')
        ax_rewards.add_line(reward_line)

        # Create q_1 subfigure (showing the q-values for prev_action = 1.0)
        ax_q_1.set_title('Q values for different actions when prev_action = no-action')

        # Create q_1 subfigure (showing the q-values for prev_action = 3.0)
        ax_q_2.set_title('Q values for different actions when prev_action = hover')

        # Create a text box showing the result of the last episode
        log_lines = []
        log_lines.append(fig.text(0.02, 0.97, 'Episode XX: Fail at height 0.00m', fontsize=12, color='black'))

        # Loop over episodes to place lines at policy changes:
        for index in range(0, number_of_episodes):
            if index in self.Q_statistics and self.Q_statistics[index]['policy_change']:
                ax_mean_Q.axvline(x=index, color='r')
                ax_delta_Q.axvline(x=index, color='r')

        # Set overall figure properties
        fig.tight_layout()

        # Save variables as handle dictionary
        dashboard_handle = {}
        for var in ['fig', 'axarr', 'mean_Q_line', 'delta_Q_line', 'reward_line', 'log_lines']:
            dashboard_handle[var] = locals()[var]

        return dashboard_handle

    def update_live_dashboard(self, number_of_episodes, agent, current_episode, fig, axarr, mean_Q_line, delta_Q_line, reward_line, log_lines):
        # Locate plots
        ax_mean_Q = axarr[0, 0]
        ax_policy = axarr[0, 1]
        ax_delta_Q = axarr[1, 0]
        ax_q_1 = axarr[1, 1]
        ax_q_2 = axarr[2, 1]
        ax_rewards = axarr[2, 0]

        # Update line
        mean_Q = []
        delta_Q = []
        rewards = []
        list_of_episodes = [a for a in range(0, number_of_episodes)]

        # Calculate mean Q and delta Q where possible
        for index in list_of_episodes:
            # Check if this episode has already occured
            if index in self.Q_statistics:
                mean_Q.append(self.Q_statistics[index]['mean'])
                delta_Q.append(self.Q_statistics[index]['abs_delta'])
                rewards.append(self.Q_statistics[index]['episode_reward'])
            else:
                mean_Q.append(None)
                delta_Q.append(None)
                rewards.append(None)

        # Update mean Q subfigure
        mean_Q_line.set_data(list_of_episodes, mean_Q)

        # Update delta Q subfigure
        delta_Q_line.set_data(list_of_episodes, delta_Q)

        # Update rewards subfigure
        reward_line.set_data(list_of_episodes, rewards)

        # Update policy subfigure
        if agent is not None and current_episode is not None and self.Q_statistics[current_episode]['policy_change']:
            self.show_policy(agent, flip=False, minimum_visits=1, action='intervene', ax=ax_policy, cbar=False)

        # Check if axis need to be updated for any of the plots
        filtered_mean_Q = [x for x in mean_Q if x is not None]
        if ax_mean_Q.get_ylim()[0] > 1.1 * min(filtered_mean_Q):
            ax_mean_Q.set_ylim([1.1 * min(filtered_mean_Q), ax_mean_Q.get_ylim()[1]])

        filtered_delta_Q = [x for x in delta_Q if x is not None]
        if ax_delta_Q.get_ylim()[1] < 1.1 * max(filtered_delta_Q):
            ax_delta_Q.set_ylim([ax_delta_Q.get_ylim()[0], 1.1 * max(filtered_delta_Q)])

        filtered_rewards = [x for x in rewards if x is not None]
        if ax_rewards.get_ylim()[0] > 1.1 * min(filtered_rewards):
            ax_rewards.set_ylim([1.1 * min(filtered_rewards), ax_rewards.get_ylim()[1]])

        # Plot vertical line for episode changes
        for ax in [ax_mean_Q, ax_delta_Q, ax_rewards]:
            if current_episode is not None and self.Q_statistics[current_episode]['policy_change']:
                ax.axvline(x=current_episode, color='r', zorder=0)

        # Create first subfigure showing the q value for different actions at prev_action = 1
        if agent is not None:
            label_dict = {1.0: 'no-action', 2.0: 'save', 3.0: 'hover'}
            state_filter = {
                'prev_action': 1.0
            }
            self.show_Q_as_bar_graph(agent, label_dict=label_dict, filter_apl=state_filter, ax=ax_q_1)

        # Create second subfigure showing the q value for different actions at prev_action = 3.0
        if agent is not None:
            label_dict = {1.0: 'no-action', 2.0: 'save', 3.0: 'hover'}
            state_filter = {
                'prev_action': 3.0
            }
            self.show_Q_as_bar_graph(agent, label_dict=label_dict, filter_apl=state_filter, ax=ax_q_2)

        # Update log lines
        log_last_episode = log_lines[0]
        env_record = self.episode_result_list[-1]
        status = env_record['result']
        if status == 'Crash':
            height = env_record['crash_height']
        elif status == 'Timeout':
            height = 0.0
        else:
            height = env_record['save_height']
        log_last_episode_text = 'Episode {0}: {1} at height {2:.2f}m'.format(current_episode, status, height)
        log_last_episode.set_text(log_last_episode_text)

        # Update legends
        ax_mean_Q.legend(['Mean Q value', 'Policy change'])
        ax_delta_Q.legend(['Absolute delta Q', 'Policy change'])
        ax_rewards.legend(['Episode reward', 'Policy change'])

        # Redraw figure
        fig.canvas.draw()

        # Save variables as handle dictionary
        dashboard_handle = {}
        for var in ['fig', 'axarr', 'mean_Q_line', 'delta_Q_line', 'reward_line', 'log_lines']:
            dashboard_handle[var] = locals()[var]

        return dashboard_handle

    def create_live_episode_plot(self, env, episode_history):
        # Creat figure and axis
        plt.ion()
        fig = plt.figure(figsize=(4, 4))

        handle = self.update_live_episode_plot(env, episode_history, 0, fig)

        return handle

    def update_live_episode_plot(self, env, episode_history, current_episode, fig):
        # Display episode history
        episode_history_df = pd.DataFrame(episode_history)
        episode_history_df.set_index(['episode', 'step'], inplace=True)
        _, ax_live_episode = env.visualize_episode(episode_history_df, fig_passed=fig)

        # Set title
        ax_live_episode.set_title('Episode: {0}'.format(current_episode))

        # Redraw figure
        fig.canvas.draw()

        # Save variables as handle dictionary
        handle = {}
        for var in ['fig']:
            handle[var] = locals()[var]

        return handle

    def flush_live_graphs(self):
        if len(self.retain_plot_handles) > 0:
            for handle in self.retain_plot_handles:
                try:
                    self.retain_plot_handles[handle]['fig'].canvas.flush_events()
                except:
                    pass

    def show_Q_convergence(self, history=None):
        # Load history
        if history is None:
            history = self.training_history

        # Only timesteps at which Q is set (no frozen steps)
        Q_df = history['delta_Q'].replace(to_replace='None', value=np.nan).dropna()

        # Plot Q delta
        Q_through_episodes = abs(Q_df).groupby('episode').sum()
        ax = Q_through_episodes.plot(logy=True)
        plt.ylabel('Summed delta Q per episode')
        plt.title('Convergence of Q')
        plt.show()

        return ax

    def show_Q_for_1D_state_space(self, agent, label_dict=None, filter_apl=None):
        """Create a lineplot comparing the Q values for different actions at different states"""
        # Get the Q-matrix
        Q = agent.Q

        # Create plot
        fig, ax = plt.subplots()

        # Get states
        state_variables = [variable for variable in self.state_space_variables]

        # Loop over action dimensions
        for i, (name, options) in enumerate(self.action_space.items()):
            # Loop over actions
            for k, value in enumerate(self.action_space[name]):
                if label_dict is not None and value in label_dict:
                    label = label_dict[value]
                else:
                    label = value

                ax.plot(Q[:, k], label=label)

        ticks = [self.state_space[state_variable] for state_variable in state_variables]
        ticks = [self.format_ticks(ticks_list) for ticks_list in ticks]
        ax.set_xticklabels(ticks[0])
        plt.legend()
        plt.show()

        return ax

    def show_Q_as_bar_graph(self, agent, label_dict=None, filter_apl=None, ax=None):
        """Create a bar plot comparing the Q values for different actions at different states"""
        # Get the Q-matrix
        Q = agent.get_Q()

        if ax is None:
            fig, ax = plt.subplots()
        else:
            title = ax.get_title()
            ax.clear()

        # Get states and actions
        state_variables = [variable for variable in self.state_space_variables]

        # Assuming one action
        action_dimension = self.action_space_variables[0]
        action_axis_Q = agent.actions[action_dimension]['axis']

        # filter_apl filter
        if filter_apl is not None:
            for key, value in filter_apl.items():
                state_variable_id = self.state_space_variables.index(key)
                state_index = agent.states[key]['value_to_index'][value]
                Q = np.take(Q, state_index, axis=state_variable_id)
                action_axis_Q += -1 # Reduce by one, because we just removed a state dimension
                state_variables.remove(key) # Remove from list of state variables

        # Loop over actions (assuming 1 action dimension)
        width = (1-0.2)/len(self.action_space[action_dimension])
        for index, action_value in enumerate(self.action_space[action_dimension]):
            action_row_Q = agent.actions[action_dimension]['value_to_index'][action_value]
            Q_temp = np.take(Q, action_row_Q, action_axis_Q)

            # Prepare plot
            x_positions = [a + index * width for a in range(0, len(Q_temp))]
            ticks = self.format_ticks(self.state_space[state_variables[0]])
            ax.bar(x_positions, Q_temp, tick_label=ticks, label=label_dict[action_value], width=width)

        # Finalize plot
        ax.set_xlabel(state_variables[0])
        ax.set_ylabel('Q value')
        ax.set_title(title)
        ax.legend()

        return ax

    def show_Q_heatmap(self, agent):
        """Create a plot to show the Q-value for different states and action combinations"""

        # Get the Q-matrix
        Q = agent.Q

        # Determine ticks
        state_variables = [variable for variable in self.state_space_variables]
        ticks = [self.state_space[state_variable] for state_variable in state_variables]
        ticks = tuple([self.format_ticks(ticks_list) for ticks_list in ticks])

        # Determine labels
        labels = tuple(state_variables)

        # Loop over action dimensions
        for i, (name, bounds) in enumerate(self.action_space.items()):
            # Loop over actions
            for k, value in enumerate(self.action_space[name]):

                # Create a heatmap based on the values, ticks and labels previously found
                fig, ax = self.create_heatmap(Q[...,k], ticks, labels, annot=True)
                ax.set_title("Q-value for different states and action: "+str(value))

                plt.show()

        return fig, ax

    @staticmethod
    def create_heatmap(data_to_plot, ticks, labels, cmap='RdYlGn', annot=False, vmin=None, vmax=None, ax=None, cbar=True):
        """"Static function to create an visual heatmap, the data argument should be a np.ndarray"""

        # Copy data to prevent overriding
        data = data_to_plot[:]

        # Calculate size of heatmap in # of blocks
        length_x = data.shape[0]
        original_data_dimension = len(data.shape)

        # Check if data is 1D, 2D or 2D+
        if original_data_dimension > 2: # Data is 3D(+)
            # Take the average of the policy?
            raise ValueError('Data is larger then 2D, not sure how to make it a heatmap')
            length_y = data.shape[1]

            ticks_x = ticks[0]
            ticks_y = ticks[1]
        elif original_data_dimension == 2: # Data is 2D
            length_y = data.shape[1]

            ticks_x = ticks[0]
            ticks_y = ticks[1]
        else: # Data is 1D
            length_y = 1

            # Set the x and y ticks
            ticks_x = ticks[0]
            ticks_y = ['']
            labels = (labels[0], '')

            # Change 1D data structure to a 2D for plotting
            data = np.expand_dims(data, axis=1)

        # Create heatmap
        if ax is None:
            fig, ax = plt.subplots()
        else:
            fig = False
            ax.clear()
        ax = sns.heatmap(data.T.astype(float),
                         mask=pd.isnull(data.T),
                         annot=annot,
                         vmin=vmin,
                         vmax=vmax,
                         cmap=cmap,
                         xticklabels=ticks_x,
                         yticklabels=ticks_y,
                         linewidths=.5,
                         ax=ax,
                         cbar=cbar)
        # im = ax.imshow(X=data.T,
        #                cmap=cmap)

        # Add ticks & labels to the x axis
        # ax.set_xticks(np.arange(length_x))
        # ax.set_xticklabels(ticks_x)
        ax.set_xlabel(labels[0])

        # Add ticks & labels to the y axis
        # ax.set_yticks(np.arange(length_y))
        # ax.set_yticklabels(ticks_y)
        ax.set_ylabel(labels[1])

        # Rotate the tick labels and set their alignment.
        # plt.setp(ax.get_xticklabels(), rotation=45, ha="right",
        #          rotation_mode="anchor")

        # Loop over data dimensions and create text annotations.
        # for i in range(length_x):
        #     for j in range(length_y):
        #         text = ax.text(i, j, data[i, j],
        #                        ha="center", va="center", color="w")

        # fig.tight_layout()

        return fig, ax

    @staticmethod
    def determine_agent_name(agent_settings, number):
        # Get properties
        actor_eps_str = str(agent_settings['epsilon'])
        if 'critic' in agent_settings:
            algorithm_str = str(agent_settings['critic_settings']['algorithm'])
            critic_eps_str = str(agent_settings['critic_settings']['epsilon'])
            alpha_str = str(agent_settings['critic_settings']['alpha'])
            elig_lambda_str = str(agent_settings['critic_settings']['elig_lambda'])
        else:
            algorithm_str = str(agent_settings['algorithm'])
            critic_eps_str = ''
            alpha_str = str(agent_settings['alpha'])
            elig_lambda_str = str(agent_settings['elig_lambda'])

        agent_name = '{algorithm}{elig_lambda}_actoreps{actor_eps}_criticeps{critic_eps}_alpha{alpha}_{number}'.format(
            algorithm=algorithm_str, elig_lambda=elig_lambda_str, actor_eps=actor_eps_str,
            critic_eps=critic_eps_str, alpha=alpha_str, number=number
        )

        return agent_name

    @staticmethod
    def train_and_evaluate_one_setting(training, env_settings, agent_settings, training_settings, evaluation_settings,
                                       number, process_number, save_folder_path, rnd_seed, initial_agent=None):
        result_dict = copy.deepcopy(agent_settings)

        # Determine the agent name (before epsilon and alpha are adjusted)
        agent_name = ReinforcementLearningTraining.determine_agent_name(agent_settings, number)

        # Set truly random seed for evaluation
        random.seed(rnd_seed)
        np.random.seed(rnd_seed+8888)

        if training_settings['number_of_episodes'] > 0:
            # Train the agent
            trained_agent = training.train(
                episode_length=training_settings['episode_length'],
                number_of_episodes=training_settings['number_of_episodes'],
                save_full_history=False,
                env_settings=env_settings,
                starting_state_func=False,
                starting_action_func=False,
                show_state_transitions=False,
                progress_updates=True,
                visualize_episode=False,
                fast_forward=training_settings['fast_forward'] if 'fast_forward' in training_settings else None,
                initial_agent=initial_agent,
                # show_live_graphs = ['dashboard', 'live_episode'],
            )
        else:
            trained_agent = initial_agent

        # Save the agent
        file_agent = agent_name+"_agent.pickle"
        file_agent_path = save_folder_path+file_agent
        file_training = agent_name+"_training.pickle"
        file_trainig_path = save_folder_path + file_training

        pickle.dump(trained_agent, open(file_agent_path, "wb"))
        pickle.dump(training, open(file_trainig_path, "wb"))

        result_dict['trained_agent'] = file_agent
        result_dict['training'] = file_training

        # Set random seed for evaluation
        # random.seed(713)
        # np.random.seed(713)

        # Evaluate the agent
        evaluation_history, evaluation_statistics = training.evaluate_agent(
            agent=trained_agent,
            episode_length=evaluation_settings['episode_length'],
            number_of_episodes=evaluation_settings['number_of_episodes'],
            env_settings=env_settings,
            starting_state_func=False,
            starting_action_func=False,
            show_state_transitions=False,
            progress_updates=False,
            visualize_episode=False,
            fast_forward=evaluation_settings['fast_forward'] if 'fast_forward' in evaluation_settings else None,
        )

        # Print to show finished
        print('Finished process #{0}, {1}% crashed, for an average reward of {2:.2f}. [{3}]'.format(
                    process_number,
                    evaluation_statistics['percentage_crashed'],
                    evaluation_statistics['average_total_reward'],
                    agent_name
                )
        )

        # Summarize and save result
        result_dict.update(evaluation_statistics)

        return result_dict
