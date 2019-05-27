#!/usr/bin/env python
import sys
import os
import numpy as np
import pickle
import collections
import math
import copy
import datetime

from Classes.TemporalDifferenceCritic import TemporalDifferenceCritic
from Classes.ReinforcementLearningToolbox import ReinforcementLearningTraining as Training
from Classes.GroundEnvironment import GroundEnvironment
from Classes.RLActor import RLActor
from Classes.QuadrotorConnection import QuadrotorConnection

# Global variables
critic = None
critic_history = []

# Learning variables
critic_alpha_decrease = False
critic_epsilon_decrease = False
agent_epsilon_decrease = False
agent_epsilon = 0.0
total_episode_counter = 0
evaluation_only = False

def prepare_training(number_of_episodes=50):

    # Set critic settings
    critic_settings = {
        'gamma': 1.0,
        'alpha': (0.5, round(number_of_episodes/2)),  # (0.3, 50)
        'algorithm': 'Q-learning',
        'elig_lambda': 0.1,
        'exploration_in_policy': 'epsilon_greedy',
        'epsilon': (0.5, 0.0, round(number_of_episodes/2)),  # (0.5, 0.05, 50)
    }


    agent_settings = {
        # 'exploration': 'epsilon_greedy',
        'epsilon': 0.01,  # Chance that a random action is picked instead of the (greedy) policy
        'critic': TemporalDifferenceCritic,
        'critic_settings': critic_settings
    }


    # Definition of the state space
    state_space = collections.OrderedDict()
    state_space['F_ext'] = np.linspace(-1.0, -0.2, 9) # Bebop 1
    # state_space['F_ext'] = np.linspace(-1.3, -0.5, 9) # Bebop 2
    state_space['prev_action'] = np.array([1., 2., 3.])

    # Definition of the action space
    action_space = collections.OrderedDict()
    action_space['intervene'] = np.array([1., 2., 3.])

    training = Training(
        env_class=GroundEnvironment,
        agent_class=RLActor,
        agent_settings=agent_settings,
        state_space=state_space,
        action_space=action_space,
        mode='online_critic_only',
    )
    return training

min_save_height = 0.25
ground_termination_distance = 0.05
critic_folder = 'OnlineLearning/'
update_live_graph = False

def prepare_critic(agent_settings):
    global critic, critic_alpha_decrease, critic_epsilon_decrease

    # Set training flag
    critic.in_training = True

    # Set critic properties

    critic_settings = {
        'exploration_in_policy': 'epsilon_greedy',
    }

    # Set the discount rate gamma
    if 'critic_settings' in agent_settings and 'gamma' in agent_settings['critic_settings']:
        critic.gamma = agent_settings['critic_settings']['gamma']


    # Set the learning rate alpha
    if 'critic_settings' in agent_settings and 'alpha' in agent_settings['critic_settings']:
        # Allow for decreasing alpha in the critic
        if isinstance(agent_settings['critic_settings']['alpha'], tuple):
            # Save the alpha settings tuple (start value, end value, number of episodes)
            critic_alpha_decrease = agent_settings['critic_settings']['alpha']

            # Set alpha to its starting point
            critic.alpha = critic_alpha_decrease[0]
        else:
            critic.alpha = agent_settings['critic_settings']['alpha']

    # Set the algorithm
    if 'critic_settings' in agent_settings and 'algorithm' in agent_settings['critic_settings']:
        critic.algorithm = agent_settings['critic_settings']['algorithm']

    # Set the eligibility traces factor lambda
    if 'critic_settings' in agent_settings and 'elig_lambda' in agent_settings['critic_settings']:
        critic.elig_lambda = agent_settings['critic_settings']['elig_lambda']

    # Set the exploration within the critic policy
    if 'critic_settings' in agent_settings and 'exploration_in_policy' in agent_settings['critic_settings']:
        critic.exploration_in_policy = agent_settings['critic_settings']['exploration_in_policy']

    # Check if the critic settings contain an epsilon that should be decreasing
    if 'critic_settings' in agent_settings and 'epsilon' in agent_settings['critic_settings']:
        if isinstance(agent_settings['critic_settings']['epsilon'], tuple):
            # Save the alpha settings tuple (start value, end value, number of episodes)
            critic_epsilon_decrease = agent_settings['critic_settings']['epsilon']

            # Set alpha to its starting point
            critic.epsilon = critic_epsilon_decrease[0]
        else:
            critic.epsilon = agent_settings['critic_settings']['epsilon']

    return True


def prepare_agent(agent_settings):
    global agent_epsilon, agent_epsilon_decrease

    # Check if the agent settings contain an epsilon that should be decreasing
    if 'epsilon' in agent_settings:
        if isinstance(agent_settings['epsilon'], tuple):
            # Save the epsilon settings tuple (start value, end value, number of episodes)
            agent_epsilon_decrease = agent_settings['epsilon']

            # Set epsilon to its starting point
            agent_epsilon = agent_epsilon_decrease[0]
        else:
            agent_epsilon = agent_settings['epsilon']


def update_learning_parameters():
    global critic, critic_alpha_decrease, critic_epsilon_decrease, agent_epsilon, agent_epsilon_decrease, total_episode_counter
    str_tmp = ''
    # Decrease agent epsilon if applicable
    if agent_epsilon_decrease is not False and 0 < total_episode_counter <= agent_epsilon_decrease[2]:
        decrease = (agent_epsilon_decrease[0] - agent_epsilon_decrease[1]) / agent_epsilon_decrease[2]
        agent_epsilon = max(agent_epsilon - decrease, 0.0)
        str_tmp += 'Agent epsilon: {0}, '.format(agent_epsilon)
    # Decrease critic epsilon if applicable:
    if critic_epsilon_decrease is not False and 0 < total_episode_counter <= critic_epsilon_decrease[2]:
        decrease = (critic_epsilon_decrease[0] - critic_epsilon_decrease[1]) / critic_epsilon_decrease[2]
        critic.epsilon = max(critic.epsilon - decrease, 0.0)
        str_tmp += 'Critic epsilon: {0}, '.format(critic.epsilon)
    # Decrease critic alpha if applicable
    if critic_alpha_decrease is not False:
        critic.alpha = max(critic_alpha_decrease[0] * (critic_alpha_decrease[1] / (critic_alpha_decrease[1] + total_episode_counter)),0.0)
        str_tmp += 'Critic alpha: {0}, '.format(critic.alpha)

    print(str_tmp)
    return True


def update_policy(log_filename, episode):
    global critic, total_episode_counter, critic_history, quadrotor, update_live_graph, evaluation_only
    print("Requested policy update: filename: {0}, episode: {1}, Total # processed episodes: {2}".format(
        log_filename, episode, total_episode_counter))

    # Retrieve episode history
    if episode > 0 and (log_filename, episode) not in critic_history:
        # Up the total episode counter by one
        total_episode_counter += 1
        critic_history.append((log_filename, episode))

        # Get episode history
        episode_df = quadrotor.get_episode_history_df(log_filename, episode)

        # Process episode history
        episode_df, episode_result = get_rewards(episode_df)

        # Create some storage variables
        delta_Q_history = []
        episode_reward = 0.0

        # Let the critic know we started the episodes
        critic.start_episode()

        # Loop over each of the relevant timesteps
        print('Starting processing')
        for index in range(0, episode_df.shape[0]):
            row = episode_df.iloc[index]

            # print(index)s
            old_state = {
                'F_ext': critic.states['F_ext']['index_to_value'][row['Discr_state_F_ext']],
                'prev_action': critic.states['prev_action']['index_to_value'][row['Discr_state_prev_action']],
            }
            action = {
                'intervene': row['Action chosen']
            }
            reward = row['reward']

            # Check if it is a non-greedy action
            if not math.isclose(row['Action chosen'], row['Policy action']):
                print('Agent performed a non-policy (exploratory) action, reset eligibility traces')
                if critic.algorithm == 'Q-learning':
                    critic.reset_eligibility_traces()

            if index < episode_df.shape[0]-1:
                next_row = episode_df.iloc[index + 1]
                new_state = {
                    'F_ext': critic.states['F_ext']['index_to_value'][next_row['Discr_state_F_ext']],
                    'prev_action': critic.states['prev_action']['index_to_value'][next_row['Discr_state_prev_action']],
                }
                if not evaluation_only:
                    delta_Q = critic.process_step(old_state, action, reward, new_state)
                else:
                    delta_Q = 0.0
            else:
                # Final step
                new_state = {
                    'F_ext': old_state['F_ext'],
                    'prev_action': row['Action chosen'],
                }
                if not evaluation_only:
                    delta_Q = critic.process_step(old_state, action, reward, new_state)
                else:
                    delta_Q = 0.0

            # Save delta Q
            delta_Q_history.append(delta_Q)

            # Save the reward
            episode_reward += reward

        critic.end_episode()

        # Update learning rate and exploration rate
        update_learning_parameters()

        # Update critic policy
        if not evaluation_only:
            critic.update_policy()

        # Get new policy (for uploading)
        new_policy_full = np.squeeze(critic.get_policy().copy())

        # See if policy has changed
        states_visited_curr, _ = critic.get_state_visits()
        if training.state_visits is not None:
            # Set policy to 0 for all states that haven't been visited the past episode
            states_visited_diff = states_visited_curr - training.state_visits
            new_policy_compare = np.where(states_visited_diff >= 1, np.squeeze(critic.get_policy()), 0.0)
            old_policy_compare = np.where(states_visited_diff >= 1, training.old_policy, 0.0)
        else:
            new_policy_compare = np.where(states_visited_curr >= 1, np.squeeze(critic.get_policy()), 0.0)
            old_policy_compare = np.where(states_visited_curr >= 1, training.old_policy, 0.0)

        policy_change = not (np.allclose(old_policy_compare, new_policy_compare))

        if (policy_change):
            training.episodes_in_which_policy_changes.append(total_episode_counter)

        # Save some basic statistics about Q
        training.Q_statistics[total_episode_counter] = {
            'mean': critic.get_Q().mean(),
            'net_delta': sum(filter(None, delta_Q_history)),
            'abs_delta': sum([abs(Q) for Q in filter(None, delta_Q_history)]),
            'policy_change': policy_change,
            'episode_reward': episode_reward,
            'critic_after_episode': copy.deepcopy(critic),
        }

        # Add Q statistics to episode result and save
        episode_result.update(training.Q_statistics[total_episode_counter])
        training.episode_result_list.append(episode_result)

        print('All episode steps processed, total delta Q: {0}'.format(sum(delta_Q_history)))
    else:

        # Get new policy (for uploading)
        new_policy_full = np.squeeze(critic.get_policy().copy())

        # See if policy has changed
        states_visited_curr, _ = critic.get_state_visits()
        if training.state_visits is not None:
            # Set policy to 0 for all states that haven't been visited the past episode
            states_visited_diff = states_visited_curr - training.state_visits
            new_policy_compare = np.where(states_visited_diff >= 1, np.squeeze(critic.get_policy()), 0.0)
            old_policy_compare = np.where(states_visited_diff >= 1, training.old_policy, 0.0)
        else:
            new_policy_compare = np.where(states_visited_curr >= 1, np.squeeze(critic.get_policy()), 0.0)
            old_policy_compare = np.where(states_visited_curr >= 1, training.old_policy, 0.0)

        policy_change = not (np.allclose(old_policy_compare, new_policy_compare))

    # new_policy = np.ones((9, 2))*1
    # training.show_policies(critic, flip=False, minimum_visits=0, action='intervene')
    # training.show_Q_for_1D_state_space(agent=critic,
    #                                    label_dict={1.0: 'no-action', 2.0: 'save', 3.0: 'hover'})

    # Push policy to the quadrotor
    quadrotor.push_new_policy(new_policy_full)
    quadrotor.update_exploration_rate(agent_epsilon)

    # New policy -> old policy
    training.old_policy = new_policy_full
    training.state_visits = states_visited_curr

    # Update live dashboard
    update_live_graph = total_episode_counter

    # Save critic and training
    save_critic()
    save_training()

    return True


def get_rewards(episode_df):
    global min_save_height, ground_termination_distance
    episode_df['reward'] = 0
    failed = False
    save_initiated = False
    hover_initiated = False
    keep_rows = []

    episode_result = {
        'crashed': False,
        'crash_height': False,
        'false_save': False,
        'saved': False,
        'save_height': False,
        'save_timestep': None,
        'result': 'Timeout'
    }

    for index, row in episode_df.iterrows():
        reward = 0

        # Check for unnecessary save
        if (row['Action performed'] == 2) and not save_initiated and not failed:
            hover_initiated = False
            save_initiated = True
            episode_result['saved'] = True
            episode_result['save_height'] = row['ENU position z']
            episode_result['save_timestep'] = index
            keep_rows.append(index)
            if row['ENU position z'] > min_save_height:
                episode_result['false_save'] = True
                episode_df.loc[index, 'reward'] += -500
                episode_result['result'] = 'False Save'
            else:
                # Reward for correct save depends on height above the ground (0 is perfect, 1 is worst)
                # Positive rewards
                dist_factor = (episode_result['save_height'] - ground_termination_distance) / (
                            min_save_height - ground_termination_distance)
                reward = dist_factor * 50

                # Negative rewards
                # dist_factor = (min_save_height - episode_result['save_height']) / (
                #             min_save_height - ground_termination_distance)
                # reward = dist_factor * -50

                episode_df.loc[index, 'reward'] += reward
                episode_result['result'] = 'Correct Save'

        # Check for hovering
        if (row['Action performed'] == 3) and not hover_initiated and not failed:
            keep_rows.append(index)
            hover_initiated = True
            if row['ENU position z'] > min_save_height:
                episode_df.loc[index, 'reward'] += -100
            else:
                episode_df.loc[index, 'reward'] += -25

            episode_df.loc[index, 'reward'] += -reward

        # Check for crash
        if (row['Action performed'] == 4) and not failed:
            failed = True
            episode_result['saved'] = False
            episode_result['crashed'] = True
            episode_result['crash_height'] = row['ENU position z']
            episode_result['result'] = 'Crash'
            keep_rows.append(index)
            episode_df.loc[index, 'reward'] += -2000

        # Also keep no-actions
        if row['Action performed'] == 1 and not save_initiated and not failed:
            hover_initiated = False
            keep_rows.append(index)

    episode_df = episode_df.loc[keep_rows]

    return episode_df, episode_result


def save_critic():
    global critic_folder, save_critic_path
    pickle.dump(critic, open(save_critic_path, "wb"))
    print('Saved critic')
    return True


def save_training():
    global critic_folder, save_training_path
    pickle.dump(training, open(save_training_path, "wb"))
    print('Saved training')
    return True


if __name__ == '__main__':
    # Simulation or real flight?
    if len(sys.argv) > 1:
        selected_mode = sys.argv[1].lower()
        print("Selected mode: {0}".format(selected_mode))
    else:
        print('Simulation (s) or real flight (f)?')
        selected_mode = input()
        if selected_mode not in ['s', 'f']:
            raise ValueError

    # Number of episodes
    print('How many episodes?')
    input_number_of_episodes = input()
    if input_number_of_episodes.isdigit():
        number_of_episodes = int(input_number_of_episodes)
    else:
        raise ValueError

    # Prepare training environment
    training = prepare_training(number_of_episodes)

    # Training or evaluation?
    print('Training (t) or evaluation (e)?')
    training_evaluation = input()
    if training_evaluation not in ['t', 'e']:
        raise ValueError

    if training_evaluation == 'e':
        # Evaluation, so fully greedy and no learning
        training.agent_settings['epsilon'] = 0.0
        training.agent_settings['critic_settings']['epsilon'] = 0.0
        training.agent_settings['critic_settings']['alpha'] = 0.0
        training.agent_settings['critic_settings']['elig_lambda'] = 0.0
        evaluation_only = True

    # Set save paths
    save_critic_path = '{folder}critic_{date:%Y-%m-%d %H:%M:%S}.pickle'.format(
        folder=critic_folder, date=datetime.datetime.now())
    save_training_path = '{folder}training_{date:%Y-%m-%d %H:%M:%S}.pickle'.format(
        folder=critic_folder, date=datetime.datetime.now())

    # Pick critic
    if len(sys.argv) > 2:
        critic_file = sys.argv[2]
        critic_path = critic_folder + critic_file
        print("Selected critic file: {0}".format(critic_file))
        try:
            obj = pickle.load(open(critic_path, "rb"))
            if hasattr(obj, 'critic'):
                critic = obj.critic
            else:
                critic = obj
            critic_loaded = True
        except (OSError, IOError) as e:
            print('Could not load that critic')
            raise ValueError

    else:
        while critic is None:
            # Show available files
            list_of_files = [file for file in os.listdir(critic_folder)
                             if os.path.isfile(os.path.join(critic_folder, file))
                             if '.pickle' in file
                             if 'critic' in file]
            list_of_files.sort(key=lambda x: os.stat(os.path.join(critic_folder, x)).st_mtime)
            for idx, val in enumerate(list_of_files):
                print('[{0}] {1}'.format(str(idx), val))

            # Let user choose the critic by ID
            print('Which critic should be used? Enter ID for existing, or n for new:')
            critic_id = input()
            if critic_id == 'n':
                critic = training.agent_settings['critic'](state_space=training.state_space,
                                                           action_space=training.action_space,
                                                  gamma=training.agent_settings['critic_settings']['gamma'],
                                                  alpha=0.0,
                                                  algorithm=training.agent_settings['critic_settings']['algorithm'],
                                                  elig_lambda=training.agent_settings['critic_settings']['elig_lambda'])
            else:
                try:
                    critic_file = list_of_files[int(critic_id)]
                    critic_path = critic_folder + critic_file
                    obj = pickle.load(open(critic_path, "rb"))
                    if hasattr(obj, 'critic') and not isinstance(obj.critic, bool):
                        critic = obj.critic
                    else:
                        critic = obj
                except (OSError, IOError, IndexError, ValueError) as e:
                    print('Could not load that critic, try again')

    try:
        # Create quadrotor connection
        quadrotor = QuadrotorConnection(selected_mode,
                                        show_polic_updates='short')

        # Prepare critic and agent
        prepare_agent(training.agent_settings)
        prepare_critic(training.agent_settings)

        # Start run
        training.start_run(critic)

        # Loop that runs untill ^C is pressed
        while True:
            # Check for requested policy updates
            if quadrotor.policy_update_requested is not None:
                update_policy(**quadrotor.policy_update_requested)
                quadrotor.policy_update_requested = None

            # Update the retaining plots, if applicable
            if update_live_graph is not False and update_live_graph > 0:
                try:
                    training.update_live_graphs(graphs=['dashboard'], number_of_episodes=number_of_episodes,
                                                current_episode=update_live_graph, agent=critic)
                except:
                    try:
                        training.request_new_live_grapsh()
                        training.update_live_graphs(graphs=['dashboard'], number_of_episodes=number_of_episodes,
                                                    current_episode=update_live_graph, agent=critic)
                    except:
                        print('Could not print live graph')

                update_live_graph = False

            training.flush_live_graphs()

    except KeyboardInterrupt:
        # End run
        # training.training_history, env_history = training.end_run()

        # Print score
        # evaluation_statistics = training.env_class.calculate_statistics(training.training_history, env_history)
        # print(evaluation_statistics)

        # Shutdown quadrotor connection
        quadrotor.shutdown()

        # See if the critic needs to be saved
        if len(sys.argv) > 1:
            # Always save when running from the command line
            save_critic_choice = True
        else:
            # Ask the user
            while True:
                print('Shutting down, do you want to save the (trained) critic? [y/n]')
                save_critic_input = input()
                if save_critic_input.lower() == 'y':
                    save_critic_choice = True
                    break
                elif save_critic_input.lower() == 'n':
                    save_critic_choice = False
                    break
                else:
                    print('Invalid input')

        # If need be, save the agent and training
        if save_critic_choice is True:
            save_critic()
            save_training()
        else:
            print('Did not save the critic')

        # Show final live graph
        training.update_live_graphs(graphs=['dashboard'], retain=False, number_of_episodes=None, agent=critic)
        training.flush_live_graphs()
