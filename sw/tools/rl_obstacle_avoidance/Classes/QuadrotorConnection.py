import sys
import os
sys.path.append("../../ext/pprzlink/lib/v2.0/python")
from ivy.std_api import *
import pprzlink.ivy
import pprzlink.message as message
import ftplib
from shutil import copyfile
import numpy as np
import time
import pandas as pd


class QuadrotorConnection:
    """This class provides the connection and functions to the quadrotor"""

    def __init__(self, selected_mode, show_polic_updates='full'):
        # Set properties
        self.selected_mode = selected_mode
        self.policy_update_requested = None
        self.show_polic_updates = show_polic_updates

        # Creation of the ivy interface
        self.ivy = pprzlink.ivy.IvyMessagesInterface(
            agent_name="rl_obstacle_avoidance_critic",  # Ivy agent name
            start_ivy=False,  # Do not start the ivy bus now
            ivy_bus="127.255.255.255:2010")  # address of the ivy bus

        # Starts the ivy interface
        self.ivy.start()
        print('IVY bus started')

        # Subscribe to RL_TRAINING_DOWN messages and sets recv_callback as the callback function.
        self.ivy.subscribe(self.rl_training_down_callback, message.PprzMessage("telemetry", "RL_TRAINING_DOWN"))
        print('Subscribed to message RL_TRAINING_DOWN')

        # Connect to FTP if required
        if self.selected_mode == 'f':
            # Connect to the quadrotor over FTP
            self.ftp = self.connect_to_quadrotor_ftp()

        return

    def rl_training_down_callback(self, ac_id, pprzMsg):
        # Print the message and the sender id
        print("Received message %s from %s" % (pprzMsg, ac_id))
        request = pprzMsg.request.strip('"')

        if request == 'request_new_policy':
            print('Policy update scheduled')

            parameters = pprzMsg.parameters.strip('"').split(" ")
            self.policy_update_requested = {
                'log_filename': parameters[0],
                'episode': int(parameters[1]),
            }
            return True
        else:
            raise ValueError('Unknown request received from quadrotor:'+request)

    def connect_to_quadrotor_ftp(self):
        try:
            ftp = ftplib.FTP("192.168.42.1", "", "")
            ftp.cwd("internal_000/")
            ftp.login()
            print(ftp.retrlines("LIST"))

            return ftp
        except:
            print("Unexpected error:", sys.exc_info()[0])

    def update_exploration_rate(self, rate):
        # rl_message = "critic DL_SETTING 42 9 {0:f}".format(rate) # Bebop2
        rl_message = "critic DL_SETTING 42 66 {0:f}".format(rate) # Bebop1
        ivy_result = self.ivy.send(rl_message)
        print('Updated exploration rate ({}) received by {}'.format(rate, str(ivy_result)))

        return True

    def push_new_policy(self, policy):
        for index, action in np.ndenumerate(policy):
            self.push_new_policy_value(index, action)

        print('Uploaded new policy')

    def push_new_policy_value(self, index, action):
        rl_message = 'critic RL_TRAINING_UP 42 ' + ' '.join(str(v) for v in index) + ' ' + str((int(action))) # Bebop 1

        ivy_result = self.ivy.send(rl_message)
        time.sleep(0.15)
        if self.show_polic_updates == 'full':
            print('Uploaded one policy value:' + rl_message + ', received by ' + str(ivy_result))
        return ivy_result

    def get_episode_history_df(self, log_filename, episode):
        """Function to retrieve episode history file"""

        # Copy file to local folder
        dst = '/tmp/' + log_filename
        if self.selected_mode == 'f':
            # Get from ftp
            print('Getting from FTP')
            try:
                self.ftp.voidcmd("NOOP")
            except:
                self.ftp = self.connect_to_quadrotor_ftp()

            file = open(dst, 'wb+')
            try:
                self.ftp.retrbinary('RETR {0}'.format(log_filename), file.write)
            except ftplib.error_temp as e:
                print('Timeout, trying to reconnect')
                self.ftp = self.connect_to_quadrotor_ftp()
                self.ftp.retrbinary('RETR {0}'.format(log_filename), file.write)
            file.close()
            print('Successfully downloaded file')
        else:
            # Get from local file
            print('Getting from local file')
            src = '/tmp/NPSsimulation/' + log_filename
            copyfile(src, dst)

        full_df = pd.read_csv(dst, index_col=0, header=0)
        episode_df = full_df[(full_df['Episode'] == episode)
                             & (full_df['Flight status'] > 49)].copy()

        print('Episode history loaded, this episode contains {0} steps.'.format(len(episode_df)))
        return episode_df

    def shutdown(self):
        self.ivy.shutdown()