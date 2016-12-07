# Paparazzi center utilities
#
# Copyright (C) 2016 ENAC, Florian BITARD (intern student)
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.

###############################################################################
# [Imports]

import lib.environment as env
import lib.console as cs

import PyQt5.QtCore as Core
import logging
import os
import signal


###############################################################################
# [Constants]

LOGGER = logging.getLogger("[PROCESSES]")

AC_MAKEFILE_NAME = "Makefile.ac"
CONF_FLAG_NAME = "AIRCRAFT"
DEVICE_FLAG_NAME = "FLASH_MODE"

DEFAULT_EXIT_CODE = 2
INTERRUPTED_EXIT_CODE = -1
SUCCESS_EXIT_CODE = 0

CLEAN = "clean"
BUILD = "build"
UPLOAD = "upload"
PROGRAM = "program"
TOOL = "tool"

CLEAN_TARGET_KEY = "clean_ac"
BUILD_TARGET_KEY = ".compile"
UPLOAD_TARGET_KEY = ".upload"

CONF_FLAG = "@" + CONF_FLAG_NAME
TARGET_FLAG = "@TARGET"
CONF_ID_FLAG = "@AC_ID"


###############################################################################
# [Stream class]

class LoggerStream(Core.QObject):
    """ Class to define a Stream object."""
    logger_log_sent = Core.pyqtSignal(str, str, str)

    def __init__(self):
        super(LoggerStream, self).__init__()

    # Reimplemented method in order to write the logs in the console :
    def write(self, line):
        log, flag = cs.analyse_log_line(line)
        log_type = cs.APPLICATION_MESSAGE_TYPE
        self.logger_log_sent.emit(log, flag, log_type)


###############################################################################
# [Process class]

class Process(Core.QObject):
    """Class to upload the built code to a device."""
    process_killed = Core.pyqtSignal()
    process_log_sent = Core.pyqtSignal(str, str, str)

    def __init__(self, process_type,
                 configuration=None, target=None, device=None, program=None):
        """
        :param process_type:
        :param configuration:
        :param target:
        :param device:
        :param program:
        -> Declare a Process object as a QObject derivative.
        -> Give it a name and the necessary parameters for its type.
        -> Generate a command for the system call by the Popen object that
        manages the subprocess and allows to redirect the output.
        -> The process runs into an independent thread.
        -> A queue is used to collect the logs from the Popen output and
        an other one is used to send it to the QTextEdit integrated console.
        -> Logs flags are collected for information.
        -> Exit code is initialized to default value. Must change in case of
        normal exit, error or user interruption.
        """
        super(Process, self).__init__()

        self.type = process_type
        self.name = None

        self.config = configuration
        self.target = target
        self.device = device
        self.program = program

        if self.type == CLEAN:
            self.command = self.generate_make_command(CLEAN_TARGET_KEY)
            self.name = " - ".join([self.type.upper(),
                                    self.config.name])
        elif self.type == BUILD:
            self.command = self.generate_make_command(BUILD_TARGET_KEY)
            self.name = " - ".join([self.type.upper(), self.config.name,
                                    self.target.name])
        elif self.type == UPLOAD:
            self.command = self.generate_make_command(UPLOAD_TARGET_KEY)
            self.name = " - ".join([self.type.upper(), self.config.name,
                                    self.target.name, self.device.name])
        else:
            self.command = self.generate_program_command()
            self.name = " - ".join([self.type.upper(),
                                    self.program.name])

        self.subprocess = Core.QProcess()
        self.subprocess.setProcessChannelMode(Core.QProcess.MergedChannels)
        self.subprocess.setReadChannel(Core.QProcess.StandardOutput)

        self.process_killed.connect(self.emergency_stop)
        self.exit_code = DEFAULT_EXIT_CODE

        self.flags = {cs.ERROR_FLAG:   0,
                      cs.WARNING_FLAG: 0,
                      cs.INFO_FLAG:    0}

    def generate_make_command(self, target_key):
        """
        :param target_key:
        -> Generate a system command to compile files by a Makefile and
        putting the right arguments if given.
        """
        if self.config is not None:
            aircraft_term = CONF_FLAG_NAME + "=" + self.config.name

            if self.target is not None:
                target_key = self.target.name + target_key

            command_terms = ["make", "-C", env.PAPARAZZI_HOME, "-f",
                             AC_MAKEFILE_NAME,
                             aircraft_term, target_key]

            if self.device is not None and self.device.variable[1]:
                device_term = DEVICE_FLAG_NAME + "=" + self.device.variable[1]
                command_terms.insert(-1, device_term)

            return " ".join(command_terms)

    def generate_program_command(self):
        """
        -> Generate a system command to run a program and add its options
        if it has some.
        """
        if self.program is not None:
            full_command = os.path.join(env.PAPARAZZI_HOME,
                                        self.program.command)

            for option in self.program.options:
                if type(option) is tuple:
                    flag, value = option
                    if value == CONF_FLAG:
                        full_command += " " + flag + " " + self.config.name
                    elif value == TARGET_FLAG:
                        full_command += " " + flag + " " + self.target.name
                    elif value == CONF_ID_FLAG:
                        full_command += " " + flag + " " + self.config.id
                    else:
                        full_command += " " + flag + " " + value
                else:
                    full_command += " " + option

            return full_command

    def check_before_start(self):
        # TODO IF NECESSARY !!!
        return self == self

    def start(self):
        """
        -> Start the thread => start the worker => call the run method.
        """
        LOGGER.info("'%s' process running ... (command='%s')",
                    self.name, self.command)
        self.subprocess.start(self.command)
        self.subprocess.readyReadStandardOutput.connect(self.send_text)
        self.subprocess.finished.connect(self.finish_process)

    def send_text(self):
        """
        -> Analyse an process output line to find a flag in it.
        -> Send the item by the sending queue object.
        -> Collect the flag found.
        """
        q_byte_array = self.subprocess.readAllStandardOutput()
        string = str(q_byte_array, encoding="utf-8").strip()
        for line in string.split("\n"):
            log, flag = cs.analyse_log_line(line)
            log_type = cs.PROCESS_MESSAGE_TYPE
            self.process_log_sent.emit(log, flag, log_type)
            if flag != cs.DEFAULT_FLAG:
                self.flags[flag] += 1

    def finish_process(self):
        """
        -> If the process finished, get the exit code.
        -> Else, the process crashed...
        """
        if self.subprocess.exitStatus() == Core.QProcess.NormalExit:
            self.exit_code = self.subprocess.exitCode()
            LOGGER.info("'%s' process finished with exit code %s.\n",
                        self.name, self.exit_code)
        else:
            self.exit_code = INTERRUPTED_EXIT_CODE
            LOGGER.error("'%s' process crashed, probably stopped by user !\n",
                         self.name)

    def emergency_stop(self):
        """
        -> Kill the subprocess by getting its ProcessID.
        """
        os.kill(self.subprocess.pid(), signal.SIGKILL)
