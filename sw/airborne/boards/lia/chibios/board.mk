#
#    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio
#
#    modified by: AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
#    Utah State University, http://aggieair.usu.edu/
#
#    Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
#    Calvin Coopmans (c.r.coopmans@ieee.org)
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.
#
# Required include directories
BOARDINC = $(CHIBIOS_SPECIFIC_DIR)

# List of all the board related files.
BOARDSRC = ${BOARDINC}/board.c
