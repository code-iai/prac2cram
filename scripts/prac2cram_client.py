#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$


import sys
import os

import rospy
import std_msgs.msg

# imports the service
from prac2cram.srv import *
# import the messages
from prac2cram.msg import *

def prac2cram_client(action_cores):

    # NOTE: you don't have to call rospy.init_node() to make calls against
    # a service. This is because service clients do not have to be
    # nodes.

    # block until the service is available
    # you can optionally specify a timeout
    rospy.wait_for_service('prac2cram')

    try:
        # create a handle to the service
        prac2cram = rospy.ServiceProxy('prac2cram', Prac2Cram)

        #h = std_msgs.msg.Header()
        #h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work

        # simplified style
        resp1 = prac2cram(action_cores)

        # formal style
        #resp2 = prac2cram.call(Prac2CramRequest(params))

        return resp1

    except rospy.ServiceException, e:
        print "Service call failed with the following error: %s" %e

def usage():
    scriptname =  os.path.basename(sys.argv[0])
    return "Usage: %s fulltest or %s simpletest" %(scriptname, scriptname)

if __name__ == "__main__":

    argv = rospy.myargv()

    if len(argv) >= 2:

      if argv[1] == 'fulltest':

        print "full test not implemented yet!" #TODO load yaml test file

      elif argv[1] == 'simpletest' or argv[1] == 'test':

        core1 = ActionCore()
        core1.action_core_name = 'Starting'
        # TODO catch null values
        core1.action_roles = [ActionRole(role_name='obj_to_be_started', role_value='centrifuge.n.01')]

        core2 = ActionCore()
        core2.action_core_name = 'TurningOnElectricalDevice'
        core2.action_roles =  [ActionRole(role_name='device', role_value='centrifuge.n.01')]

        action_cores = [core1, core2]

        print "\nResult for action cores \n%s: \n%s"%(action_cores, prac2cram_client(action_cores))

      else: print usage()

    else: print usage()
