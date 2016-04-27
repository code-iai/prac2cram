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
import yaml
import rospy

# imports the service
from prac2cram.srv import Prac2Cram
# import the messages
from prac2cram.msg import ActionCore, ActionRole

def prac2cram_client(action_cores):

    # NOTE: you don't have to call rospy.init_node() to make calls against
    # a service. This is because service clients do not have to be
    # nodes.

    # block until the service is available
    # you can optionally specify a timeout
    try:
        rospy.wait_for_service('prac2cram', timeout = 5)
    except rospy.ROSException, e:
        print "Service call timed out! Please start the prac2cram service."
        return


    try:
        # create a handle to the service
        prac2cram = rospy.ServiceProxy('prac2cram', Prac2Cram)

        #h = std_msgs.msg.Header()
        #h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work

        # simplified style
        response = prac2cram(action_cores)

        # formal style
        #resp2 = prac2cram.call(Prac2CramRequest(params))

        return response

    except rospy.ServiceException, e:
        print "Service call failed with the following error: %s" %e

def usage():
    scriptname =  os.path.basename(sys.argv[0])
    return "Usage: %s fulltest or %s simpletest" %(scriptname, scriptname)

if __name__ == "__main__":

    argv = rospy.myargv()


    if len(argv) >= 2:

      if argv[1] == 'fulltest':

        d = yaml.load(test_yaml)
        print d
        print "full test not implemented yet!" #TODO load yaml test file

      elif argv[1] == 'simpletest' or argv[1] == 'test':

        core1 = ActionCore(action_core_name='Neutralizing')
        core1.action_roles = [ActionRole(role_name='obj_to_be_started', role_value='centrifuge.n.01')]
        core2 = ActionCore()
        core2.action_core_name = 'TurningOnElectricalDevice'
        core2.action_roles =  [ActionRole(role_name='device', role_value='centrifuge.n.01')]

        action_cores = [core1, core2]

        result = prac2cram_client(action_cores)

        if result:
            print "\nResult for action cores \n%s: \n%s"%(action_cores, result)

      else: print usage()

    else: print usage()


test_yaml = """action_cores:
          - action_core_name: 'Neutralizing'
            action_roles:
              - role_name: neutralizer
                role_value: hydrofluoric_acid.n.01
              - role_name: neutralizee
                role_value: pyridine.n.01
              - role_name: amount
                role_value: four.n.01
              - role_name: unit
                role_value: drop.n.02
          - action_core_name: 'Adding'
            action_roles:
              - role_name: theme
                role_value: hydrofluoric_acid.n.01
              - role_name: goal
                role_value: pyridine.n.01
              - role_name: amount
                role_value: four.n.01
              - role_name: unit
                role_value: drop.n.02
          - action_core_name: 'Pipetting'
            action_roles:
              - role_name: content
                role_value: hydrofluoric_acid.n.01
              - role_name: goal
                role_value: pyridine.n.01
              - role_name: amount
                role_value: four.n.01
              - role_name: unit
                role_value: drop.n.02"""
