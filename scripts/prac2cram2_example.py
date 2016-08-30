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
import json

# imports the service
from prac2cram.srv import Prac2Cram2


def usage():
    scriptname =  os.path.basename(sys.argv[0])
    return "Usage: %s fulltest or %s simpletest" %(scriptname, scriptname)

if __name__ == "__main__":

    argv = rospy.myargv()

    prac2cram2 = rospy.ServiceProxy('prac2cram2', Prac2Cram2)

    pipette_NaOH = [{"name": "use-pipette", "roles": {"amount": 5, "content": "NatriumHydroxide", "goal": "UnknownSubstance", "unit": "drops"}}]
    pour_NaOH = [{"name": "use-measuring-cup", "roles": {"amount": 5, "content": "NatriumHydroxide", "goal": "UnknownSubstance", "unit": "ml"}}]
    
    brown_branch_body = [[{"name": "say", "roles": {"message": "the substance contains Iron (Fe3+)"}}]]
    brown_branch_condition = [{"name": "expect-color", "roles": {"color": "brown"}}]
    brown_branch = {"body": brown_branch_body, "condition": brown_branch_condition}

    clear_branch_body = [[{"name": "say", "roles": {"message": "the substance contains Aluminium (Al3+)"}}]]
    clear_branch_condition = [{"name": "expect-color", "roles": {"color": "clear"}}]
    clear_branch = {"body": clear_branch_body, "condition": clear_branch_condition}

    whitecons_branch_body = [[{"name": "say", "roles": {"message": "the substance contains Magnesium (Mg2+)"}}]]
    whitecons_branch_condition = [{"name": "expect-color", "roles": {"color": "white"}}]
    whitecons_branch = {"body": whitecons_branch_body, "condition": whitecons_branch_condition}

    concentrated_NaOH_branch = [{"name": "if", "roles": {"branches": [clear_branch, whitecons_branch]}}]

    white_branch_body = [pour_NaOH, concentrated_NaOH_branch]
    white_branch_condition = [{"name": "expect-color", "roles": {"color": "white"}}]
    white_branch = {"body": white_branch_body, "condition": white_branch_condition}

    dilute_NaOH_branch = [{"name": "if", "roles": {"branches": [brown_branch, white_branch]}}]

    identify_metal_cations = [pipette_NaOH, dilute_NaOH_branch]

    identify_metal_cations_JSON = str(json.dumps(identify_metal_cations))
    identify_metal_cations_ROS = Prac2Cram2()

    print str(json.dumps(identify_metal_cations))

    response = prac2cram2(str(json.dumps(identify_metal_cations)))

    # print response

