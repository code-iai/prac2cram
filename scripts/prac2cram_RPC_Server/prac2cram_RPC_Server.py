#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2016, IAI Bremen
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

import logging
logging.basicConfig(level=logging.DEBUG)

import rospy

# imports the service
from prac2cram.srv import Prac2Cram
# import the messages
from prac2cram.msg import ActionCore, ActionRole

import gevent
import gevent.wsgi
import gevent.queue
from tinyrpc.protocols.jsonrpc import JSONRPCProtocol
from tinyrpc.transports.wsgi import WsgiServerTransport
from tinyrpc.server.gevent import RPCServerGreenlets
from tinyrpc.dispatch import RPCDispatcher

dispatcher = RPCDispatcher()
transport = WsgiServerTransport(queue_class=gevent.queue.Queue)

# start wsgi server as a background-greenlet
wsgi_server = gevent.wsgi.WSGIServer(('127.0.0.1', 5050), transport.handle)
gevent.spawn(wsgi_server.serve_forever)

rpc_server = RPCServerGreenlets(
    transport,
    JSONRPCProtocol(),
    dispatcher
)

def getROSActionCores(action_cores_RPC):
  action_cores_ROS = []
  for action_core_RPC in action_cores_RPC:
    action_core_ROS = ActionCore()
    action_core_ROS.action_core_name = action_core_RPC['action_core_name']
    action_core_ROS.action_roles = []
    for role_RPC in action_core_RPC['action_roles']:
      role_ROS = ActionRole(role_name=role_RPC['role_name'], role_value=role_RPC['role_value'])
      action_core_ROS.action_roles.append(role_ROS)
    action_cores_ROS.append(action_core_ROS)
  return action_cores_ROS

@dispatcher.public
def prac2cram_client(action_cores_RPC):

    # Maybe not needed, since the members have the same names, but better safe etc.
    action_cores = getROSActionCores(action_cores_RPC)

    # NOTE: you don't have to call rospy.init_node() to make calls against
    # a service. This is because service clients do not have to be
    # nodes.

    # block until the service is available
    # you can optionally specify a timeout
    try:
        rospy.wait_for_service('prac2cram', timeout = 3)
    except rospy.ROSException, e:
        message = "Service call timed out! Please start the prac2cram service."
        logging.error(message)
        return {'status': -1, 'message': message}


    try:
        # create a handle to the service
        prac2cram = rospy.ServiceProxy('prac2cram', Prac2Cram)

        # we cdon't need a header for the moment
        #h = std_msgs.msg.Header() 
        #h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work

        response = prac2cram(action_cores)
        logging.info("Response: %s" %response)
        logging.info("Type of Response: %s" %type(response))

        # the ROS Service response is not JSON serializable so transform it into simple dictionary
        json_response = {}
        json_response['status'] = response.status
        json_response['message'] = response.message

        return json_response

    except rospy.ServiceException, e:
        print "Service call failed with the following error: %s" %e

# in the main greenlet, run our rpc_server
rpc_server.serve_forever()

