#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import time
import signal
import subprocess

import gevent
import gevent.wsgi
import gevent.queue
from tinyrpc.protocols.jsonrpc import JSONRPCProtocol
from tinyrpc.transports.wsgi import WsgiServerTransport
from tinyrpc.server.gevent import RPCServerGreenlets
from tinyrpc.dispatch import RPCDispatcher

dispatcher = RPCDispatcher()
transport = WsgiServerTransport(queue_class=gevent.queue.Queue)

portOffsNum = int(sys.argv[1])
packageName = str(sys.argv[2])


print 'Starting subprocess of portIdx ' + str(portOffsNum) + ' and type ' + str(packageName)

# start wsgi server as a background-greenlet
wsgi_server = gevent.wsgi.WSGIServer(('0.0.0.0', 5050 + portOffsNum), transport.handle)
gevent.spawn(wsgi_server.serve_forever)

rpc_server = RPCServerGreenlets(
    transport,
    JSONRPCProtocol(),
    dispatcher
)


#Setup port parameters
gazeboPort = 11345 + portOffsNum
rosPort = 11311 + portOffsNum
rosBridgePort = 9090 + portOffsNum
#prac2cramPort = 5050 + portOffsNum
os.environ["GAZEBO_MASTER_URI"] = "http://localhost:" + str(gazeboPort)
os.environ["ROS_MASTER_URI"] = "http://localhost:" + str(rosPort)
os.environ["ROSBRIDGE_WEBPORT"] = str(rosBridgePort)

#Run roscore
print "Running roscore ..."
roscoreProc = subprocess.Popen('roscore -p ' + str(rosPort), stdout=None, shell=True, stderr=None, preexec_fn=os.setsid)
time.sleep(5)
print "                ... should be running."
#prac2cramProc = subprocess.Popen('rosrun prac2cram prac2cram_RPC_Server.py ' + prac2cramPort, stdout=subprocess.PIPE, shell=True, stderr=subprocess.PIPE, preexec_fn=os.setsid))

#Setup rosparam for ROSBridge port
setParProc = subprocess.call('rosparam set /port ' + str(rosBridgePort), shell=True)

#Roslaunch gazebo instance and associated nodes
print "Starting gazebo ..."
gazeboProc = subprocess.Popen('roslaunch ' + packageName + ' start_gazebo.launch', stdout=None, shell=True, stderr=None, preexec_fn=os.setsid)
time.sleep(20)
print "                ... should be started."

#def silly():
#    for stdout_line in iter(gazeboProc.stdout.readline, ""):
#        yield stdout_line

#gazeboOutGI = silly()

#for l in silly():
#    print l

#while True:
#    line = gazeboProc.stdout.readline()
#    if line:
#        print line


#Roslaunch CRAM
print "Starting CRAM ..."
cramProc = subprocess.Popen('rosrun ' + packageName + ' start_cram.sh', stdout=None, shell=True, stderr=None, preexec_fn=os.setsid)
time.sleep(20)
print "              ... should be started."

#Rosrun mongodb logger (but do this only when needed, ie. right before a sim begins)
#mongoProc = subprocess.Popen('rosrun mongodb_log mongodb_log /tf /logged_designators /logged_metadata --mongodb-name roslog_' + str(portOffsNum), stdout=subprocess.PIPE, shell=True, stderr=subprocess.PIPE, preexec_fn=os.setsid)

#Terminate mongodb logger (after a sim ends)
#os.killpg(os.getpgid(mongoProc), signal.SIGTERM)

print "Setting INT/TERM sig handle ..."

def exit_gracefully(signum, frame):
    os.killpg(os.getpgid(cramProc.pid), signal.SIGTERM)
    os.killpg(os.getpgid(gazeboProc.pid), signal.SIGTERM)
    os.killpg(os.getpgid(roscoreProc.pid), signal.SIGTERM)
    sys.exit(0)

signal.signal(signal.SIGINT, exit_gracefully)
signal.signal(signal.SIGTERM, exit_gracefully)


#@dispatcher.public
#def <RPC server code>:

rpc_server.serve_forever()

