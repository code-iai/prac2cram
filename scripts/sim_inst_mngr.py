#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import time
import signal
import subprocess

import rospy
import statecodes
from prac2cram.msg import CRAMTick

import gevent
import gevent.wsgi
import gevent.queue
from tinyrpc.protocols.jsonrpc import JSONRPCProtocol
from tinyrpc.transports.http import HttpPostClientTransport
from tinyrpc import RPCClient

portOffsNum = int(sys.argv[1])
packageName = str(sys.argv[2])

parentPort = None
if (3 < len(sys.argv)):
    parentPort = sys.argv[3]

print 'Starting subprocess of portIdx ' + str(portOffsNum) + ' and type ' + str(packageName)

#Setup port parameters
gazeboPort = 11345 + portOffsNum
rosPort = 11311 + portOffsNum
rosBridgePort = 9090 + portOffsNum
rpcPort = 5050 + portOffsNum

os.environ["GAZEBO_MASTER_URI"] = "http://localhost:" + str(gazeboPort)
os.environ["ROS_MASTER_URI"] = "http://localhost:" + str(rosPort)
os.environ["ROSBRIDGE_WEBPORT"] = str(rosBridgePort)

simURL = "http://localhost:" + str(rpcPort)
simClient = None
simRPC = None

parentURL = None
parentClient = None
parentRPC = None
if (parentPort != None):
    parentURL = "http://localhost:" + parentPort
    parentClient = RPCClient(JSONRPCProtocol(), HttpPostClientTransport(parentURL))
    parentRPC = parentClient.get_proxy()

roscoreProc = None
gazeboProc = None
setParProc = None
rpcProc = None
cramProc = None

cState = statecodes.SC_BOOTING
nState = statecodes.SC_BOOTING

CRAMWatchdogTicked = False
CRAMWatchdogErrTick = False
CRAMWatchdogDoneTick = False

def shutdownChildren():
    global rpcProc, simRPC, simClient, cramProc, gazeboProc, roscoreProc
    rospy.signal_shutdown("Shutting down simulation process tree.")
    if (rpcProc != None):
        os.killpg(os.getpgid(rpcProc.pid), signal.SIGTERM)
        rpcProc = None
        simRPC = None
        simClient = None
    if (cramProc != None):
        os.killpg(os.getpgid(cramProc.pid), signal.SIGTERM)
        cramProc = None
    if (gazeboProc != None):
        os.killpg(os.getpgid(gazeboProc.pid), signal.SIGTERM)
        gazeboProc = None
    if (roscoreProc != None):
        os.killpg(os.getpgid(roscoreProc.pid), signal.SIGTERM)
        roscoreProc = None

print "Setting INT/TERM sig handle ..."

def exit_gracefully(signum, frame):
    global cState, nState
    cState = statecodes.SC_EXIT
    nState = statecodes.SC_EXIT
    shutdownChildren()
    sys.exit(0)

signal.signal(signal.SIGINT, exit_gracefully)
signal.signal(signal.SIGTERM, exit_gracefully)

def CRAMTickCallback(cramTick):
    global CRAMWatchdogTicked, CRAMWatchdogErrTick, CRAMWatchdogDoneTick
    CRAMWatchdogTicked = True
    if (0 != cramTick.error):
        CRAMWatchdogErrTick = True
    if (0 != cramTick.done):
        CRAMWatchdogDoneTick = True    

def notifyParentOfState(state):
    global parentRPC, portOffsNum
    if (None != parentRPC):
        parentRPC.notify_state({"childId": portOffsNum, "state": state})

def onIdle():
    global CRAMWatchdogTicked, CRAMWatchdogErrTick, CRAMWatchdogDoneTick
    global cState
    #Setup watchdog to track ticks from CRAM
    CRAMWatchdogTicked = False
    CRAMWatchdogErrTick = False
    CRAMWatchdogDoneTick = False
    #Update state machine
    cState = statecodes.SC_IDLE
    #Tell parent (if any) that this instance is now ready to get commands
    notifyParentOfState(cState)
    
def onError():
    global cState, nState
    cState = statecodes.SC_ERROR
    #Tell parent (if any) that this instance is currently unavailable
    notifyParentOfState(cState)
    #Need to restart everything
    shutdownChildren()
    #Gazebo takes a while to actually exit
    time.sleep(15)
    #On next watchdog loop, will (re)boot child processes
    nState = statecodes.SC_BOOTING

def onBoot():
    global cState, nState
    global roscoreProc, rosPort, setParProc, rosBridgePort, rpcProc, portOffsNum, rpcPort, parentURL
    global simClient, simURL, simRPC, gazeboProc, packageName, cramProc
    cState = statecodes.SC_BOOTING
    #Run roscore
    print "Running roscore ..."
    roscoreProc = subprocess.Popen('roscore -p ' + str(rosPort), stdout=None, shell=True, stderr=None, preexec_fn=os.setsid)
    time.sleep(5)
    print "                ... should be running."
    #prac2cramProc = subprocess.Popen('rosrun prac2cram prac2cram_RPC_Server.py ' + prac2cramPort, stdout=subprocess.PIPE, shell=True, stderr=subprocess.PIPE, preexec_fn=os.setsid))
    
    #Setup rosparam for ROSBridge port
    setParProc = subprocess.call('rosparam set /port ' + str(rosBridgePort), shell=True)
    
    #Run sim_rpc
    comstring = 'python ./sim_rpc.py ' + str(portOffsNum) + ' ' + str(rpcPort)
    if (None != parentURL):
        comstring = comstring + ' ' + parentURL
    rpcProc = subprocess.Popen(comstring, stdout=None, shell=True, stderr=None, preexec_fn=os.setsid)
    time.sleep(1)
    simClient = RPCClient(JSONRPCProtocol(), HttpPostClientTransport(simURL))
    simRPC = simClient.get_proxy()
    #Setup the listener to ticks from CRAM
    rospy.init_node('sim_inst_mng')
    rospy.Subscriber("cramticks", CRAMTick, CRAMTickCallback)

    #Roslaunch gazebo instance and associated nodes
    print "Starting gazebo ..."
    gazeboProc = subprocess.Popen('roslaunch ' + packageName + ' start_gazebo.launch', stdout=None, shell=True, stderr=None, preexec_fn=os.setsid)
    time.sleep(20)
    print "                ... should be started."

    #Roslaunch CRAM
    print "Starting CRAM ..."
    cramProc = subprocess.Popen('rosrun ' + packageName + ' start_cram.sh', stdout=None, shell=True, stderr=None, preexec_fn=os.setsid)
    time.sleep(30)
    print "              ... should be started."
    #Next watchdog loop will setup the idle state
    nState = statecodes.SC_IDLE

def watchdogLoop():
    global cState, nState, CRAMWatchdogTicked, CRAMWatchdogDoneTick, CRAMWatchdogErrTick
    while statecodes.SC_EXIT != nState:
        if (statecodes.SC_BOOTING == nState):
            onBoot()
        elif (statecodes.SC_IDLE == nState) and (statecodes.SC_IDLE != cState):
            onIdle()
        elif (statecodes.SC_ERROR == nState):
            onError()
        elif (statecodes.SC_IDLE == cState) or (statecodes.SC_BUSY == cState):
            time.sleep(8)
            if (False == CRAMWatchdogTicked) or (True == CRAMWatchdogErrTick):
                #Next watchdog loop will trigger error handling
                nState = statecodes.SC_ERROR
            #BUSY state now handled by sim_rpc
            #elif (True == CRAMWatchdogDoneTick) and (statecodes.SC_BUSY == cState):
            #    #Next watchdog loop will transition to idle
            #    nState = statecodes.SC_IDLE
            CRAMWatchdogDoneTick = False
            CRAMWatchdogErrTick = False
            CRAMWatchdogTicked = False

#def rpcLoop(rpc_server):
#    rpc_server.serve_forever()

#thread = Thread(target = rpcLoop, args = (rpc_server,))
#thread.setDaemon(True)
#thread.start()

watchdogLoop()

