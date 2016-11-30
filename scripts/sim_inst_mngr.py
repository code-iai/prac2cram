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
from tinyrpc import RPCClient

import statecodes
from p2c_rosaux import getROSTasks, getStringList
from prac2cram.msg import CRAMTick

portOffsNum = int(sys.argv[1])
packageName = str(sys.argv[2])

parentPort = None
if (2 < len(sys.argv)):
    parentPort = sys.argv[3]

print 'Starting subprocess of portIdx ' + str(portOffsNum) + ' and type ' + str(packageName)

# start wsgi server as a background-greenlet
dispatcher = RPCDispatcher()
transport = WsgiServerTransport(queue_class=gevent.queue.Queue)

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
os.environ["GAZEBO_MASTER_URI"] = "http://localhost:" + str(gazeboPort)
os.environ["ROS_MASTER_URI"] = "http://localhost:" + str(rosPort)
os.environ["ROSBRIDGE_WEBPORT"] = str(rosBridgePort)

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
cramProc = None
mongoProc = None

cState = statecodes.SC_BOOTING
nState = statecodes.SC_BOOTING

CRAMWatchdogTicked = False
CRAMWatchdogErrTick = False
CRAMWatchdogDoneTick = False

def shutdownChildren():
    rospy.signal_shutdown("Shutting down simulation process tree.")
    if (mongoProc != None):
        os.killpg(os.getpgid(mongoProc.pid), signal.SIGTERM)
        mongoProc = None
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
    cState = statecodes.SC_EXIT
    nState = statecodes.SC_EXIT
    shutdownChildren()
    sys.exit(0)

signal.signal(signal.SIGINT, exit_gracefully)
signal.signal(signal.SIGTERM, exit_gracefully)

def CRAMTickCallback(cramTick):
    CRAMWatchdogTicked = True
    if (0 != cramTick.error)
        CRAMWatchdogErrTick = True
    if (0 != cramTick.done)
        CRAMWatchdogDoneTick = True    

def notifyParentOfState():
    if (None != parentRPC)
        parentRPC.notify_state({"childId": portOffsNum, "state": cState})

def sendMongoLogsToParent():
    #TODO: insert some notification to the parent here that mongo logs are available

def onIdle():
    #If running, terminate mongo logging
    if (None != mongoProc):
        os.killpg(os.getpgid(mongoProc), signal.SIGTERM)
        mongoProc = None
    sendMongoLogsToParent()
    #Setup watchdog to track ticks from CRAM
    CRAMWatchdogTicked = False
    CRAMWatchdogErrTick = False
    CRAMWatchdogDoneTick = False
    #Update state machine
    cState = statecodes.SC_IDLE
    #Tell parent (if any) that this instance is now ready to get commands
    notifyParentOfState()
    
def onError():
    cState = statecodes.SC_ERROR
    #Tell parent (if any) that this instance is currently unavailable
    notifyParentOfState()
    #Need to restart everything
    shutdownChildren()
    #Gazebo takes a while to actually exit
    time.sleep(10)
    #On next watchdog loop, will (re)boot child processes
    nState = statecodes.SC_BOOTING

def onBoot():
    cState = statecodes.SC_BOOTING
    #Run roscore
    print "Running roscore ..."
    roscoreProc = subprocess.Popen('roscore -p ' + str(rosPort), stdout=None, shell=True, stderr=None, preexec_fn=os.setsid)
    time.sleep(5)
    print "                ... should be running."
    #prac2cramProc = subprocess.Popen('rosrun prac2cram prac2cram_RPC_Server.py ' + prac2cramPort, stdout=subprocess.PIPE, shell=True, stderr=subprocess.PIPE, preexec_fn=os.setsid))
    
    #Setup rosparam for ROSBridge port
    setParProc = subprocess.call('rosparam set /port ' + str(rosBridgePort), shell=True)
    
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

@dispatcher.public
def getState():
    return {"childId": portOffsNum, "state": cState}

@dispatcher.public
def start_simulation(tasks_RPC):
    if (statecodes.SC_IDLE != cState) or (statecodes.SC_IDLE != nState):
        return {"childId": portOffsNum, "retcode": statecodes.RC_NOTREADY, "state": cState, "message": "Not ready yet."}

    # Maybe not needed, since the members have the same names, but better safe etc.
    tasks_ROS = getROSTasks(tasks_RPC)
    response = None
    message = ""
    messages = [""]
    planstrings = [""]
    status = -1
    retcode = statecodes.RC_ALLOK

    # NOTE: you don't have to call rospy.init_node() to make calls against
    # a service. This is because service clients do not have to be
    # nodes.

    # block until the service is available
    # you can optionally specify a timeout
    #rospy.wait_for_service('prac2cram', timeout=5) # in seconds

    try:
        # create a handle to the service
        prac2cram = rospy.ServiceProxy('prac2cram', Prac2Cram)

        # simplified style
        response = prac2cram(tasks_ROS)
        # formal style
        #resp2 = prac2cram.call(Prac2CramRequest(params))

    except rospy.ServiceException, e:
        response = None
        status = -1
        message = "Service call failed with the following error: " + str(e)
        retcode = statecodes.RC_ROSSRVFAIL

    if (None != response):
        #Rosrun mongodb logger (but do this only when needed, ie. right before a sim begins)
        mongoProc = subprocess.Popen('rosrun mongodb_log mongodb_log /tf /logged_designators /logged_metadata --mongodb-name roslog_' + str(portOffsNum), stdout=subprocess.PIPE, shell=True, stderr=subprocess.PIPE, preexec_fn=os.setsid)
        message = "Started simulation."
        messages = getStringList(response.messages)
        planstrings = getStringList(response.plan_strings)
        cState = statecodes.SC_BUSY
        nState = statecodes.SC_BUSY
    return {"childId": portOffsNum, "retcode": retcode, "state": cState, "message": message, "messages": messages, "plan_strings": planstrings}

def watchdogLoop():
    while statecodes.SC_EXIT != nState:
        if (statecodes.SC_BOOTING == nState):
            onBoot()
        elif (statecodes.SC_IDLE == nState) and (statecodes.SC_IDLE != cState):
            onIdle()
        elif (statecodes.SC_ERROR == nState):
            onError()
        elif (statecodes.SC_BUSY == cState) or (statecodes.SC_IDLE == cState):
            time.sleep(10)
            if (False == CRAMWatchdogTicked) or (True == CRAMWatchdogErrTick):
                #Next watchdog loop will trigger error handling
                nState = statecodes.SC_ERROR
            elif (True == CRAMWatchdogDoneTick) and (statecodes.SC_BUSY == cState):
                #Next watchdog loop will transition to idle
                nState = statecodes.SC_IDLE
            CRAMWatchdogDoneTick = False
            CRAMWatchdogErrTick = False
            CRAMWatchdogTicked = False

thread = Thread(target = watchdogLoop)
thread.setDaemon(True)
thread.start()

rpc_server.serve_forever()

