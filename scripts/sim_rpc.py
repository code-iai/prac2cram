import os
import sys
import time
import signal
import socket
import subprocess
from threading import Thread

import gevent
import gevent.wsgi
import gevent.queue
from tinyrpc.protocols.jsonrpc import JSONRPCProtocol
from tinyrpc.transports.wsgi import WsgiServerTransport
from tinyrpc.transports.http import HttpPostClientTransport
from tinyrpc.server.gevent import RPCServerGreenlets
from tinyrpc.dispatch import RPCDispatcher
from tinyrpc import RPCClient

import rospy
import statecodes
from prac2cram.msg import CRAMTick
from prac2cram.srv import Prac2Cram
from p2c_rosaux import getROSTasks, getStringList

portOffsNum = int(sys.argv[1])
rpcPort = int(sys.argv[2])
instPort = int(sys.argv[3])

instURL = "http://localhost:" + str(instPort)
instClient = RPCClient(JSONRPCProtocol(), HttpPostClientTransport(instURL))
instRPC = instClient.get_proxy()

parentURL = None
parentClient = None
parentRPC = None
if (4 < len(sys.argv)):
    parentURL = sys.argv[4]
    parentClient = RPCClient(JSONRPCProtocol(), HttpPostClientTransport(parentURL))
    parentRPC = parentClient.get_proxy()

ownId = portOffsNum
if (5 < len(sys.argv)):
    ownId = sys.argv[5]

simRunning = False

SIMLifeTime = 10*60
nLTThread = None
def MonitorLifetime():
    global simRunning, SIMLifeTime
    while True:
        time.sleep(1)
        if False == simRunning:
            SIMLifeTime = SIMLifeTime - 1
        else:
            SIMLifeTime = 10*60
        if 0 >= SIMLifeTime:
            break
    SIMLifeTime = 10*60
    print "Spent 1h in IDLE state. Will now reboot."
    instRPC.requestReboot(ownId)

nLTThread = Thread(target = MonitorLifetime)
nLTThread.setDaemon(True)
nLTThread.start()



mongoProc = None

doneTicks = 0

# start wsgi server as a background-greenlet
dispatcher = RPCDispatcher()
transport = WsgiServerTransport(queue_class=gevent.queue.Queue)

wsgi_server = gevent.wsgi.WSGIServer(('0.0.0.0', rpcPort), transport.handle)
gevent.spawn(wsgi_server.serve_forever)

rpc_server = RPCServerGreenlets(
    transport,
    JSONRPCProtocol(),
    dispatcher
)

def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    return s.getsockname()[0]
ROSBridgeAddress = "ws://" + str(get_ip_address()) + ":" + str(9090 + portOffsNum)

def notifyParentOfState(state, message):
    global parentRPC, ownId
    if (None != parentRPC):
        parentRPC.notify_state({"childId": ownId, "state": state, "message": message})

def sendMongoLogsToParent():
    #TODO: insert some notification to the parent here that mongo logs are available
    return None

def stopMongo():
    global mongoProc
    if(None != mongoProc):
        os.killpg(os.getpgid(mongoProc.pid), signal.SIGTERM)
        mongoProc = None

def onDone():
    global simRunning
    #If running, terminate mongo logging
    stopMongo()
    simRunning = False
    sendMongoLogsToParent()
    notifyParentOfState(statecodes.SC_IDLE, "finished simulation.")

@dispatcher.public
def prac2cram_client(tasks_RPC):
    global simRunning, ownId, mongoProc
    if (True == simRunning):
        return {"status": -1, "childId": ownId, "retcode": statecodes.RC_NOTREADY, "state": statecodes.SC_BUSY, "message": "Not ready yet.", "messages": [""], "plan_strings": [""], "visualizationIP" : ROSBridgeAddress}

    # Maybe not needed, since the members have the same names, but better safe etc.
    tasks_ROS = getROSTasks(tasks_RPC)
    response = None
    message = ""
    messages = [""]
    planstrings = [""]
    status = -1
    retcode = statecodes.RC_ALLOK
    state = statecodes.SC_BUSY

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
        status = response.status

    except rospy.ServiceException, e:
        response = None
        status = -1
        messages = ["Service call failed with the following error: " + str(e)]
        retcode = statecodes.RC_ROSSRVFAIL
        state = statecodes.SC_IDLE

    if (None != response):
        #Rosrun mongodb logger (but do this only when needed, ie. right before a sim begins)
        if (0 == status):
            simRunning = True
            mongoProc = subprocess.Popen('rosrun mongodb_log mongodb_log /tf /logged_designators /logged_metadata --mongodb-name roslog_' + str(ownId), stdout=None, shell=True, stderr=None, preexec_fn=os.setsid)
            message = "Started simulation."
            #This should not be needed: the parent can deduce the BUSY state based on the return
            #notifyParentOfState(statecodes.SC_BUSY, "started simulation.")
        else:
            state = statecodes.SC_IDLE
            message = "Did not start simulation. See messages for reasons."
        messages = getStringList(response.messages)
        planstrings = getStringList(response.plan_strings)
    return {"status": status, "childId": ownId, "retcode": retcode, "state": state, "message": message, "messages": messages, "plan_strings": planstrings, "visualizationIP" : ROSBridgeAddress}

def CRAMTickCallback(cramTick):
    global doneTicks, simRunning
    instRPC.CRAMTickCallback({"done": cramTick.done, "error": cramTick.error})
    if (False == simRunning) or (0 == cramTick.done):
        doneTicks = 0
    elif (True == simRunning) and (0 != cramTick.done) and (4 > doneTicks):
        doneTicks = doneTicks + 1
    elif (True == simRunning) and (0 != cramTick.done) and (4 <= doneTicks):
        doneTicks = 0
        onDone()

rospy.init_node('sim_rpc')
rospy.Subscriber("cramticks", CRAMTick, CRAMTickCallback)

def exit_gracefully(signum, frame):
    stopMongo()
    sys.exit(0)
signal.signal(signal.SIGINT, exit_gracefully)
signal.signal(signal.SIGTERM, exit_gracefully)


rpc_server.serve_forever()

