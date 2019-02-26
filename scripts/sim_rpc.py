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
from prac2cram.srv import Prac2Cram, CancelSim
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

SIMLifeTime = 4*60*60
nLTThread = None
def MonitorLifetime():
    global simRunning, SIMLifeTime
    while True:
        time.sleep(1)
        if False == simRunning:
            SIMLifeTime = SIMLifeTime - 1
        else:
            SIMLifeTime = 4*60*60
        if 0 >= SIMLifeTime:
            break
    SIMLifeTime = 4*60*60
    print "Spent 4h in IDLE state. Will now reboot."
    simRunning = True
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

def getDBName(anId):
    return "roslog_" + str(anId)

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
    global ownId
    dbName = getDBName(ownId)
    cmdMkDir = "mkdir ./" + dbName
    subprocess.call(cmdMkDir, stdout=None, stderr=None, shell=True)
    cmdExpTF = "mongoexport --db " + dbName + " --collection tf --out " + "./" + dbName + "/tf.json"
    subprocess.call(cmdExpTF, stdout=None, stderr=None, shell=True)
    cmdExpDesig = "mongoexport --db " + dbName + " --collection logged_designators --out " + "./" + dbName + "/logged_designators.json"
    subprocess.call(cmdExpDesig, stdout=None, stderr=None, shell=True)
    cmdDropDB = "mongo " + dbName + ' --eval "db.dropDatabase()"'
    subprocess.call(cmdDropDB, stdout=None, stderr=None, shell=True)
    #TODO: add semrec exporting here
    #TODO: add send-to-OpenEASE code here
    cmdClr = "rm -r ./" + dbName
    subprocess.call(cmdClr, stdout=None, stderr=None, shell=True)
    #TODO: insert some notification to the parent here that mongo logs are available, and where to find them
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
def prac2cram_cancel_simulation():
    rospy.wait_for_service('prac2cram/cancel_sim', timeout=5)
    try:
        prac2cramCancelSim = rospy.ServiceProxy('prac2cram/cancel_sim', CancelSim)
        response = prac2cramCancelSim()
        return {'status': response.status, 'result': response.result}
    except rospy.ServiceException, e:
        print "Service call failed with the following error: %s" %e

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
            mongoProc = subprocess.Popen('rosrun mongodb_log mongodb_log /tf /logged_designators /logged_metadata --mongodb-name ' + getDBName(ownId), stdout=None, shell=True, stderr=None, preexec_fn=os.setsid)
            message = "Started simulation."
            #This should not be needed: the parent can deduce the BUSY state based on the return
            #notifyParentOfState(statecodes.SC_BUSY, "started simulation.")
        else:
            state = statecodes.SC_IDLE
            message = "Did not start simulation. See messages for reasons."
        messages = getStringList(response.messages)
        messages.append("TASKS_RPC: " + str(tasks_RPC))
        messages.append("TASKS_ROS: " + str(tasks_ROS))
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

