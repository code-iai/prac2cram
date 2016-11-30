import sys
import signal

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
from p2c_rosaux import getROSTasks, getStringList

portOffsNum = int(sys.argv[1])
rpcPort = int(sys.argv[2])

parentURL = None
parentClient = None
parentRPC = None
if (3 < len(sys.argv)):
    parentURL = sys.argv[3]
    parentClient = RPCClient(JSONRPCProtocol(), HttpPostClientTransport(parentURL))
    parentRPC = parentClient.get_proxy()

mongoProc = None

simRunning = False

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

def notifyParentOfState(state):
    if (None != parentRPC):
        parentRPC.notify_state({"childId": portOffsNum, "state": state})

def sendMongoLogsToParent():
    #TODO: insert some notification to the parent here that mongo logs are available
    return None

def onDone():
    #If running, terminate mongo logging
    if (None != mongoProc):
        os.killpg(os.getpgid(mongoProc), signal.SIGTERM)
        mongoProc = None
    simRunning = False
    sendMongoLogsToParent()
    notifyParentOfState(statecodes.SC_IDLE)

@dispatcher.public
def start_simulation(tasks_RPC):
    if (True == simRunning):
        return {"childId": portOffsNum, "retcode": statecodes.RC_NOTREADY, "state": statecodes.SC_BUSY, "message": "Not ready yet."}

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

    except rospy.ServiceException, e:
        response = None
        status = -1
        message = "Service call failed with the following error: " + str(e)
        retcode = statecodes.RC_ROSSRVFAIL
        state = statecodes.SC_IDLE

    if (None != response):
        #Rosrun mongodb logger (but do this only when needed, ie. right before a sim begins)
        simRunning = True
        mongoProc = subprocess.Popen('rosrun mongodb_log mongodb_log /tf /logged_designators /logged_metadata --mongodb-name roslog_' + str(portOffsNum), stdout=subprocess.NONE, shell=True, stderr=subprocess.NONE, preexec_fn=os.setsid)
        message = "Started simulation."
        messages = getStringList(response.messages)
        planstrings = getStringList(response.plan_strings)
        #This should not be needed: the parent can deduce the BUSY state based on the return
        #notifyParentOfState(statecodes.SC_BUSY)
    return {"childId": portOffsNum, "retcode": retcode, "state": state, "message": message, "messages": messages, "plan_strings": planstrings}

def CRAMTickCallback(cramTick):
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
    sys.exit(0)
signal.signal(signal.SIGINT, exit_gracefully)
signal.signal(signal.SIGTERM, exit_gracefully)


rpc_server.serve_forever()
