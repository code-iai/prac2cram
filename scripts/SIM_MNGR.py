import os
import subprocess
import time

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
wsgi_server = gevent.wsgi.WSGIServer(('0.0.0.0', 4040), transport.handle)
gevent.spawn(wsgi_server.serve_forever)

rpc_server = RPCServerGreenlets(
    transport,
    JSONRPCProtocol(),
    dispatcher
)

childNums = (0, 1)
childPackages = ('pizza_demo', 'pizza_demo')
subprocesses = []

for k, p in zip(childNums, childPackages):
    subprocesses.append(subprocess.Popen('python sim_inst_mngr.py ' + str(k) + ' ' + str(p), stdout=subprocess.PIPE, shell=True, stderr=subprocess.PIPE, preexec_fn=os.setsid))

#@dispatcher.public
#def <RPC server code>:


rpc_server.serve_forever()
