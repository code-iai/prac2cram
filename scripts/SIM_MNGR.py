#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import time
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
    print 'Opening subprocess ' + 'python sim_inst_mngr.py ' + str(k) + ' ' + str(p)
    subprocesses.append(subprocess.Popen('python sim_inst_mngr.py ' + str(k) + ' ' + str(p), stdout=None, shell=True, stderr=None, preexec_fn=os.setsid))

def exit_gracefully(sig, frame):
    for s in subprocesses:
        os.killpg(os.getpgid(s.pid), signal.SIGTERM)
    sys.exit(0)

#@dispatcher.public
#def <RPC server code>:


rpc_server.serve_forever()
