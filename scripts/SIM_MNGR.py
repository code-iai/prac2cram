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

from threading import Thread

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

#childNums = (0, 1)
#childPackages = ('pizza_demo', 'pizza_demo')
childNums = (1,)
childPackages = ('pizza_demo',)
subprocesses = []
childThreads = []

def execute_command(cmd):
    global subprocesses
    print 'Exec: ' + cmd
    pr = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True, preexec_fn=os.setsid)
    subprocesses.append(pr)
    while True:
        time.sleep(1)
        while True:     
            lineO = pr.stdout.readline()
            #print lineO
            if not lineO: break
    #print "No more output from " + cmd
    #print "Thinking it's done, will exit it."
    #os.killpg(os.getpgid(pr.pid), signal.SIGTERM)

for k, p in zip(childNums, childPackages):
    cmdStr = 'python sim_inst_mngr.py ' + str(k) + ' ' + str(p)
    print 'Opening subprocess ' + cmdStr
    nThread = Thread(target=execute_command, args=(cmdStr,))
    childThreads.append(nThread)
    nThread.setDaemon(True)
    nThread.start()
    #subprocesses.append(subprocess.Popen('python sim_inst_mngr.py ' + str(k) + ' ' + str(p), stdout=subprocess.PIPE, shell=True, stderr=subprocess.PIPE, preexec_fn=os.setsid))

def exit_gracefully(sig, frame):
    for s in subprocesses:
        os.killpg(os.getpgid(s.pid), signal.SIGTERM)
    sys.exit(0)
signal.signal(signal.SIGINT, exit_gracefully)
signal.signal(signal.SIGTERM, exit_gracefully)

#@dispatcher.public
#def <RPC server code>:


rpc_server.serve_forever()

