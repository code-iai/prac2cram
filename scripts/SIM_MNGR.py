#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import time
import signal
import datetime
import subprocess

import string
import random

import gevent
import gevent.wsgi
import gevent.queue
from tinyrpc.protocols.jsonrpc import JSONRPCProtocol
from tinyrpc.transports.wsgi import WsgiServerTransport
from tinyrpc.server.gevent import RPCServerGreenlets
from tinyrpc.dispatch import RPCDispatcher
from tinyrpc.transports.http import HttpPostClientTransport
from tinyrpc import RPCClient

from threading import Thread

import statecodes

dispatcher = RPCDispatcher()
transport = WsgiServerTransport(queue_class=gevent.queue.Queue)

ownPort = 4040

# start wsgi server as a background-greenlet
wsgi_server = gevent.wsgi.WSGIServer(('0.0.0.0', ownPort), transport.handle)
gevent.spawn(wsgi_server.serve_forever)

rpc_server = RPCServerGreenlets(
    transport,
    JSONRPCProtocol(),
    dispatcher
)

#This should be read out of some config info
#childNums = (0, 1)
#childPackages = ('pizza_demo', 'pizza_demo')
childNums = (1,)
childPackages = ('pizza_demo',)
packageActions = {'pizza_demo': ['Cutting']}

subprocesses = []
childThreads = []

childStates = {}
childIds = []
child2PackageMap = {}

# WARNING: Remote clients must only know a child's alias. They must never learn the childId.
childAliases = {}
childAlias2Id = {}
childClients = {}
childClientConnection = {}
clientChildren = {}

childRPCNodes = {}
childRPCs = {}

def createId(size=8, chars=string.ascii_uppercase+string.ascii_lowercase+string.digits):
    return ''.join(random.SystemRandom().choice(chars) for _ in range(size))

def findFirstAction(tasksRPC):
    if type(tasksRPC) != type([0,]):
        return None
    actionDesc = tasksRPC[0]
    if "action_cores" not in actionDesc:
        return None
    actionCores = actionDesc["action_cores"]
    if type(actionCores) != type([1,]):
        return None
    specActionCore = actionCores[-1]
    if "action_core_name" not in specActionCore:
        return None
    else:
        return specActionCore["action_core_name"]    
    
def findWorldsByAction(firstAction):
    global packageActions
    worlds = None
    for package in packageActions:
        if firstAction in packageActions[package]:
            if None == worlds:
                worlds = []
            worlds.append(package)
    return worlds

def findFreeChildInWorld(w):
    global childPackages, childIds, childStates, childClients
    childId = None
    for c, p in zip(childIds, childPackages):
        if (p == w) and (statecodes.SC_IDLE == childStates[c]) and (c not in childClients):
            childId = c
            break
    return childId

def findIdleClientChild(clientId, firstAction):
    global childStates, clientChildren, packageActions, child2PackageMap
    childId = None
    for c in clientChildren[clientId]:
        if (statecodes.SC_IDLE == childStates[c]) and (firstAction in packageActions[child2PackageMap[c]]):
            childId = c
            break
    return childId

def childWatchdog(childId):
    global childAliases, childAlias2Id, childClientConnection, childStates, childClients, clientChildren
    alias = childAliases[childId]
    while True:
        time.sleep(1)
        if (statecodes.SC_BUSY != childStates[childId]):
            childClientConnection[childId] = childClientConnection[childId] - 1
        if (0 > childClientConnection[childId]):
            break
    clientId = childClients.pop(childId, None)
    if (clientId != None) and (clientId in clientChildren):
        clientChildren[clientId].remove(childId)
        if (0 == len(clientChildren[clientId])):
            clientChildren.pop(clientId, None)
            print "Client " + str(clientId) + " timed out on all child connections, will remove from records."
    print "Child " + str(childId) + " disconnected from client " + str(clientId)
    #TODO: this is a good place to reset the child, actually
    #Create new child alias
    childAliases.pop(childId, None)
    childAlias2Id.pop(alias, None)
    alias = createId()
    childAliases[childId] = alias
    childAlias2Id[alias] = childId
    print "Ended watchdog for child " + str(childId) + "."

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

for k, p in zip(childNums, childPackages):
    newId = createId()
    newAlias = createId()
    cmdStr = 'python sim_inst_mngr.py ' + str(k) + ' ' + str(p) + ' ' + str(ownPort) + ' ' + str(newId)
    print 'Opening subprocess ' + cmdStr
    childIds.append(newId)
    child2PackageMap[newId] = p
    childStates[newId] = statecodes.SC_BOOTING
    childAliases[newId] = newAlias
    childAlias2Id[newAlias] = newId
    nThread = Thread(target=execute_command, args=(cmdStr,))
    childThreads.append(nThread)
    nThread.setDaemon(True)
    nThread.start()
    print 'Connecting to that instance\'s RPC'
    rpcURL = "http://localhost:" + str(5050 + k)
    rpcClient = RPCClient(JSONRPCProtocol(), HttpPostClientTransport(rpcURL))
    childRPC = rpcClient.get_proxy()
    childRPCNodes[newId] = rpcClient
    childRPCs[newId] = childRPC

def exit_gracefully(sig, frame):
    for s in subprocesses:
        os.killpg(os.getpgid(s.pid), signal.SIGTERM)
    sys.exit(0)
signal.signal(signal.SIGINT, exit_gracefully)
signal.signal(signal.SIGTERM, exit_gracefully)

@dispatcher.public
def notify_state(chSt):
    global childStates, childClients
    childId = chSt["childId"]
    state = chSt["state"]
    if childId in childStates:
        childStates[childId] = state
        print str(datetime.datetime.now().time()) + " ChildId " + str(childId) + " passed to state " + statecodes.stateName(state)
        #TODO: Notify client if one exists-- REMEMBER TO USE THE CHILDALIAS FOR THAT
    else:
        print "Received notification from unknown childId " + str(childId)

@dispatcher.public
def prac2cram_client(command):
    global childClients, childAliases, childClientConnection, childRPCs, clientChildren
    childAlias = None
    childId = None
    tasksRPC = None
    clientId = None
    if "clientId" in command:
        clientId = command["clientId"]
    else:
        return {"status": "ERROR: client did not supply an id, and its request is ignored.", "result": {}}
    if "tasks" in command:
        tasksRPC = command["tasks"]
    else:
        return {"status": "ERROR: command contained no action cores, and is ignored.", "result": {}}
    firstAction = findFirstAction(tasksRPC)
    if None == firstAction:
        return {"status": "ERROR: couldn't find first action core; request may be malformed and was ignored.", "result": {}}
    if ("childId" in command) and ("" != command["childId"]):
        #If the command names a child (using its alias) to use, check that this client has a connection to that child
        childAlias = command["childId"]
        if childAlias in childAlias2Id:
            childId = childAlias2Id[childAlias]
            expectedClient = "not" + clientId
            if childId in childClients:
                expectedClient = childClients[childId]
            if expectedClient != clientId:
                return {"status": "ERROR: client claimed connection to a child already claimed by another client; request ignored.", "result": {}}
        else:
            return {"status": "ERROR: client claimed connection to an unrecognized child id; request ignored.", "result": {}}
    elif (clientId in clientChildren):
        #If the command does not name a child, nevertheless check if any children are connected to this client, are not busy,
        #and are in a world that implements the required first action.
        childId = findIdleClientChild(clientId, firstAction)
    else:
        childId = None
    if (None == childId):
        #Find a free child in a world that implements the first action
        worldsToTry = findWorldsByAction(firstAction)
        if None == worldsToTry:
            return {"status": "ERROR: the first requested action core doesn't seem to fit any of the child worlds; request ignored.", "result": {}}
        for w in worldsToTry:
            childId = findFreeChildInWorld(w)
            if None != childId:
                break
        if None == childId:
            return {"status": "ERROR: no child available to do the first action right now (they're all busy); request ignored. Try again in a few minutes.", "result": {}}
        childAlias = childAliases[childId]
    childClientConnection[childId] = 5*60
    if clientId not in clientChildren:
        clientChildren[clientId] = []
        print "Client " + str(clientId) + " has been added to the record."
    if childId not in clientChildren[clientId]:
        clientChildren[clientId].append(childId)
        print "Registering child " + str(childId) + " as connected to client " + str(clientId) + "."
    if childId not in childClients:
        print "Started a watchdog for child " + str(childId) + "."
        childClients[childId] = clientId
        nThread = Thread(target=childWatchdog, args=(childId,))
        nThread.setDaemon(True)
        nThread.start()
    result = childRPCs[childId].prac2cram_client(tasksRPC)
    #Note: we let the child's own notification services to take care of notifying IDLE/BUSY on state changes.
    # Web client must never learn the child's Id, or else they could spoof notifications of state.
    result["childId"] = childAlias
    result["retcode"] = statecodes.retcodeName(result["retcode"])
    result["state"] = statecodes.stateName(result["state"])
    if ("status" in result) and (0 == result["status"]):
        childStates[childId] = statecodes.SC_BUSY
        print str(datetime.datetime.now().time()) + " ChildId " + str(childId) + " passed to state " + statecodes.stateName(statecodes.SC_BUSY)
    return {"status": "Sent request to simulation, see result.", "result": result}

rpc_server.serve_forever()

