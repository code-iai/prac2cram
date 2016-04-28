#!/usr/bin/env python
# -*- coding: utf-8 -*-

from tinyrpc.protocols.jsonrpc import JSONRPCProtocol
from tinyrpc.transports.http import HttpPostClientTransport
from tinyrpc import RPCClient

rpc_client = RPCClient(
    JSONRPCProtocol(),
    HttpPostClientTransport('http://127.0.0.1:5050/')
)

remote_server = rpc_client.get_proxy()

#core1 = {'action_core_name': 'Starting', 'action_roles': [{'role_name': 'obj_to_be_started', 'role_value': 'centrifuge.n.01'}]}
#core2 = {'action_core_name': 'dbg-prac2cram', 'action_roles': [{'role_name': 'device', 'role_value': 'centrifuge.n.01'}]}
core = {'action_core_name': 'use-pipette', 'action_roles': [{'role_name': 'content', 'role_value': 'phenolphtaleine'}, {'role_name': 'goal', 'role_value': 'purine'}, {'role_name': 'amount', 'role_value': '2'}, {'role_name': 'unit', 'role_value': 'drops'}]}
action_cores = [core]

# call a method called 'reverse_string' with a single string argument
result = remote_server.prac2cram_client(action_cores)

print "Server answered:" 
print result

