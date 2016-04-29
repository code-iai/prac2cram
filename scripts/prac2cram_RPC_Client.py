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
#core = {'action_core_name': 'use-measuring-cup', 'action_roles': [{'role_name': 'content', 'role_value': 'phenolphtaleine'}, {'role_name': 'goal', 'role_value': 'purine'}, {'role_name': 'amount', 'role_value': '2'}, {'role_name': 'unit', 'role_value': 'drops'}]}

#core = {'action_core_name': 'turn-on--electrical-device', 'action_roles': [{'role_name': 'device', 'role_value': 'mixer'}, {'role_name': 'part', 'role_value': 'on-button'}]}
#action_cores = [core]
#tasks = [action_cores]


## Neutralization

placing = {'action_cores': [{'action_core_name': 'put-object', 'action_roles': [{'role_name': 'substance', 'role_value': 'unknownsubstance'}, {'role_name': 'location', 'role_value': 'mixer'}]}]}
pipetting = {'action_cores': [{'action_core_name': 'use-pipette', 'action_roles': [{'role_name': 'content', 'role_value': 'indicator'}, {'role_name': 'goal', 'role_value': 'UnknownSubstance'}, {'role_name': 'amount', 'role_value': '2'}, {'role_name': 'unit', 'role_value': 'drops'}]}]}
pouring = {'action_cores': [{'action_core_name': 'use-measuring-cup', 'action_roles': [{'role_name': 'content', 'role_value': 'purine'}, {'role_name': 'goal', 'role_value': 'UnknownSubstance'}, {'role_name': 'amount', 'role_value': '100'}, {'role_name': 'unit', 'role_value': 'ml'}]}]}
turning_on_mixer = {'action_cores': [{'action_core_name': 'turn-on-electrical-device', 'action_roles': [{'role_name': 'device', 'role_value': 'mixer'}, {'role_name': 'part', 'role_value': 'on-button'}]}]}
neutralization_tasks = [placing, turning_on_mixer, pipetting, pouring, turning_on_mixer]

# call a method called 'reverse_string' with a single string argument
result = remote_server.prac2cram_client(neutralization_tasks)

print "Server answered:" 
print result

