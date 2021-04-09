#!/usr/bin/env python3

import json
import copy

with open('../../base/src/dronecontroller.cpp') as f:
    src = f.read().splitlines()

current_state = ''
dot_src = "digraph dronecontroller {\n"
for line in src:
    if '*_logger' in line and current_state!= '':
        break

    if 'case' in line:
        words = line.split(' ')
        current_state = words[words.index('case')+1][:-1]
        dot_src += '\t'+current_state+';\n'

    if '_flight_mode' in line and current_state != '':
        state_transition = line.split(' ')[-1][:-1]
        dot_src += '\t'+current_state+' -> '+state_transition+';\n'


dot_src += '}\n'
print(dot_src)

with open('./dronecontroller.gv', 'w') as f:
    f.write(dot_src)
    f.close()
