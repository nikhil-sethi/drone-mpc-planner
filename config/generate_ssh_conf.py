#!/usr/bin/env python3

import subprocess
import os
import sys
import time


with open("pats_ssh_config", "w") as f:
    for i in range(0, 999):
        f.write('host pats' + str(i))
        i1 = int(10 + (i - (i % 255)) / 255)
        i2 = i % 255
        f.write('\n\tHostname 10.13.' + str(i1) + '.' + str(i2))
        f.write('\n\tIdentityFile ~/.ssh/pats_wg_id_ed25519')
        f.write('\n\tUser pats\n\n')
