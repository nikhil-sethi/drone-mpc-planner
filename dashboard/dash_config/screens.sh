#! /usr/bin/env bash

/usr/bin/screen -t down /bin/bash -c 'cd ~/code/pats/dashboard && ./dash_daemon.py; exec /bin/bash'
/usr/bin/screen -t mail /bin/bash -c 'cd ~/code/pats/dashboard && ./daily_digest.py; exec /bin/bash'
/usr/bin/screen -t patsc /bin/bash -c 'cd ~/code/pats/dashboard &&  docker logs --follow dashboard_pats_c_1 ; exec /bin/bash'
/usr/bin/screen -t bash /bin/bash -c 'cd ~ ; exec /bin/bash'
