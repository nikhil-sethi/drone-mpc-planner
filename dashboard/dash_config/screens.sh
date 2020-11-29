#! /usr/bin/env bash

i=$(($i + 1))
/usr/bin/screen -t down $i /bin/bash -c 'cd ~/code/pats/dashboard && ./jsons_to_db.py -i ~/jsons -o ~/pats.db -p 3600; exec /bin/bash'
i=$(($i + 1))
/usr/bin/screen -t mail $i /bin/bash -c 'cd ~/code/pats/dashboard && ./daily_digest.py; exec /bin/bash'
i=$(($i + 1))
/usr/bin/screen -t dash $i /bin/bash -c 'cd ~/code/pats/dashboard && gunicorn --workers=3 pats_c:server --bind unix:pats.sock;  exec /bin/bash'
i=$(($i + 1))
/usr/bin/screen -t data $i /bin/bash -c 'cd ~/jsons; exec /bin/bash'
