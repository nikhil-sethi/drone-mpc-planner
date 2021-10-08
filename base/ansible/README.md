# Upgrade systems

- create a monitoring tag, push the tag, set the monitoring branch to this tag. (e.g. `git tag monitoring_v1.666 && git push --tags && git co monitoring && git reset --hard monitoring_v1.666`)
- cd to the ansible folder: `cd ~/code/pats/base/ansible`
- download the system database `rsync -a dash:patsc/db/pats_systems.db ~/patsc/db/pats_systems.db`
- Make sure to select the correct group from the database and apply the in the yaml files. You can either use one of the customers, by taking the customer name and replacing every space and `.` with `_` , or you can use the bigger groups `monitoring`, `hunts` and `office`.
- Run `ansible-playbook playbooks/upgrade-monitoring.yaml`
- ONLY if necessary, reboot the systems: `ansible-playbook playbooks/reboot.yaml -K`