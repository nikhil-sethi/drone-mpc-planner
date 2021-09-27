# Upgrade systems

- create a monitoring tag, push the tag, set the monitoring branch to this tag. (e.g. `git tag monitoring_v1.666 && git push --tags && git co monitoring && git reset --hard monitoring_v1.666`)
- cd to the ansible folder: `cd ~/code/pats/base/ansible`
- Make sure to select the correct group from the inventory and apply the in the yaml files
- Run `ansible-playbook playbooks/upgrade-monitoring.yaml`
- ONLY if necessary, reboot the systems: `ansible-playbook playbooks/reboot.yaml -K`