# Upgrade systems

- cd to the ansible folder: `cd ~/code/pats/base/ansible`
- Make sure to select the correct group from the inventory and apply the in the yaml files
- Run `ansible-playbook playbooks/upgrade-monitoring.yaml`
- ONLY if necessary, reboot the systems: `ansible-playbook playbooks/reboot.yaml -K`