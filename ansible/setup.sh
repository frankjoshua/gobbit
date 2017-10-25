#!/bin/bash
ANSIBLE_TRANSPORT="ssh"
ANSIBLE_SSH_ARGS="-o ForwardAgent=yes"
export ANSIBLE_HOST_KEY_CHECKING=False
ansible-playbook -i ./hosts robot.yml $@
