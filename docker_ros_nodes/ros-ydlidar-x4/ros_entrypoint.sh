#!/bin/bash
set -e

#Give some time for master to startup
sleep 5
# setup ros environment
source ".bashrc"
exec "$@"
