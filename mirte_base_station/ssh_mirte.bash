#!/bin/bash

# SSH into the remote server and run the setup_ros.sh script
ssh -t mirte@"$MIRTE_IP" '~/setup_ros.sh ; bash'

echo $MIRTE_IP