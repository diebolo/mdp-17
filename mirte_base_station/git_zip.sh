#!/bin/bash

scp -r mirte@"$MIRTE_IP":~/mirte_ws/src/ ~/mdp-17/mirte_src

find ~/mdp-17/mirte_src -type d -name ".git" -exec rm -rf {} +

cd ~/mdp-17
git add .
git commit -m "Mirte src update"
git push --force
