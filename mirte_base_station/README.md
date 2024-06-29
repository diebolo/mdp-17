# Group 17 MDP
Thank you for taking the time to look at our codebase! In this repo you will find our solution to the Lely Discovery project. Below are some instructions on how to install, setup and run our code.

# Install
```bash
#First clone the repo, important to use recursive :
git clone --recursive git@gitlab.tudelft.nl:cor/ro47007/2024/group17.git
cd group17/src/mirte-ros-packages
rm -rf mirte_bringup mirte_control mirte_telemetrix mirte_teleop
cd ../..

# To pull our cpswarm branch
git submodule update --remote

# Run rosdep
rosdep update
rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y
catkin build
```

# Setup mirte on wifi
**Start your phone hotspot before starting mirte**

Use a phone hotspot
name: `mirte_17`
password: `mirtepassword`

Start mirte
Go to `http://mirte.local/#/Settings` when connected to mirte
Select your network

We need to find the ip of mirte in the hotspot network
One way to do this:
```bash
hostname -I
# Look for last 3 numbers, for example:
192.168.42.XXX
# then:

sudo apt-get install nmap

# Then for example:
nmap -sn 192.168.42.0/24
```
Output for me:
```bash
Starting Nmap 7.80 ( https://nmap.org ) at 2024-05-14 10:50 CEST
Nmap scan report for DavidLaptopWS (192.168.42.101)
Host is up (0.00025s latency).
Nmap scan report for _gateway (192.168.42.142)
Host is up (0.0054s latency).
Nmap scan report for 192.168.42.253
Host is up (0.11s latency).
Nmap done: 256 IP addresses (3 hosts up) scanned in 20.91 seconds
```

So mirte is `192.168.42.253`.

Then you can do the standard connect with ssh as above.
`ssh mirte@192.168.42.253`
From now on refered to as `192.168.42.YYY`
## First time Mirte connect
On your first time ssh connect run the following commands

```bash
ssh-copy-id mirte@192.168.XXX.YYY
ssh mirte@192.168.XXX.YYY
```

# Run
How to run the full solution.

First connect to mirte over the hotspot and open a terminal with ros1 noetic on the laptop. Then execute the following commands:
```bash
# Connect with mirte over wifi
source ./setup_ip.bash
./ssh_mirte.bash
# You are now connected to the mirte filesystem using ssh
# First stop the mirte service
sudo service mirte-ros stop
# The source ros, the local workspace and setup the ips
source ~/setup_ros.sh
# Run the launchfile
roslaunch mirte_clean_robot full_robot.launch
```
Now on the laptop basestation run the following commands
```bash
# In terminal 1
source ./setup_ip.bash
roslaunch mirte-clean full_local.launch

# In terminal 2
source ./setup_ip.bash
rviz -d full_local_vis.rviz
```