source devel/setup.bash

ROS_IP=$(hostname -I | awk '{print $1}')

MIRTE_HOST=$(hostname -I | awk '{print $1}' | cut -d '.' -f 1-3)

ping -c 1 $MIRTE_HOST.253 >/dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "Mirte host is reachable."
    MIRTE_IP=$MIRTE_HOST.253
else
    read -p "Mirte host is not reachable. Enter Mirte IP: ${MIRTE_HOST}." MIRTE_CLIENT
    MIRTE_IP=${MIRTE_HOST}.${MIRTE_CLIENT}
fi


export MIRTE_IP

echo $MIRTE_IP

ssh mirte@"$MIRTE_IP" sudo date --set @$(date -u +%s)

export ROS_IP
echo $ROS_IP

export ROS_MASTER_URI=http://${MIRTE_IP}:11311
echo ${ROS_MASTER_URI}
