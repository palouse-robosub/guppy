apt-get update
apt-get install can-utils
modprobe vcan
ip link add dev can0 type vcan
ip link set up can0