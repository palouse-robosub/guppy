sudo apt-get update
sudo apt-get install can-utils
sudo modprobe vcan
sudo ip link add dev can0 type vcan
sudo ip link set up can0