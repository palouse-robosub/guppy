cd /home/robosub/guppy
source install/setup.bash

until ping -c1 192.168.194.95 &>/dev/null; do
    echo "Waiting for DVL..."
    sleep 1
done

export LD_LIBRARY_PATH=/opt/spinnaker/lib:$LD_LIBRARY_PATH
echo ----------ATTEMPTING TO START GUPPY----------
ros2 launch guppy hw.xml
