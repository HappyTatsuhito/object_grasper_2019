xterm -fn 10x20 -geometry 38x11+0+980 -e "/opt/ros/kinetic/bin/rostopic echo /m0_controller/state" &
sleep 0.1s
xterm -fn 10x20 -geometry 38x11+382+980 -e "/opt/ros/kinetic/bin/rostopic echo /m1_controller/state" &
sleep 0.1s
xterm -fn 10x20 -geometry 38x11+764+980 -e "/opt/ros/kinetic/bin/rostopic echo /m2_controller/state" &
sleep 0.1s
xterm -fn 10x20 -geometry 38x11+1146+980 -e "/opt/ros/kinetic/bin/rostopic echo /m3_controller/state" &
sleep 0.1s
xterm -fn 10x20 -geometry 38x11+1528+980 -e "/opt/ros/kinetic/bin/rostopic echo /m4_controller/state" &
sleep 0.1s
