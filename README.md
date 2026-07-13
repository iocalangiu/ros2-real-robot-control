# ros2-basics-python-real-robot

![ROS2 real robot demo](ros2_real_robot.gif)

The program has three ROS 2 nodes work together: 
- wall follower node – subscribes to /scan, calls the /find_wall service to align with a wall, then publishes velocity commands to /cmd_vel, and sends a goal to the /record_odom action
- wall finder node – serves the /find_wall (FindWall.srv) request, using /scan to locate and align with the wall
- odom recorder node – action server for /record_odom (OdomRecord.action), subscribes to /odom and returns feedback/result on distance traveled

<img width="767" height="423" alt="image" src="https://github.com/user-attachments/assets/dc608871-80fe-4747-908b-040a21973ec8" />
