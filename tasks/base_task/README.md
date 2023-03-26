# Base clover task

![visualiztion](vis.png)

This is base task for cloversim that provides world with 10 by 10 meters aruco map.

This task also contains simple checker, that tests flying in square with side of 1.
Exact steps drone should execute:

1. Takeoff
1. Fly 1m positive x
1. Fly 1m positive y
1. Fly 1m negative x
1. Fly 1m negative y
1. Land

## Step-by-step tutorial

1. To control drone flight we need to communicate with over ROS clover node. So we need to import rospy, init ROS node and connect to clover services:

   ```py
   import rospy
   from clover import srv
   from std_srvs.srv import Trigger
   import math

   rospy.init_node('flight')

   land = rospy.ServiceProxy('land', Trigger)
   get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
   navigate = rospy.ServiceProxy('navigate', srv.Navigate)


   ```

2. First step of task is to take off land. We also need way to track whether we arrived at target position, so we will add `navigate_wait` function to our code. Then to take off we need to call `navigate_wait` with frame body, auto_arm enabled and z set to needed height:

   ```py
   def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
      navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

      while not rospy.is_shutdown():
         telem = get_telemetry(frame_id='navigate_target')
         if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
               break
         rospy.sleep(0.2)

   navigate_wait(frame_id='body', auto_arm=True, z=1)
   ```

   Try to run your program and see drone taking off, you should get points for Point A. (use reset simulator to move everything to initial state)

3. Next step is to flight in right directions to fly in square, this is achieved by using `navigate_wait`:
   ```py
   navigate_wait(frame_id='navigate_target', x=1)
   navigate_wait(frame_id='navigate_target', y=1)
   navigate_wait(frame_id='navigate_target', x=-1)
   navigate_wait(frame_id='navigate_target', y=-1)
   ```

   Try to run your program, now drone should take off and fly in square, you should get 40 points. (use reset simulator to move everything to initial state)

4. To get final points you need to land drone with `land`:
    ```py
    land()
    ```