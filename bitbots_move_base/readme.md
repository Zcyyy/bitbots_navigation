# **bitbots_move_base**  
Change the coordinates of the goal. Location of obstacles released.  
## **dependencies**  
  > rospy
  > humanoid_league_msgs  
  > std_msgs  
  > message_runtime  
  > move_base  
  > gazebo_msgs   
  
## **Node**  
**1. goal_converter**  
  * Changes the header of the read message. By quaternion -> euler Angle -> quaternion operation, only obtain Yaw Angle.Get the robot's orientation relative to the goal?(Need to specific information can be determined)  
  * Receive messages from **'behavior/goal'**. Publish the message through **'move_base_simple/goal'**,Their message types are the same.  
  * **msgs type**:[PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html),Contains a header and a pose message with a position and an orientation in the pose.   

**2. obstacle_publisher**  
  * This node publishes the ball and other obstacles as an obstacle to avoid walking through it.  
  * Receive messages from **'ball_relative'** and **'obstacle_relative'**. They get rid of the **'obstacles'** by publishing messages.The **'move_base/clear_costmaps'** service is also requested.    
  * **msgs type**:[BallRelative](https://github.com/b51/humanoid_league_msgs/blob/2a6ab162d89f8b90338e5b7e3b1f441b55edf1e0/msg/BallRelative.msg),
  [ObstaclesRelative](https://github.com/b51/humanoid_league_msgs/blob/2a6ab162d89f8b90338e5b7e3b1f441b55edf1e0/msg/ObstaclesRelative.msg),
  [PointCloud2](http://docs.ros.org/groovy/api/sensor_msgs/html/msg/PointCloud2.html):Obstacles include robots, balls and unclassified objects (like human legs, etc.). In addition to location information, volume information of each obstacle also has a confidence interval (\[0,1], where 0 means none and 1 means certain).  
  * **srv type**:[Empty](http://docs.ros.org/diamondback/api/std_srvs/html/srv/Empty.html).  
  
**3. map**
  * Send the transformation matrix information of a spatial 'map' transformation to 'odom' (actually described by two vectors). The data information includes a translation vector \[0,0,0], and a quaternion \[0,0,0,1].
