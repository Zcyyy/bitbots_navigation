# **bitbots_bezier_pathfinding**   
  The bitbots_bezier_pathfinding uses bezier curves for pathfinding.  
## **Dependencies**  
  > catkin  
  > rospy  
  > geometry_msgs  
  > nav_msgs  
  
## **Node**
**1.bezier_pathfinding**  
  * Load the default values and publish information 'path_publisher','command_pose_publisher','command_publisher','goal_subscriber'.Receive the message from 'goal_subscriber'.   
  * **msgs type:** [Path](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Path.html),PoseStamped,[Twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html).
