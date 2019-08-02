# **bitbots_localization**  
fake_pointcloud --> pointcloud2laserscan --> amcl  
Field models can be created with the map_generator script in bitbots_tools.   

## **dependencies**  
  > rospy  
  > std_msgs  
  > nav_msgs  
  > humanoid_league_msgs  
  > sensor_msgs  
  > robot_localization  
  > pointcloud_to_laserscan  
  > map_server  
  > amcl  

## **Node**
**1. fake_lines_relative**  
   * Send distance information from **'line-relative'**,Get the signal from the signal module.  
   * **msgs type:** [LineInformationRelative](https://github.com/b51/humanoid_league_msgs/blob/2a6ab162d89f8b90338e5b7e3b1f441b55edf1e0/msg/LineInformationRelative.msg)：Contains cross lines, segments, and circles，[LineSegmentRelative](https://github.com/b51/humanoid_league_msgs/blob/2a6ab162d89f8b90338e5b7e3b1f441b55edf1e0/msg/LineSegmentRelative.msg)：Contains the endpoint and confidence of a line segment.  

**2. bitbots_FakePC2**   
   * Receive line distance information from **'line_relative'** and send point cloud information from **'cloud_in'**.  
   * **msg type:** The data type of point cloud data is [PointCloud2](http://docs.ros.org/lunar/api/sensor_msgs/html/msg/PointCloud2.html).
   
**3. amcl_nomotion_update**  
   * Messages are received from '/pre_scan' and published from '/scan' after processing. The message type to receive and publish is [LaserScan](http://docs.ros.org/indigo/api/sensor_msgs/html/msg/LaserScan.html)。   
   * Check robot body movement and head movement. Publish moving data. If so, check for tolerance; If not, send an error message.
