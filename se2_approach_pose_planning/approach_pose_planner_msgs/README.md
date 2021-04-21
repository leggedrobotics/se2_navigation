# se2\_navigation_msgs

Collection of messages and services for communication between ros planners and controllers. These messages are also used to communicate to the rviz planning interface.

## Messages
-----------
#### Path Segment (PathSegmentMsg.msg)   
+  *int8 FORWARD = 0*   
+  *int8 BACKWARDS = 1*   
+  *int8 drivingDirection* - variable holding the driving direction  
+  *geometry_msgs/Pose[] points* - poses of the robot along the path      
      
#### Path (PathMsg.msg)  
+  *std_msgs/Header header*   
+  *se2_navigation_msgs/PathSegmentMsg[] segment* - path segments   

#### Path Request (PathRequestMsg.msg)  
+ *geometry_msgs/Pose startingPose*   
+ *geometry_msgs/Pose goalPose*   

#### Controller Command (ControllerCommandMsg.msg)   

+ *int8 START_TRACKING=0*   
+ *int8 STOP_TRACKING=1*  
+ *int8 command*   

## Services
-----------

#### Request Current State (RequestCurrentStateSrv.srv)  
+ **Request**
    + `-`
+ **Response**  
    + *geometry_msgs/Pose* - current pose of the robot  
    + *geometry_msgs/Twist* - twist of the robot

#### Request Path (RequestPathSrv.srv)  
+ **Request**
    + *PathRequestMsg pathRequest* - starting and goal pose for the planner
+ **Response**  
    + *bool status* - whether path has been succesfully found  

#### Send Controller Command (SendControllerCommandSrv.srv)  
+ **Request**
    + *ControllerCommandMsg command* - command for the controller
+ **Response**  
    + *bool status* - whether controller has done what was asked for 
 
## Dependencies

* std_msgs

* geometry_msgs
