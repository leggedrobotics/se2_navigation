# approach\_pose\_planner\_msgs

Collection of messages that are used to request paths from the approach pose planner. 

## Messages
-----------

#### Approach Pose Request (ApproachPoseRequestMsg.msg)  
+ *geometry_msgs/Pose startingPose* - current robot pose   
+ *geometry_msgs/Point goalPoint* - target location (x,y) where the robot should come close to

## Services
-----------

#### Request Approach Pose (RequestPathSrv.srv)  
+ **Request**
    + *ApproachPoseRequestMsg approachPoseRequest* 
+ **Response**  
    + *bool status* - whether an approach pose has been succesfully has been succesfully found  

 
## Dependencies

* std\_msgs

* geometry\_msgs

* se2\_navigation\_msgs
