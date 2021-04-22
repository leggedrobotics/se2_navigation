#!/usr/bin/env python
import rospy
from std_msgs.msg import String,Float64
import rosservice
from se2_grid_map_generator_msgs.srv import SaveMap,AddCircularObstacle,AddPolygonObstacle
from se2_grid_map_generator_msgs.srv import SaveMapRequest,AddCircularObstacleRequest,AddPolygonObstacleRequest
from se2_grid_map_generator_msgs.msg import CircularObstacle


addCircObstacleSrvName = "/se2_grid_map_generator_node/addCircularObstacle"
saveMapSrvName = "/se2_grid_map_generator_node/saveMap"


def saveMap(filename):
    try:
        saveMapSrv = rospy.ServiceProxy(saveMapSrvName, SaveMap)
        req = SaveMapRequest()
        req.filepath = filename
        saveMapSrv(req)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def addCircObstacle(x,y,r):
    try:
        addCircObstacleSrv = rospy.ServiceProxy(addCircObstacleSrvName, AddCircularObstacle)
        circObstacle  = CircularObstacle()
        circObstacle.layers = [String('elevation'),String('obstacle')]
        circObstacle.values = [Float64(1),Float64(1)]
        circObstacle.circle.radius.data = r
        circObstacle.circle.center.x.data = x
        circObstacle.circle.center.y.data = y
        req = AddCircularObstacleRequest()
        req.obstacle = circObstacle
        addCircObstacleSrv(req)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


rospy.init_node('grid_map_generator')
#generate some circles
rospy.wait_for_service(addCircObstacleSrvName)
addCircObstacle(10, 0, 1)
addCircObstacle(0, 10, 1)
addCircObstacle(10, 15, 3)
addCircObstacle(-5, -10, 4)
addCircObstacle(-8, 8, 3)
addCircObstacle(10, -15, 3)

saveMap("")
print("generating map node done")