#!/usr/bin/env python3

import math
import rospy

from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill, SetPen
from geometry_msgs.msg import Twist, Vector3

def waitService(name, t):
    rospy.wait_for_service(name)
    srv = rospy.ServiceProxy(name, t)
    return srv

class ChasingTurtleNode:

    def __init__(self, target):
        self.target = target
        self.p1 = None
        self.p2 = None
        self.stop = False

    def targetPoseCallback(self, data):
        self.p1 = data

    def selfPoseCallback(self, data):
        self.p2 = data

    def run(self):
        rospy.init_node('chasing_turtle', anonymous=True)
        spawn = waitService('spawn', Spawn)
        kill = waitService('kill', Kill)

        x0 = rospy.get_param('~initial/x', 1.0)
        y0 = rospy.get_param('~initial/y', 1.0)
        t0 = rospy.get_param('~initial/t', 0.0)
        vscale = rospy.get_param('~velocity/scale', 2.0)
        vmax = rospy.get_param('~velocity/max', 3.0)
        target = rospy.get_param('target', self.target)

        name = spawn(x=x0, y=y0, theta=t0).name

        sub1 = rospy.Subscriber(f'{self.target}/pose', Pose, self.targetPoseCallback)
        sub2 = rospy.Subscriber(f'{name}/pose', Pose, self.selfPoseCallback)
            
        pub = rospy.Publisher(f'{name}/cmd_vel', Twist, queue_size=1)
        rate = rospy.Rate(10)
        
        while not self.stop:
            if None in (self.p1, self.p2):
                continue

            distance = ((self.p1.x - self.p2.x) ** 2 + (self.p1.y - self.p2.y) ** 2) ** 0.5
            angle = math.atan2(self.p1.y - self.p2.y, self.p1.x - self.p2.x)

            pub.publish(
            	Twist(
                    linear=Vector3(1.5 * distance, 0.0 ,0.0),
                    angular=Vector3(0.0, 0.0, 4 * abs(angle - self.p2.theta)),
                ))

        try:
            kill(name=name)
        except rospy.service.ServiceException:
            pass



def main():

    target = 'turtle1'

    ct = ChasingTurtleNode(target)
    ct.run()

if __name__ == "__main__":
    exit(main() or 0)
