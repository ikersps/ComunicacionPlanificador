from geometry_msgs.msg import PoseStamped
from .hits_toolsv3 import *
from syst_msgs.msg import StringArray, IntArray
from syst_msgs.msg import Waypoints
import numpy as np

import rclpy
from rclpy.node import Node 

class Collision_avoidance(Node):
    def __init__(self):
        super().__init__('Collision_avoidance')
        self.get_logger().info('Publishing: COLLISION AVOIDANCE')
        self.positions = [0]
        self.initial_positions = [0]  #It will be used to define the drone trajectories
        self.publishers_ = [0]
        self.prev_result = []
        self.countDiffDrones = [0]  #Controls if the position of all drones have arrived
        self.drone_ids_subscription = self.create_subscription(StringArray, f'/drone_ids', self.drone_ids_callback, 10)

    def drone_ids_callback(self, msg):
        drone_ids = msg.drone_ids
        self.get_logger().info('Receiving drones ids COLLISION AVOIDANCE')
        self.positions = self.positions * len(drone_ids)
        self.initial_positions = self.initial_positions * len(drone_ids)
        self.publishers_ = self.publishers_ * len(drone_ids)
        self.countDiffDrones = self.countDiffDrones * len(drone_ids)
        for drone_id in drone_ids:
            Pose_subscription(drone_id, len(drone_ids), self)  #It creates one subscription for the pos of each drone

class Pose_subscription(Node):
    def __init__(self, drone_id, nDrones, collision_avoidance):
        super().__init__('Collision_avoidance' + drone_id)
        self.count = 0  #Counts the amount of positions of the same drone
        self.index = int(drone_id.replace("drone_", ""))
        self.collision_avoidance = collision_avoidance
        self.nDrones = nDrones
        self.prevCount = 0

        # self.wps_subscription = self.collision_avoidance.create_subscription(Waypoints, f'/{drone_id}/route', self.waypoints_callback, 10)
        self.collision_avoidance.publishers_[self.index] = self.create_publisher(IntArray, f'/{drone_id}/collisions', 10)
        self.pose_subscription = self.collision_avoidance.create_subscription(PoseStamped, f'/{drone_id}/pose', self.new_pose_callback, 10)
        #self.collision_avoidance.set_publisher(self.index, self.create_publisher(Float32, f'/{drone_id}/collision_avoidance', 10))
        
        
    def new_pose_callback(self, msg):
        self.count += 1
        if self.count % 5 == 0:
            self.collision_avoidance.positions[self.index] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            self.collision_avoidance.countDiffDrones[self.index] = 1

            if self.collision_avoidance.countDiffDrones == ([1] * self.nDrones):
                
                self.collision_avoidance.countDiffDrones = ([0] * self.nDrones)
                hits_tool = Hits_toolsv3(self.collision_avoidance.initial_positions, self.collision_avoidance.positions, [1])
                result = hits_tool.nearest_positions()

                #If the difference between count and prevCount equals to 30 it means that about 3
                #seconds would have passed since the last time collisions were checked
                if len(result) != 0 and (self.count - self.prevCount >= 30 or result != self.collision_avoidance.prev_result):
                    self.get_logger().info('COLLISION AVOIDANCE result = %s' % result)
                    self.collision_avoidance.prev_result = result
                    idsMsg = IntArray()
                    idsMsg.ids = result
                    for e in result:
                        self.collision_avoidance.publishers_[e].publish(idsMsg)
        elif self.count % 5 == 1:
            self.collision_avoidance.initial_positions[self.index] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

def main():
    rclpy.init()

    collision_avoidance = Collision_avoidance()

    rclpy.spin(collision_avoidance)

    rclpy.shutdown()

if __name__ == '__main__':
    main()