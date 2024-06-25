from geometry_msgs.msg import PoseStamped
from .hits_toolsv2 import *
from syst_msgs.msg import StringArray, DoubleArray
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
        self.waypoints = [0]
        self.next_waypoints = [0]  
        self.prev_result = []
        self.countDiffDrones = [0]  #Controls if the position of all drones have arrived
        self.prevCount = 0
        self.speeds = []
        self.drone_ids_subscription = self.create_subscription(StringArray, f'drone_ids', self.drone_ids_callback, 10)
        self.speeds_subscription = self.create_subscription(DoubleArray, f'/speeds', self.speeds_callback, 10)

    def drone_ids_callback(self, msg):
        self.drone_ids = msg.drone_ids
        self.get_logger().info('Receiving drones ids COLLISION AVOIDANCE')
        self.positions = self.positions * len(self.drone_ids)
        self.initial_positions = self.initial_positions * len(self.drone_ids)
        self.waypoints = self.waypoints * len(self.drone_ids)
        self.next_waypoints = self.next_waypoints * len(self.drone_ids)
        self.countDiffDrones = self.countDiffDrones * len(self.drone_ids)
        index = 0
        for _ in self.drone_ids:
            Pose_subscription(index, len(self.drone_ids), self)  #It creates one subscription for the pos of each drone
            index += 1

    def speeds_callback(self, msg):
            self.speeds = msg.speeds
class Pose_subscription(Node):
    def __init__(self, index, nDrones, collision_avoidance):
        self.collision_avoidance = collision_avoidance
        drone_id = self.collision_avoidance.drone_ids[index]
        super().__init__('Collision_avoidance' + drone_id)
        self.get_logger().info('INDEX %d ID %s' % (index, drone_id))
        self.count = 0  #Counts the amount of positions of the same drone
        self.index = index
        self.nDrones = nDrones
        self.posWps = [0] * nDrones 
        self.wps_subscription = self.collision_avoidance.create_subscription(Waypoints, f'/{drone_id}/route', self.waypoints_callback, 10)
        self.pose_subscription = self.collision_avoidance.create_subscription(PoseStamped, f'/{drone_id}/pose', self.new_pose_callback, 10)
        #self.collision_avoidance.set_publisher(self.index, self.create_publisher(Float32, f'/{drone_id}/collision_avoidance', 10))
        
        
    def new_pose_callback(self, msg):
        self.count += 1
        if self.count % 5 == 0:
            self.collision_avoidance.positions[self.index] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            self.collision_avoidance.countDiffDrones[self.index] = 1

            if self.collision_avoidance.countDiffDrones == ([1] * self.nDrones):
                self.collision_avoidance.countDiffDrones = ([0] * self.nDrones)

                for i in range(self.nDrones):
                    next_wps = self.next_waypoint(self.collision_avoidance.waypoints[i], self.collision_avoidance.positions[i])
                    self.collision_avoidance.next_waypoints[i] = next_wps[1]
                    self.posWps[i] = next_wps[0]
                hits_tool = Hits_toolsv2(self.collision_avoidance.initial_positions, self.collision_avoidance.positions, self.collision_avoidance.speeds, self.collision_avoidance.next_waypoints)
                result = hits_tool.hit()

                #If the difference between count and prevCount equals to 30 it means that about 3
                #seconds would have passed since the last time collisions were checked
                if len(result) != 0 and (self.count - self.collision_avoidance.prevCount >= 60 or result != self.collision_avoidance.prev_result):
                    self.collision_avoidance.prev_result = result
                    self.collision_avoidance.prevCount = self.count
                    self.get_logger().info('RESUUUULT %s' % result)
                    for e in result:
                        actualPos = self.collision_avoidance.positions[e[0]]
                        next_waypoint = self.collision_avoidance.next_waypoints[e[0]]
                        distance = np.linalg.norm(actualPos - next_waypoint)
                        if (distance < 10):
                            next_waypoint[2] += e[1]
                            self.collision_avoidance.waypoints[e[0]][self.posWps] = next_waypoint
                        else:
                            vector = next_waypoint - actualPos 
                            module = np.linalg.norm(vector)
                            vector = (vector / module) * 10
                            next_waypoint[0] =  actualPos[0] + vector[0]
                            next_waypoint[1] =  actualPos[1] + vector[1]
                            next_waypoint[2] =  actualPos[2] + e[1]
                            np.insert(self.collision_avoidance.waypoints[e[0]], self.posWps, next_waypoint)
                        self.get_logger().info('ADIOOOOS %s %s' % (next_waypoint, actualPos))
                    #     msg = Float32()
                    #     msg.data = float(e[1])
                    #     self.collision_avoidance.publishers[e[0]].publish(msg)
                    #     self.get_logger().info('Publishing: "%d"' % int(msg.data))
        elif self.count % 5 == 1:
            self.collision_avoidance.initial_positions[self.index] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    def waypoints_callback(self, msg):
        self.collision_avoidance.waypoints[self.index] = np.array(msg.wps, dtype=np.float64).reshape((int)(len(msg.wps)/3), 3)
    
    def next_waypoint(self, waypoints, position):
        finded = True
        i = 0
        j = 1
        while not finded:
                finded = self.same_line(waypoints[i], position, waypoints[j])
                i += 1
                j += 1
        return [j, waypoints[j]]
        

    def same_line(point0, point1, point2) -> bool:
        vector1 = (point1[0] - point0[0], point1[1] - point0[1], point1[2] - point0[2])
        vector2 = (point2[0] - point0[0], point2[1] - point0[1], point2[2] - point0[2])

        diff_x = vector1[1] * vector2[2] - vector1[2] * vector2[1]
        diff_y = vector1[2] * vector2[0] - vector1[0] * vector2[2]
        diff_z = vector1[0] * vector2[1] - vector1[1] * vector2[0]

        if abs(diff_x) < 0.001 and abs(diff_y) < 0.001 and abs(diff_z) < 0.001:
            return True
        return False

def main():
    rclpy.init()

    collision_avoidance = Collision_avoidance()

    rclpy.spin(collision_avoidance)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
