from syst_msgs.srv import AdvService
from syst_msgs.msg import Waypoints, StringArray, DoubleArray
import rclpy
from rclpy.node import Node

from planning_algorithm.main import planning_algorithm
from .wps_processation_tools import *

import os

i = 0


class Station(Node):

    def __init__(self):
        super().__init__('Station')
        
        self.quantity = self.declare_parameter('drones_quantity', 0.0).get_parameter_value().double_value
        self.service_active = True
        self.wps_metadata = WPS_metadata()
        self.drones = []
        self.speeds = [0]
        self.speeds = self.speeds * int(self.quantity)  
        self.flight_height = self.declare_parameter('flight_height', 0.0).get_parameter_value().double_value
        self.srv = self.create_service(AdvService, '/advertisement_service', self.register_drone)

    def register_drone(self, request, response):

        if self.service_active:
            response.response = 1.0
            self.get_logger().info('Incoming request\ndrone_id: %s speed: %d tof: %d sweep_width: %d\ncoordx: %d, coordy: %d' % (request.drone_id, \
                            request.speed, request.tof, request.sweep_width, request.coordx, request.coordy))
            
            self.drones.append([request.coordx, request.coordy, request.sweep_width, request.speed, request.tof])
            self.wps_metadata.add_drone(Drone_initial(request.drone_id, (request.coordx, request.coordy)), request.sweep_width)
            self.get_logger().info('DRONE %s' % request.drone_id)
            self.speeds[int(request.drone_id.replace("drone_", ""))] = request.speed

            self.quantity = self.quantity - 1
            if self.quantity == 0:
                self.service_active = False
                
                publisher_drone_ids = self.create_publisher(StringArray, f'/drone_ids', 10)
                msg = StringArray()
                msg.drone_ids = self.wps_metadata.flatten_str()
                publisher_drone_ids.publish(msg)

                publisher_speeds = self.create_publisher(DoubleArray, f'/speeds', 10)
                msg2 = DoubleArray()
                msg2.speeds = self.speeds
                publisher_speeds.publish(msg2)
                
                self.publish_wps()
            return response
        response.response = 0.0
        return response
    
    def publish_wps(self):
        wps = planning_algorithm(self.drones, os.path.join(os.getcwd(), 'install/planner/share/planner/config/perimeter.yaml'), self.wps_metadata.flatten_str())

        index = 0

        drones_names = self.wps_metadata.flatten()
        proccesed_wps = process_wps(wps, self.flight_height)

        for list_array_2d in proccesed_wps:
            for array_2d in list_array_2d:
                array_to_send = np.append(np.array([[drones_names[index].coords[0], drones_names[index].coords[1], 0]]), \
                                    array_2d, axis=0)
                array_to_send = np.append(array_to_send, np.array([[drones_names[index].coords[0], \
                                    drones_names[index].coords[1], 0]]), axis=0)
                publisher = self.create_publisher(Waypoints, f'/{drones_names[index].name}/route', 10)
                msg = Waypoints()
                msg.wps = array_to_send.flatten().tolist()

                publisher.publish(msg)
                self.get_logger().info('Publishing: "%s"' % drones_names[index])
                index = index + 1

def main():
    rclpy.init()

    station = Station()

    rclpy.spin(station)

    rclpy.shutdown()

if __name__ == '__main__':
    main()