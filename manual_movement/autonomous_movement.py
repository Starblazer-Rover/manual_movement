import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray

import math
import serial


class AutonomousMovement(Node):

    def __init__(self):
        super().__init__('autonomous_node')

        self.costmap_subscriber = self.create_subscription(OccupancyGrid, '/map/grid', self.costmap_callback, 1)
        self.pose_subscriber = self.create_subscription(PoseStamped, '/odom/target_point', self.pose_callback, 1)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        self.vector_publisher = self.create_publisher(Float32MultiArray, '/movement/vector', 10)

        #self.ser = serial.Serial('/dev/ttyACM2', 115200)

        self.costmap = None
        self.pose = None

        self.attempts = 0

        self.start_x = None
        self.start_y = None

        self.collision = [91, 92, 93, 94, 106, 107, 108, 109, 121, 122, 123, 124]

        self.states = {"Waiting": 0,
                       "Arrived?": 1,
                       "Aligned?": 2,
                       "Obstacle?": 3,
                       "Repositioned?": 4,
                       "Failed": 5,
                       "Finished": 6,
                       "Testing": 7
                       }
        self.state = 7
        

    def calculate_angle(self, odom, pose):
        odom_x = odom.pose.pose.position.x
        odom_y = odom.pose.pose.position.y

        pose_x = pose.pose.position.x
        pose_y = pose.pose.position.y

        delta_x = pose_x - odom_x
        delta_y = pose_y - odom_y

        theta_radians = math.atan2(delta_y, delta_x)

        theta_degrees = math.degrees(theta_radians)

        return theta_degrees
    
    def calculate_distance(self, odom, target):
        current_x = odom.pose.pose.position.x
        current_y = odom.pose.pose.position.y

        target_x = target[0]
        target_y = target[1]

        delta_x = target_x - current_x
        delta_y = target_y - current_y  

        distance = math.sqrt((delta_x ** 2) + (delta_y ** 2)) 

        return distance
    
    def convert_angle(self, odom):
        w = odom.pose.pose.orientation.w
        x = odom.pose.pose.orientation.x
        y = odom.pose.pose.orientation.y
        z = odom.pose.pose.orientation.z

        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        yaw = math.degrees(yaw) % 360

        return yaw

    def costmap_callback(self, msg):
        self.costmap = msg
        self.costmap_waiting = False

    def pose_callback(self, msg):
        self.pose = msg

    def set_led_red(self):
        self.ser.write('1\n'.encode())

    def set_led_green(self):
        self.ser.write('2\n'.encode())
    
    def set_led_off(self):
        self.ser.write('0\n'.encode())

    def turn_right(self, duration):
        print("Turning Right")

        msg = Float32MultiArray()
        data = [10.0, -10.0]
        msg.data = data
        self.vector_publisher.publish(msg)

        self.wait(duration)

        msg.data = [0.0, 0.0]
        self.vector_publisher.publish(msg)

        print("Stopping")

        return

    def turn_left(self, duration):
        print("Turning left")

        msg = Float32MultiArray()
        data = [-10.0, 10.0]
        msg.data = data
        self.vector_publisher.publish(msg)

        self.wait(duration)

        msg.data = [0.0, 0.0]
        self.vector_publisher.publish(msg)

        print("Stopping")

        return
    
    def forward(self, duration):
        print("Moving Forward")
        
        msg = Float32MultiArray()
        data = [10.0, 10.0]
        msg.data = data
        self.vector_publisher.publish(msg)

        self.wait(duration)

        msg.data = [0.0, 0.0]
        self.vector_publisher.publish(msg)

        print('Stopping')

        return
    
    def wait(self, duration):
        start_time = Clock().now().to_msg()

        start_time = start_time.sec + (start_time.nanosec / 1000000000)

        end_time = start_time

        while end_time - start_time <= duration:
            end_time = Clock().now().to_msg()
            end_time = end_time.sec + (end_time.nanosec / 1000000000)

        return

    def orient_angle(self, current_angle, target_angle):
        difference = target_angle - current_angle

        if difference < 0:
            self.turn_right(1)
        else:
            self.turn_left(1)

    def is_occupied(self):
        for i in self.collision:
            if self.costmap[i] > 0:
                return True

        return False

    def odometry_callback(self, odom):
        match self.state:
            case 0:
                print('State: Waiting')
                if self.costmap != None and self.pose != None:
                    print('Finished Waiting')
                    self.state = self.states["Arrived?"]
                    self.set_led_red()

                return
            case 1:
                print('State: Arrived?')
                target = (self.pose.pose.position.x, self.pose.pose.position.y)

                distance = self.calculate_distance(odom, target)

                if distance <= 50:
                    print('Arrived')
                    self.state = self.states["Finished"]
                else:
                    print('Not Arrived')
                    self.state = self.states["Aligned?"]

                return
            case 2:
                print('State: Aligned?')
                current_angle = self.convert_angle(odom)
                target_angle = self.calculate_angle(odom, self.pose)

                if abs(target_angle - current_angle) < 5:
                    print('Aligned')
                    self.state = self.states["Obstacle?"]
                else:
                    print('Not Aligned')
                    self.orient_angle(current_angle, target_angle)

                return
            case 3:
                print('State: Obstacle?')
                if self.is_occupied():
                    print('Obstacle Seen')
                    self.state = self.states["Repositioned?"]
                else:
                    print('Obstacle not Seen')
                    self.forward(3)
                    self.state = self.states["Arrived?"]

                return
            case 4:
                print('State: Repositioned?')

                if self.attempts < 3:
                    print('Repositioning Right')
                    self.turn_right(0.5)

                    if not self.is_occupied():
                        self.forward(3)
                        self.orient_angle(current_angle, target_angle)
                        self.attempts += 1
                elif self.attempts < 6:
                    print('Repositioning Left')
                    self.turn_left(3)

                    if not self.is_occupied():
                        self.forward(3)
                        self.orient_angle(current_angle, target_angle)
                        self.attempts += 1
                else:
                    print('Failed to Reposition')
                    self.state = self.states["Failed"]

                if not self.is_occupied():
                    print('Repositioning Successful')
                    self.state = self.states["Arrived?"]

                return
            case 5:
                print('Failed')
                self.set_led_off()

                return
            case 6:
                print('Finished')
                self.set_led_green()

                return
            case 7:
                print('Testing')
                
                self.turn_right(3)
                self.state = self.states['Failed']

                return


def main(args=None):
    rclpy.init(args=args)

    autonomous_publisher = AutonomousMovement()

    try:
        rclpy.spin(autonomous_publisher)
    except KeyboardInterrupt:
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0]
        autonomous_publisher.vector_publisher.publish(msg)
    finally:
        autonomous_publisher.destroy_node()


if __name__ == '__main__':
    main()




