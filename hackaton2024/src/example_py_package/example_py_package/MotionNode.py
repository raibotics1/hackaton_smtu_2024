import rclpy
from rclpy.node import Node
from typing import Tuple
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
# pip3 install ros-<distro>-tf-transformations
from tf_transformations import euler_from_quaternion

from numpy import arctan2, rad2deg, deg2rad, array
from numpy.linalg import norm
# pip3 install simple-pid
from simple_pid import PID


def angle_diff(a1: float, a2: float) -> float:
    a = a1 - a2
    return (a + 180) % 360 - 180

def get_dist_course(delta_point: array) -> Tuple[float]:
    dist = norm(delta_point)
    # https://numpy.org/doc/stable/reference/generated/numpy.arctan2.html
    # 0 - по оси +x
    # возраставет по часовой стрелке
    course = rad2deg(arctan2(delta_point[0], delta_point[1])) - 90
    return dist, course % 360

def get_yaw_from_quaternion(q) -> int:
    return round(rad2deg(euler_from_quaternion([q.x, q.y, q.z, q.w])[2]))


class MotionNode(Node):
    def __init__(self):
        super().__init__('motion')
        
        self.__twist_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.create_subscription(Odometry, 'odom', self.__pose_callback, 1)
        self.create_subscription(PoseStamped, '/goal_pose', self.__new_goal_callback, 1)

        self.__goal     = None           # in meters
        self.__position = array([0, 0])  # in meters
        self.__heading  = 0              # in degrees

        max_clamp  = self.declare_parameter("clamp", 0.5).value
        linera_p   = self.declare_parameter("linear_p", 1).value
        linera_d   = self.declare_parameter("linear_d", 0).value
        angular_p  = self.declare_parameter("angular_p", 0.05).value
        angular_d  = self.declare_parameter("angular_d", 0).value
        self.linear_pd = PID(linera_p, 0, linera_d, output_limits=(-max_clamp, max_clamp))
        self.angular_pd = PID(angular_p, 0, angular_d, output_limits=(-max_clamp, max_clamp))

        self.DISTANCE_THRESHOLD = self.declare_parameter("distance_threshold", 0.1).value
        
        self.create_timer(1 / self.declare_parameter("hz", 10).value, self.update)
        self.log_info('Motion controller started')

    def update(self):
        if self.get_goal() is None:
            self.pub_stop()
            return

        delta = self.get_goal() - self.get_position()
        distance, course = get_dist_course(delta)
        angular_error = angle_diff(course, self.get_heading())
                
        if distance <= self.DISTANCE_THRESHOLD:
            self.achieve_goal()
            return

        if abs(angular_error) < 10:
            linear_speed = -self.linear_pd(distance)
        else:
            linear_speed = 0
        angular_speed = self.angular_pd(angular_error)
        
        self.pub_cmd_vel(lin_x=linear_speed, ang_z=angular_speed)
    
    def get_goal(self) -> array:
        return self.__goal.copy() if self.__goal is not None else None
    
    def get_position(self) -> array:
        return self.__position.copy()
    
    def get_heading(self) -> int:
        return self.__heading
    
    def achieve_goal(self) -> None:
        self.__goal = None
        self.log_info(f"goal is achieved")
    
    def log_info(self, msg: str) -> None:
        self.get_logger().info(msg)
    
    def pub_stop(self):
        self.pub_cmd_vel(0, 0, 0)

    def pub_cmd_vel(self, lin_x: float=0.0, lin_y: float=0.0, ang_z: float=0.0) -> None:
        msg = Twist()
        msg.linear.x = float(lin_x)
        msg.linear.y = float(lin_y)
        msg.angular.z = float(ang_z)
        self.__twist_publisher.publish(msg)

    def __pose_callback(self, msg: Odometry) -> None:
        self.__position = array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.__heading = -get_yaw_from_quaternion(msg.pose.pose.orientation) % 360
    
    def __new_goal_callback(self, msg: PoseStamped) -> None:
        pos = msg.pose.position
        self.__goal = array([pos.x, pos.y])
        self.log_info(f"new_goal={self.get_goal()}")


def main(args=None):
    rclpy.init(args=args)
    motion = MotionNode()
    rclpy.spin(motion)
    motion.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
