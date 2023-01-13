import rclpy
from rclpy.node import Node
import sys
import threading
from fastapi import FastAPI
import uvicorn
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose, FollowWaypoints, ComputePathToPose, ComputePathThroughPoses
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from  typing  import  Optional 
from  pydantic  import  BaseModel 

from std_msgs.msg import String

app = FastAPI()

class  Response ( BaseModel ): 
    msg :  str 


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_handle = None
        self.result_future = None

        @app.get( '/publish', response_model = Response ) 
        async  def  publish ():
            while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
                print("'NavigateToPose' action server not available, waiting...")
            goal_msg = NavigateToPose.Goal()

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = 4.0
            goal_pose.pose.position.y = -2.0
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.x = 0.0
            goal_pose.pose.orientation.y = 0.0
            goal_pose.pose.orientation.z = 0.0
            goal_pose.pose.orientation.w = 1.0


            goal_msg.pose = goal_pose

            print('Navigating to goal: ' + str(goal_pose.pose.position.x) + ' ' +
                        str(goal_pose.pose.position.y) + '...')
            send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                    self._feedbackCallback)
            rclpy.spin_until_future_complete(self, send_goal_future)
            self.goal_handle = send_goal_future.result()

            if not self.goal_handle.accepted:
                print('Goal to ' + str(goal_pose.pose.position.x) + ' ' +
                            str(goal_pose.pose.position.y) + ' was rejected!')
                return False

            self.result_future = self.goal_handle.get_result_async()

            response  =  {  
                'msg' :  '' 
            } 
            response [ 'msg' ]  =  "Message published" 
            return  response 

    def _feedbackCallback(self, msg):
        self.debug('Received action feedback message')
        self.feedback = msg.feedback
        return
        

def main(args=None):
    rclpy.init()
    minimal_publisher = MinimalPublisher()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(minimal_publisher)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    uvicorn.run(app, port=5000, log_level='warning')
    rclpy.shutdown()

if __name__ == '__main__':
    main()