"""Small asynchronous adapter to the Humble NavigateToPose action."""
import argparse, math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class Navigator(Node):
    def __init__(self,use_sim_time=True):
        super().__init__('towergo_navigator',parameter_overrides=[Parameter('use_sim_time',value=use_sim_time)])
        self.client=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.goal_handle=None; self.result_future=None; self.feedback=None
    def send(self,x,y,yaw):
        if not all(math.isfinite(v) for v in (x,y,yaw)): raise ValueError('Goal must be finite')
        if self.goal_handle is not None and self.result_future and not self.result_future.done():
            raise RuntimeError('Cancel or complete the active goal first')
        goal=NavigateToPose.Goal(); goal.pose=PoseStamped()
        goal.pose.header.frame_id='map'; goal.pose.header.stamp=self.get_clock().now().to_msg()
        goal.pose.pose.position.x=float(x); goal.pose.pose.position.y=float(y)
        goal.pose.pose.orientation.z=math.sin(yaw/2); goal.pose.pose.orientation.w=math.cos(yaw/2)
        future=self.client.send_goal_async(goal,feedback_callback=self.on_feedback)
        future.add_done_callback(self.accepted)
        return future
    def on_feedback(self,msg): self.feedback=msg.feedback
    def accepted(self,future):
        self.goal_handle=future.result()
        if self.goal_handle.accepted: self.result_future=self.goal_handle.get_result_async()
    def cancel(self):
        if self.goal_handle and self.goal_handle.accepted: return self.goal_handle.cancel_goal_async()
        return None

def main():
    parser=argparse.ArgumentParser();parser.add_argument('x',type=float);parser.add_argument('y',type=float);parser.add_argument('yaw',type=float)
    args,rosargs=parser.parse_known_args();rclpy.init(args=rosargs);n=Navigator()
    if not n.client.wait_for_server(timeout_sec=15.): raise RuntimeError('Nav2 action unavailable')
    try:
        f=n.send(args.x,args.y,args.yaw);rclpy.spin_until_future_complete(n,f)
        # Humble schedules done callbacks after the Future becomes complete.
        # Drain that callback before accessing the accepted handle/result future.
        while rclpy.ok() and n.goal_handle is None:
            rclpy.spin_once(n,timeout_sec=0.1)
        if n.goal_handle is None or not n.goal_handle.accepted: raise RuntimeError('Goal rejected')
        rclpy.spin_until_future_complete(n,n.result_future)
        result=n.result_future.result();print('Action status:',result.status)
        if result.status!=4: raise SystemExit(1)
    except KeyboardInterrupt:
        f=n.cancel()
        if f:rclpy.spin_until_future_complete(n,f,timeout_sec=2.)
    finally:n.destroy_node();rclpy.shutdown()
