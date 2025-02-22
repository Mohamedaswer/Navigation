#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatus

# Import the custom messages from rur_navigation
from rur_navigation.msg import OrderAction, OrderFeedback, OrderResult  # Ensure these messages exist

class ButlerRobot:
    def __init__(self):
        rospy.init_node('butler_robot')
        
        # Move base action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Action server for handling orders
        self.server = actionlib.SimpleActionServer('butler_order', OrderAction, self.execute_order, False)
        self.server.start()

        # Publishers and Subscribers
        self.status_pub = rospy.Publisher('/butler/status', String, queue_size=10)
        self.confirmation_sub = rospy.Subscriber('/butler/confirmation', String, self.confirmation_callback)
        
        # Internal variables
        self.awaiting_confirmation = False
        self.timeout_duration = rospy.Duration(10)  # 10 seconds timeout
        
    def move_to_goal(self, position):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.orientation.w = 1.0
        
        self.client.send_goal(goal)
        self.client.wait_for_result()
        return self.client.get_state() == GoalStatus.SUCCEEDED
    
    def execute_order(self, goal):
        table = goal.table_number
        rospy.loginfo(f"New order received for Table {table}")
        
        # Step 1: Move to Kitchen
        if not self.move_to_goal([2.0, 2.0]):
            self.server.set_aborted(OrderResult(status="Failed to reach Kitchen"))
            return
        
        # Step 2: Wait for Kitchen Confirmation
        self.awaiting_confirmation = True
        start_time = rospy.Time.now()
        while self.awaiting_confirmation and rospy.Time.now() - start_time < self.timeout_duration:
            rospy.sleep(1)
        if self.awaiting_confirmation:
            self.server.set_aborted(OrderResult(status="Timeout at Kitchen"))
            return
        
        # Step 3: Move to Table
        if not self.move_to_goal([table * 2, 2.0]):
            self.server.set_aborted(OrderResult(status="Failed to reach Table"))
            return
        
        # Step 4: Wait for Table Confirmation
        self.awaiting_confirmation = True
        start_time = rospy.Time.now()
        while self.awaiting_confirmation and rospy.Time.now() - start_time < self.timeout_duration:
            rospy.sleep(1)
        if self.awaiting_confirmation:
            rospy.loginfo("Table did not confirm, returning to Kitchen")
            self.move_to_goal([2.0, 2.0])
        
        # Step 5: Return to Home Position
        self.move_to_goal([0.0, 0.0])
        self.server.set_succeeded(OrderResult(status="Order Completed"))
    
    def confirmation_callback(self, msg):
        rospy.loginfo(f"Received Confirmation: {msg.data}")
        self.awaiting_confirmation = False

if __name__ == '__main__':
    ButlerRobot()
    rospy.spin()

