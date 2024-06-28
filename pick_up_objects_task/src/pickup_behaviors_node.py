#!/usr/bin/env python

import py_trees
import rospy
from std_srvs.srv import Trigger, TriggerRequest
import numpy as np
import py_trees.decorators
import py_trees.display
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import copy

lists_of_points = [(1.25, 0.5), (1.25, -1.25), (0.0, -1.25),
                          (-0.5, 1.25), (-1.25, 0.5)]

bb = py_trees.blackboard.Blackboard()

# Behavior for calling `check_object` task and if True, store object name to Blackboard
class CheckObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CheckObject, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            "object_name", access=py_trees.common.Access.WRITE)

    def setup(self):
        self.logger.debug("  %s [CheckObject::setup()]" % self.name)
        rospy.wait_for_service('/manage_objects/check_object')
        try:
            self.server = rospy.ServiceProxy(
                '/manage_objects/check_object', Trigger)
            self.logger.debug(
                "  %s [CheckObject::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [CheckObject::setup() ERROR!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [CheckObject::initialise()]" % self.name)

    def update(self):
        try:
            self.logger.debug(
                "  {}: call service /manage_objects/check_object".format(self.name))
            resp = self.server(TriggerRequest())
            if resp.success:
                self.blackboard.object_name = resp.message
                print("set to blackboard: ", resp.message)
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        except:
            self.logger.debug(
                "  {}: Error calling service /manage_objects/check_object".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [CheckObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

# Behavior for calling `get_object`
class GetObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(GetObject, self).__init__(name)

    def setup(self):
        self.logger.debug("  %s [GetObject::setup()]" % self.name)
        rospy.wait_for_service('/manage_objects/get_object')
        try:
            self.server = rospy.ServiceProxy(
                '/manage_objects/get_object', Trigger)
            self.logger.debug(
                "  %s [GetObject::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [GetObject::setup() ERROR!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [GetObject::initialise()]" % self.name)

    def update(self):
        try:
            self.logger.debug(
                "  {}: call service /manage_objects/get_object".format(self.name))
            resp = self.server(TriggerRequest())
            if resp.success:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        except:
            self.logger.debug(
                "  {}: Error calling service /manage_objects/get_object".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [GetObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


# Behavior for calling `let_object`
class LetObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(LetObject, self).__init__(name)

    def setup(self):
        self.logger.debug("  %s [LetObject::setup()]" % self.name)
        rospy.wait_for_service('/manage_objects/let_object')
        try:
            self.server = rospy.ServiceProxy(
                '/manage_objects/let_object', Trigger)
            self.logger.debug(
                "  %s [LetObject::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [LetObject::setup() ERROR!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [LetObject::initialise()]" % self.name)

    def update(self):
        try:
            self.logger.debug(
                "  {}: call service /manage_objects/let_object".format(self.name))
            resp = self.server(TriggerRequest())
            if resp.success:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        except:
            self.logger.debug(
                "  {}: Error calling service /manage_objects/let_object".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [LetObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

# TODO: Create any other required behavior like those to move the robot to a point, 
#       add or check elements in the blackboard, ...

# Custom Behavior class to take a location from a list and store it in the blackboard for later use.
class TakePoints(py_trees.behaviour.Behaviour):
    """
    This behavior takes a location from a predefined list and stores it in the blackboard.
    It is used to iterate through a list of points that the robot needs to visit.
    """
    def __init__(self, name):
        super(TakePoints, self).__init__(name)

    def setup(self):
        self.logger.debug("  %s [TakePoints::setup()]" % self.name)

    def initialise(self):
        global lists_of_points, bb
        self.logger.debug("  %s [TakePoints::initialise()]" % self.name)
        self.point= lists_of_points  # list of points is a global list of points.
        self.bb = copy.copy(bb)  #  bb is a global blackboard instance

    def update(self):
        """
        Takes the first location from the list and updates the blackboard with it.
        Returns SUCCESS if a location is taken, FAILURE if the list is empty.
        """
        self.logger.debug("  %s [TakePoints::update()]" % self.name)
        if len(self.point) > 0:
            pop_points = self.point.pop(0)
            self.logger.info(f"Target point {pop_points}")
            self.bb.set("object_name", pop_points)  # Update blackboard
            return py_trees.common.Status.SUCCESS
        else:
            print("All the points are checked")
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.info(f"Terminated Status {new_status}")

# Custom Behavior to move the robot to a specified point.
class MoveToPoint(py_trees.behaviour.Behaviour):
    """
    This behavior moves the robot to the goal set by the TakePoints class.
    It subscribes to the robot's odometry to keep track of its current position and
    publishes a goal to the navigation system to move the robot.
    """
    def __init__(self, name):
        super(MoveToPoint, self).__init__(name)
        self.bb = copy.copy(bb)  # bb is a global blackboard instance
        self.robot_pose = (0, 0)  # Default pose

    def setup(self):
        """
        Sets up necessary subscribers and publishers for odometry and goal setting.
        """
        self.logger.debug("  %s [MoveToPoint::setup()]" % self.name)
        self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

    def initialise(self):
        """
        Retrieves the target location from the blackboard, set by the TakePoints class.
        """
        self.logger.debug("  %s [MoveToPoint::initialise()]" % self.name)
        self.goal = self.bb.get("object_name")  # Retrieve target goal from blackboard

    def odom_callback(self, data):
        """
        Updates the robot's current odometry based on the callback data.
        """
        self.robot_pose = (data.pose.pose.position.x, data.pose.pose.position.y)

    def update(self):
        """
        Checks the robot's position against the goal and publishes the goal if not reached.
        Returns RUNNING if still moving towards the goal, SUCCESS if the goal is reached,
        or FAILURE if there is no goal.
        """
        self.logger.debug("  %s [MoveToPoint::update()]" % self.name)
        if self.goal is None:
            return py_trees.common.Status.FAILURE  # No goal to move towards
        #Convert to goal and robot pose to  NumPy arrays and ensure they are 1D (flattened)
        self.goal = np.array(self.goal).reshape(-1)
        self.robot_pose = np.array(self.robot_pose).reshape(-1)
        dis = np.linalg.norm(self.goal-self.robot_pose)  # Calculate distance to the goal
        self.logger.debug("Distance to goal: {:.2f}".format(dis))

        if dis < 0.2:  # If close enough to the goal
            return py_trees.common.Status.SUCCESS
        else:
            self.publish_goal(self.goal)  # Publish the goal to move towards
            return py_trees.common.Status.RUNNING

    def publish_goal(self, goal):
        """
        Publishes the goal to the navigation system for the robot to move towards.
        """
        goal_stamp = PoseStamped()
        goal_stamp.header.stamp = rospy.Time.now()
        goal_stamp.header.frame_id = 'odom'
        goal_stamp.pose.position.x = goal[0]
        goal_stamp.pose.position.y = goal[1]
        goal_stamp.pose.orientation.w = 1.0
        self.pub_goal.publish(goal_stamp)
        self.logger.debug("Published new goal: ({}, {})".format(goal[0], goal[1]))

    def terminate(self, new_status):
        """
        Logs termination status.
        """
        self.logger.info(f"{self.name} terminated Status {new_status}")
        
ACCEPTABLE_DISTANCE_THRESHOLD = 0.35 
# Class for verifying if the robot has reached a specified point.
class CheckPoints(py_trees.behaviour.Behaviour):
    """
    This behavior checks if the robot has reached the location specified in the blackboard.
    It subscribes to the robot's odometry data to compare its current position with the target position.
    Success is returned if the robot is within an acceptable distance threshold from the target.
    """
    def __init__(self, name):
        super(CheckPoints, self).__init__(name)
        global bb
        self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.bb = copy.copy(bb)  # Blackboard for shared data access
        self.robot_pose = None  # Initialize robot_pose
       

    def setup(self):
        """ Setup tasks before behavior execution begins. """
        self.logger.debug("  %s [CheckPoints::setup()]" % self.name)

    def initialise(self):
        """ Initial setup for the behavior, called at the start of execution. """
        self.logger.debug("  %s [CheckPoints::initialise()]" % self.name)
        
    def odom_callback(self, data):
        """ Updates the robot's current position based on odometry data. """
        self.robot_pose = (data.pose.pose.position.x, data.pose.pose.position.y)
    
    def update(self):
        """
        Compares the robot's current position with the target position.
        Returns SUCCESS if the robot is within the predefined distance threshold from the target.
        """
        self.logger.debug("  %s [CheckPoints::update()]" % self.name)
        target_pose = self.bb.get("object_name")
        if target_pose and self.robot_pose:
           #Convert to  robot pose and target pose to  NumPy arrays and ensure they are 1D (flattened)
            self.robot_pose = np.array(self.robot_pose).reshape(-1)
            target_pose = np.array(target_pose).reshape(-1)
            #Calculate distance to target
            distance_to_target = np.linalg.norm(self.robot_pose-target_pose)
            if distance_to_target < ACCEPTABLE_DISTANCE_THRESHOLD:
                return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """ Handles termination of this behavior with logging. """
        self.logger.info(f"CheckLocation Terminated Status {new_status}")


# Behavior to select a specific goal based on conditions and store it in the blackboard.
class TakeGoalPoints(py_trees.behaviour.Behaviour):
    """
    This behavior selects a specific goal location based on a condition and stores it in the blackboard.
    It is used to dynamically choose the next goal for the robot based on the current task or context.
    """
    def __init__(self, name):
        super(TakeGoalPoints, self).__init__(name)
        global bb
        self.bb = copy.copy(bb)  # Access to the blackboard
        self.flag = False  # Condition flag

    def setup(self):
        """ Setup tasks before behavior execution begins. """
        self.logger.debug("  %s [TakePoints::setup()]" % self.name)

    def initialise(self):
        """ Determines the goal based on conditions and updates the blackboard. """
        self.logger.debug("  %s [TakePoints::initialise()]" % self.name)
        target_goal = self.bb.get("object_name")
        if target_goal == "coke":
            self.location = (1.5, 1.5)  # goal for "coke"
            self.flag = True
        elif target_goal == "beer":
            self.location = (-1.5, -1.5)  # goal for "beer"
            self.flag = True

    def update(self):
        """
        Sets the determined location as the target goal in the blackboard.
        Always returns SUCCESS as it is a non-conditional setter behavior.
        """
        self.logger.debug("  %s [TakePoints::update()]" % self.name)
        if self.flag:  # If a goal has been determined
            target_location = self.location
            self.logger.info(f"Selected Goal {target_location}")
            self.bb.set("object_name", target_location)
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """ Handles termination of this behavior with logging. """
        self.logger.info(f"Terminated Status {new_status}")


def create_tree():
    global lists_of_points
    bb = py_trees.blackboard.Client()
    bb.register_key("flag", access=py_trees.common.Access.WRITE)
    bb.register_key("flag", access=py_trees.common.Access.READ)
    bb.set("flag", False)

    get_points = TakePoints("Get Points")
    check_points = CheckPoints("Check Points")
    move_to_point = MoveToPoint("Move To Point")
    check_object = CheckObject("check_object")
    get_object = GetObject("get_object")
    take_goal_points = TakeGoalPoints("Take Goal Points")
    move_to_goal = MoveToPoint("Move To Goal")
    check_goal_points = CheckPoints("Check Goal Points")
    let_object = LetObject("let_object")
    root = py_trees.composites.Sequence(name="Pick up Behavior", memory=True)
    root.add_children([get_points, move_to_point, check_points, check_object, get_object, 
                       take_goal_points, move_to_goal, check_goal_points, let_object])
    
    return root

def execute_behavior_tree(tick_count=30):
    """
    Sets up and executes the behavior tree for a specified number of ticks.
    
    :param tick_count: Number of times the tree should be ticked.
    """
    root = create_tree() 
    behavior_tree = py_trees.trees.BehaviourTree(root)
    py_trees.display.render_dot_tree(root, name="/tmp/behavior_tree")  # Saving the tree structure to a dot file

    try:
        print("Setting up for all tree children...")
        behavior_tree.setup(timeout=15)  # Setup the tree with a timeout

        # Tick the tree for the specified number of ticks or until interrupted
        for _ in range(tick_count):
            try:
                behavior_tree.tick()
                rospy.sleep(1)  # Simulate time passing (e.g., waiting for 1 second before the next tick)
            except KeyboardInterrupt:
                print("Interrupted by the user.")
                break
    except KeyboardInterrupt:
        print("Execution interrupted.")

if __name__ == "__main__":
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rospy.init_node("behavior_trees")
    execute_behavior_tree(tick_count=500)
    rospy.spin()
 