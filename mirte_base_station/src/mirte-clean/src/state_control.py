#!/usr/bin/env python3
import rospy
import smach
import tf2_ros
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty, EmptyResponse

# from shutdown_service import StateMachineShutdownService
import threading
from std_srvs.srv import Empty, EmptyResponse, SetBool
from std_msgs.msg import String

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from exploration import Exploration, set_explore_publisher
from darp import DarpGlobal
from slam import SlamWrapper


# Define state Initialize
class Initialize(smach.State):
    """
    This state represents the initialization phase of the system.
    It waits for the map to be received and checks if it was successfully received.
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["map_received", "map_loaded", "abort_state"],
            output_keys=["abort_state", "starting_point", "slam"],
            input_keys=["starting_point", "slam"],
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.got_map = False
        self.subscriber = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

    def map_callback(self, data):
        """
        Callback function for the map subscriber.
        Sets the 'got_map' flag to True when the map is received.
        """
        self.got_map = True

    def get_starting_point(self, userdata):
        # This gets the position and yaw angle of the robot in the map frame
        try:
            userdata.starting_point = self.tf_buffer.lookup_transform(
                "map", "base_link", rospy.Time(), rospy.Duration(1.0)
            )
            # # Extract translation from transform
            # position = transform.transform.translation
            # # Extract orientation from transform and convert to Euler angles
            # orientation = transform.transform.rotation
            # (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
            # return [position.x, position.y, yaw]
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as ex:
            rospy.logerr("Failed to get transform point from map to base_link: %s", ex)
            return False
        return True

    def execute(self, userdata):
        """
        Executes the Initialize state.
        Waits for 2 seconds to ensure the SLAM node is fully up.
        Returns 'map_received' outcome if the map was received,
        otherwise returns 'Abort_Initialize'.
        """

        rospy.loginfo("Executing state Initialize")

        ##### REMOVED DUE TO TIME CONSTRAINTS #####
        ##### THIS WAS USED TO USE DIFFERENT SLAM MAPPING AND LOCALIZATION #####
        # rospack = rospkg.RosPack()
        # mapping_dir = os.path.join(rospack.get_path("mirte-clean"), "./maps/latest_map.pgm")
        # rospy.loginfo(f"Dir to write to: {mapping_dir}")

        # if os.path.isfile(mapping_dir):
        #     rospy.loginfo("Map file already exists, skipping mapping")
        #     userdata.slam.start_localization()
        # else:
        #     rospy.loginfo("Starting mapping")
        #     userdata.slam.start_mapping()

        while not self.get_starting_point(userdata):
            if self.preempt_requested():
                self.service_preempt()
                return "abort_state"

        userdata.abort_state = False
        start_time = rospy.Time.now()
        while not self.got_map:
            if rospy.Time.now() - start_time > rospy.Duration(
                60.0
            ):  # Timeout of 60 seconds
                rospy.logerr("Get map timed out")
                rospy.loginfo("Aborting state machine")
                userdata.abort_state = True
                return "abort_state"

            if self.preempt_requested():
                self.service_preempt()
                return "abort_state"

            rospy.sleep(0.2)

        # if userdata.slam.status == "localization":
        #     rospy.loginfo("Map loaded, switching to DARP")
        #     return "map_loaded"

        rospy.loginfo("Got map, waiting to start front expl for 5 sec")
        rospy.sleep(5)  # Ensure Frontier expl is up

        if self.preempt_requested():
            self.service_preempt()
            return "abort_state"

        return "map_received"


# Define state Abort state
class AbortState(smach.State):
    """
    A state class that represents the abort initialization state.

    This state waits for a few seconds and then tries starting frontier exploration again.

    Args:
        None

    Returns:
        "init" (str): The outcome of the state.

    """

    def __init__(self):
        smach.State.__init__(
            self, outcomes=["return_home"], input_keys=["phase"], output_keys=["phase"]
        )

    def execute(self, userdata):
        # Returns to the abort state
        userdata.phase = "abort_process"
        rospy.loginfo("STOPPING, ABORT BECAUSE OF ERROR OR STOP CALL")

        return "return_home"


# Define state Local_Planner
class ReturnHome(smach.State):
    """
    A state class representing the return home behavior of the robot.

    This state is responsible for returning the robot to its starting location.
    """

    def __init__(self):
        # Initialize the state with the specified outcomes and input keys
        smach.State.__init__(
            self,
            outcomes=["at_home", "go_home_again"],
            input_keys=["starting_point", "phase", "slam"],
            output_keys=["slam"],
        )
        self.starting_point_transform = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def goto_start_pos(self):
        # Create an action client for the move_base action server
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        client.wait_for_server()

        # Create a goal for the move_base action
        goal = MoveBaseGoal()
        goal.target_pose.header = self.starting_point_transform.header
        goal.target_pose.pose.position = (
            self.starting_point_transform.transform.translation
        )
        goal.target_pose.pose.orientation = (
            self.starting_point_transform.transform.rotation
        )

        # Send the goal to the action server and wait for the result
        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return client.get_result()

    def is_robot_at_starting_location(self):
        # Get the current pose of the robot
        try:
            current_pose = self.tf_buffer.lookup_transform(
                "map", "base_link", rospy.Time(), rospy.Duration(1.0)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as ex:
            rospy.logerr("Failed to get transform point from map to base_link: %s", ex)
            return False

        # Calculate the distance between the current pose and the starting pose
        distance = (
            (
                current_pose.transform.translation.x
                - self.starting_point_transform.transform.translation.x
            )
            ** 2
            + (
                current_pose.transform.translation.y
                - self.starting_point_transform.transform.translation.y
            )
            ** 2
        ) ** 0.5

        # Check if the robot is at the starting location and has the same yaw pose
        if distance < 1.2:
            return True
        return False

    def execute(self, userdata):
        """
        Execute the return home behavior.

        This method is called when the state is being executed.
        It returns the outcome of the state execution.

        Args:
            userdata: Userdata passed to the state.

        Returns:
            The outcome of the state execution.
        """
        rospy.loginfo("Returning to starting location")

        # Call the /enable_control service
        rospy.wait_for_service("/enable_control")
        try:
            enable_control = rospy.ServiceProxy("/enable_control", SetBool)
            resp = enable_control(False)
            if resp.success:
                rospy.loginfo("ReturnHome: Successfully called /enable_control service")
            else:
                rospy.logerr(
                    "ReturnHome: Failed to call /enable_control service: %s",
                    resp.message,
                )
        except rospy.ServiceException as e:
            rospy.logerr("ReturnHome: Service call failed: %s", e)

        # Check if the starting point has been set
        if userdata.starting_point is None:
            raise Exception(
                "STARTING POINT HAS NOT BEEN CREATED YET, RETURN HOME CALLED"
            )

        # if userdata.slam.status != "localization":
        #     if userdata.slam.start_localization() == "slam_error":
        #         rospy.logerr("Failed to start SLAM")
        #         return "abort_state"

        self.starting_point_transform = userdata.starting_point
        self.goto_start_pos()

        rospy.loginfo("Checking if robot is back at starting location")
        while not self.is_robot_at_starting_location() and not self.preempt_requested():
            rospy.loginfo("Checking if robot is back at starting location")
            rospy.sleep(1.0)

        if self.preempt_requested():
            self.service_preempt()
            rospy.loginfo("ReturnHome preempted -> Going home AGAIN")
            return "go_home_again"

        return "at_home"


class AtHome(smach.State):
    """
    Represents the state when the robot is at the starting location.
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["start_darp", "start_explore", "go_home_again"],
            input_keys=["phase"],
        )

        service_start_frontier = rospy.Service(
            "/sm/start_frontier_exploration",
            Empty,
            self.handle_start_frontier_exploration,
        )
        service_start_darp = rospy.Service(
            "/sm/start_darp", Empty, self.handle_start_darp
        )

        self.do_state = None

    def handle_start_frontier_exploration(self, req):
        self.do_state = "e"

    def handle_start_darp(self, req):
        self.do_state = "d"

    def execute(self, userdata):
        rospy.loginfo("At starting location")
        self.do_state = None

        # Start darp global planner
        # If success return path defined
        rospy.loginfo("Waiting for A command from ui")
        while not self.preempt_requested():
            if self.do_state is not None:
                if self.do_state == "e":
                    return "start_explore"
                elif self.do_state == "d":
                    return "start_darp"

        rospy.loginfo("AtHome preempted -> Going home AGAIN")
        self.service_preempt()
        return "go_home_again"


def publish_state(sm, state_pub):
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        active_states = sm.get_active_states()
        if active_states:
            state_pub.publish(str(active_states[0]))
        rate.sleep()


def main():
    """
    Entry point of the program.

    Initializes the ROS node and creates a SMACH state machine.
    Adds states to the state machine and defines their transitions.
    Executes the SMACH plan.
    """

    # Service call callbacks in same scope as sm
    def setup_service_calls():
        rospy.loginfo("Services are set up")
        service_shutdown = rospy.Service("/sm/shutdown", Empty, handle_shutdown)

    def handle_shutdown(req):
        rospy.loginfo("Shutdown service called. Stopping state machine.")
        sm.request_preempt()
        return EmptyResponse()

    rospy.init_node("state_machine")

    setup_service_calls()

    state_pub = rospy.Publisher("/sm/state", String, queue_size=10)

    set_explore_publisher(state_pub)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["completed", "abort"])

    sm.userdata.phase = None
    sm.userdata.abort_state = None
    sm.userdata.starting_point = None  # Initialize starting_point
    sm.userdata.got_map = False
    sm.userdata.slam = SlamWrapper()

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add(
            "Initialize",
            Initialize(),
            transitions={
                "map_received": "Exploration",
                "map_loaded": "DarpGlobal",
                "abort_state": "AbortState",
            },
        )
        smach.StateMachine.add(
            "AbortState", AbortState(), transitions={"return_home": "ReturnHome"}
        )

        smach.StateMachine.add(
            "Exploration",
            Exploration(),
            transitions={"exploration_done": "ReturnHome", "abort_state": "AbortState"},
        )

        smach.StateMachine.add(
            "DarpGlobal",
            DarpGlobal(),
            transitions={"completed": "ReturnHome", "abort_state": "AbortState"},
        )

        smach.StateMachine.add(
            "ReturnHome",
            ReturnHome(),
            transitions={"at_home": "AtHome", "go_home_again": "ReturnHome"},
        )

        smach.StateMachine.add(
            "AtHome",
            AtHome(),
            transitions={
                "start_darp": "DarpGlobal",
                "start_explore": "Exploration",
                "go_home_again": "ReturnHome",
            },
        )

    # Execute SMACH plan
    # outcome = sm.execute()

    # Create and start the introspection server / debugging smach viewer
    # sis = smach_ros.IntrospectionServer('base_station_smach', sm, '/SM_ROOT')
    # sis.start()

    # Start a separate thread to handle the shutdown service
    # res = sm.execute()
    # sm_thread.start()
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    publish_state(sm, state_pub)

    smach_thread.join()
    # rospy.spin()
    # sis.stop()


if __name__ == "__main__":
    main()
