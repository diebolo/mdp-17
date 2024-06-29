import rospy
import smach
from geometry_msgs.msg import Point
from coverage_path.srv import GeneratePath
from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalID
from std_srvs.srv import SetBool, Empty


# Define state Darp_Global
class DarpGlobal(smach.State):
    """
    This class represents the Darp_Global state in the state machine.

    It inherits from the smach.State class and defines the necessary methods
    for execution and outcome handling.

    Attributes:
        None

    Methods:
        __init__: Initializes the Darp_Global state.
        execute: Executes the Darp_Global state.

    """

    def __init__(self):
        """
        Initializes the Darp_Global state.

        Args:
            None

        Returns:
            None

        """
        smach.State.__init__(
            self, outcomes=["completed", "abort_state"], input_keys=["starting_point"]
        )

        self.finished_trajectory = False

        # Add the subscriber
        self.subscriber = rospy.Subscriber(
            "/trajectory_finished", Bool, self.handle_request_traj_finished
        )

    def handle_request_traj_finished(self, msg):
        """
        Callback function for handling the completion status of a trajectory.

        Args:
            msg (bool): The completion status of the trajectory.

        Returns:
            None

        """
        if msg.data:
            self.finished_trajectory = True
            rospy.loginfo("Callback from trajectory control -> Trajectory finished")
        else:
            rospy.loginfo("Callback from trajectory control -> Trajectory not finished")

    def execute(self, userdata):
        """
        Executes the Darp_Global state.

        This method is called when the state machine enters the Darp_Global state.
        It performs the necessary actions and returns the outcome.

        Args:
            userdata: Userdata passed to the state.

        Returns:
            str: The outcome of the state execution.

        """
        if self.preempt_requested():
            self.service_preempt()
            return "abort_state"

        self.finished_trajectory = False

        rospy.loginfo("Executing state Darp_Global")

        # Call the /area_division/divide service
        rospy.loginfo("DarpGlobal: Waiting to call /area_division/divide service")
        rospy.wait_for_service("/area_division/divide")
        try:
            divide_area = rospy.ServiceProxy("/area_division/divide", SetBool)
            resp = divide_area(True)
            if resp.success:
                rospy.loginfo(
                    "DarpGlobal: Successfully called /area_division/divide service"
                )
            else:
                rospy.logerr(
                    "DarpGlobal: /area_division/divide Service call failed: %s",
                    resp.message,
                )
                return "abort_state"

        except rospy.ServiceException as e:
            rospy.logerr("DarpGlobal: /area_division/divide Service call failed: %s", e)

        # Call the /enable_control service
        rospy.loginfo("DarpGlobal: Waiting to call /enable_control_and_wait service")
        rospy.wait_for_service("/enable_control_and_wait")
        try:
            enable_control = rospy.ServiceProxy("/enable_control_and_wait", SetBool)
            resp = enable_control(True)
            if resp.success:
                rospy.loginfo(
                    "DarpGlobal: Successfully called /enable_control_and_wait True service"
                )
            else:
                rospy.logerr(
                    "DarpGlobal: Failed to call /enable_control_and_wait service: %s",
                    resp.message,
                )
        except rospy.ServiceException as e:
            rospy.logerr("DarpGlobal: Service call failed: %s", e)

        # Publish an empty message to /move_base/cancel
        self.publisher = rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)
        self.publisher.publish(GoalID())

        rospy.sleep(2)

        point = Point(
            x=userdata.starting_point.transform.translation.x,
            y=userdata.starting_point.transform.translation.y,
            z=0.0,
        )

        generate_path = rospy.ServiceProxy("/generate_path", GeneratePath)

        # Call the service
        try:
            response = generate_path(point)
            if response.feedback:
                rospy.loginfo("Path generation successful")
            else:
                rospy.loginfo("Path generation failed")
                return "abort_state"
        except rospy.ServiceException as e:
            rospy.logerr("Generate path Service call failed: %s", e)
            return "abort_state"

        # Start darp global planner
        # Wait for service call from darp that cleaning has completed
        # Then return home
        while True:
            if self.preempt_requested():
                self.service_preempt()
                return "abort_state"

            if self.finished_trajectory:
                rospy.loginfo("Darp has completed -> Going home")
                return "completed"
