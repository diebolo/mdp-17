import rospy
import smach
import tf2_ros
import time
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String

# from state_control import state_pub
state_pub = None


def set_explore_publisher(pub):
    global state_pub
    state_pub = pub


# Define state Exploration
class Exploration(smach.State):
    """
    A state class for the exploration phase.

    This class represents a state in a state machine for performing frontier exploration.
    It provides methods for initializing the state, executing the state, and handling abort signals.

    Args:
        None

    Attributes:
        got_abort_signal (bool): Flag indicating whether an abort signal has been received.
        abort_subscriber (rospy.Subscriber): Subscriber for the abort signal topic.
        seconds (float): Time in seconds.

    Returns:
        None
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["exploration_done", "abort_state"],
            input_keys=["starting_point", "phase", "slam"],
            output_keys=["slam", "phase"],
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.seconds = time.time()

        self.starting_point_transform = None

        self.got_finished_signal = False

        # Add the service server
        self.service = rospy.Service(
            "/state_control/explore/stopped", Empty, self.handle_request_stopped
        )

        global state_pub
        state_pub.publish(String("Exploration"))

    def handle_request_stopped(self, req):
        # This method is called when a service request is received.
        # Here, you can modify the state of the state machine based on the request.
        # For example, you can set a flag that is checked in the execute() method:
        self.got_finished_signal = True
        return EmptyResponse()  # Return an appropriate response

    def explore_start_call(self):
        explore_name = "/explore/start"

        rospy.wait_for_service(explore_name)
        try:
            start_explore_service = rospy.ServiceProxy(explore_name, Empty)
            resp1 = start_explore_service()
            if resp1:
                rospy.loginfo("Frontier exploration started")
            else:
                rospy.loginfo("Failed to start frontier exploration")
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return False

    def explore_abort_call(self):
        """
        Aborts the frontier exploration.

        This method calls the '/explore/abort' service to abort the frontier exploration.
        If the service call is successful, it logs a message indicating that the exploration was aborted.
        If the service call fails, it logs a message indicating that the abort failed.

        Returns:
            bool: True if the exploration was successfully aborted, False otherwise.
        """
        explore_name = "/explore/abort"

        rospy.wait_for_service(explore_name)
        try:
            abort_explore_service = rospy.ServiceProxy(explore_name, Empty)
            resp1 = abort_explore_service()
            if resp1:
                rospy.loginfo("Frontier exploration aborted")
            else:
                rospy.loginfo("Failed to abort frontier exploration")
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return False

    def execute(self, userdata):
        """
        Execute the frontier exploration state.

        This function is called when the state is being executed.
        It performs the necessary actions for frontier exploration,
        such as sending goals and checking for abort signals.

        Args:
            userdata: Userdata passed to the state.

        Returns:
            str: The outcome of the state execution.
        """
        rospy.loginfo("Executing state Exploration")
        self.got_finished_signal = False

        # if userdata.slam.status != "mapping":
        #     if userdata.slam.start_mapping() == "slam_error":
        #         rospy.logerr("Failed to start SLAM")
        #         return "abort_state"

        userdata.phase = "start_frontier"

        self.explore_start_call()

        # Check got_abort_signal every 0.2 seconds
        start_time = rospy.Time.now()
        while True:
            if self.preempt_requested():
                self.service_preempt()
                rospy.loginfo("Exploration was preempted, aborting explore")
                self.explore_abort_call()
                rospy.loginfo("Now going home")
                return "abort_state"

            if rospy.Time.now() - start_time > rospy.Duration(120.0):
                rospy.logerr("Frontier exploration timed out 2 mins, aborting explore")

                self.explore_abort_call()

                rospy.loginfo("Now going home")

                return "abort_state"

            if self.got_finished_signal:
                rospy.loginfo("Exploration complete, going home")
                userdata.phase = "exploration_done"
                return "exploration_done"

            rospy.sleep(0.2)
