import rospy
import smach
import smach_ros
from std_msgs.msg import String
import threading


class StateOne(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["outcome1"])

    def execute(self, userdata):
        rospy.loginfo("Executing state ONE")
        return "outcome1"


class StateTwo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["outcome2"])

    def execute(self, userdata):
        rospy.loginfo("Executing state TWO")
        return "outcome2"


def monitor_state_machine(sm, state_pub):
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        active_states = sm.get_active_states()
        if active_states:
            state_pub.publish(str(active_states[0]))
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("smach_state_publisher")

    sm = smach.StateMachine(outcomes=["outcome4"])

    with sm:
        smach.StateMachine.add(
            "STATE_ONE", StateOne(), transitions={"outcome1": "STATE_TWO"}
        )
        smach.StateMachine.add(
            "STATE_TWO", StateTwo(), transitions={"outcome2": "STATE_ONE"}
        )

    state_pub = rospy.Publisher("/smach/state", String, queue_size=10)

    sis = smach_ros.IntrospectionServer("server_name", sm, "/SM_ROOT")
    sis.start()

    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    monitor_state_machine(sm, state_pub)

    smach_thread.join()
    sis.stop()
