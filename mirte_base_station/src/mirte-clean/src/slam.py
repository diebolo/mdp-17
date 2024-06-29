import rospy
import subprocess
import rospkg
import os
from slam_toolbox_msgs.srv import SerializePoseGraph


class SlamWrapper:
    """
    A class that wraps the SLAM functionality.
    REMOVED DUE TO TIME LIMITATIONS, BUT IS READY TO BE IMPLEMENTED

    Attributes:
        localization_proc (subprocess.Popen): The localization process.
        mapping_proc (subprocess.Popen): The mapping process.
        localization_status (bool): The status of the localization process.
        mapping_status (bool): The status of the mapping process.
        status (str): The current status of the SLAM wrapper.

    Methods:
        start_localization: Starts the localization process.
        stop_localization: Stops the localization process.
        start_mapping: Starts the mapping process.
        stop_mapping: Serializes the map and stops the mapping process.
    """

    def __init__(self):
        """Initializes the SLAM wrapper"""
        self.localization_proc = None
        self.mapping_proc = None

        self.localization_status = False
        self.mapping_status = False
        self.status = "idle"

    def start_localization(self):
        """Starts the SLAM process"""
        return "localization_started"

        if self.mapping_status:
            self.stop_mapping()

        try:
            self.localization_proc = subprocess.Popen(
                ["roslaunch", "mirte-clean", "activate_localization.launch"]
            )
            self.localization_status = True
            self.status = "localization"
            return "localization_started"
        except rospy.ROSInterruptException:
            return "slam_error"

    def stop_localization(self):
        """Shuts down SLAM"""
        return "localization_stopped"

        try:
            # Stop the SLAM process
            self.localization_proc.terminate()
            self.localization_proc.wait()
            self.localization_status = False
            self.status = "idle"
            return "localization_stopped"
        except rospy.ROSInterruptException:
            return "slam_error"

    def start_mapping(self):
        """Starts the SLAM process"""
        if self.localization_status:
            self.stop_localization()

        try:
            self.mapping_proc = subprocess.Popen(
                ["roslaunch", "mirte-clean", "activate_mapping.launch"]
            )
            self.mapping_status = True
            self.status = "mapping"
            return "mapping_started"
        except rospy.ROSInterruptException:
            return "slam_error"

    def stop_mapping(self):
        """Serializes the map (saves) and then shuts down SLAM"""
        return "mapping_stopped"

        slam_service_name = "/slam_toolbox/serialize_map"
        rospy.wait_for_service(slam_service_name)
        try:
            # Get own package path
            rospack = rospkg.RosPack()
            mapping_dir = os.path.join(
                rospack.get_path("mirte-clean"), "/maps/latest_map"
            )

            # Serialize the map
            slam_service = rospy.ServiceProxy(slam_service_name, SerializePoseGraph)
            slam_service(mapping_dir)
        except rospy.ROSException as e:
            rospy.logerr("Service call failed: %s" % e)
            return "slam_error"

        try:
            # Stop the SLAM process
            self.mapping_proc.terminate()
            self.mapping_proc.wait()
            self.mapping_status = False
            self.status = "idle"
            return "mapping_stopped"
        except rospy.ROSInterruptException:
            return "slam_error"
