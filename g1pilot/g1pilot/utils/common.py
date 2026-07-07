from enum import IntEnum
import os
import threading


# DDS domain convention (single source of truth for sim vs. real):
#   Simulation (MuJoCo) : domain 1, loopback interface "lo"
#   Real robot          : domain 0, the physical network interface
#
# IMPORTANT: this is the *Unitree* DDS domain used by unitree_sdk2py. It must
# differ from ROS_DOMAIN_ID (the rmw_cyclonedds domain), otherwise a node that
# uses both ROS (rclpy) and the Unitree SDK opens domain N twice in the same
# process and CycloneDDS fails with "create domain error". The pairing is
# therefore mirrored between the modes (set via the Docker images/compose):
#   Sim  : ROS_DOMAIN_ID=0, Unitree domain 1 (lo)
#   Real : ROS_DOMAIN_ID=1, Unitree domain 0 (ROBOT_INTERFACE) -- the real G1
#          transmits on domain 0, that side is fixed by the robot firmware.
# Note the Unitree SDK builds its own CycloneDDS config (binds the interface
# directly) and ignores CYCLONEDDS_URI; the loopback cyclonedds.xml in the
# images only constrains the ROS graph.
SIM_DDS_DOMAIN = 1
REAL_DDS_DOMAIN = 0
SIM_DDS_INTERFACE = "lo"


def is_sim_mode() -> bool:
    """True if the G1_SIM_MODE environment variable selects simulation."""
    return os.getenv("G1_SIM_MODE", "false").lower() == "true"


def resolve_dds(interface: str):
    """
    Return the (domain_id, interface) pair to use for DDS, based on G1_SIM_MODE.

    In simulation the loopback interface and domain 1 are forced so that
    g1pilot talks to the MuJoCo bridge regardless of the configured interface.
    """
    if is_sim_mode():
        return SIM_DDS_DOMAIN, SIM_DDS_INTERFACE
    return REAL_DDS_DOMAIN, interface


def init_dds(interface: str, logger=None):
    """
    Initialise the Unitree DDS ChannelFactory for the current mode and return
    the (domain_id, interface) actually used. Centralises the sim/real switch
    so every node behaves identically.
    """
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    domain_id, dds_iface = resolve_dds(interface)
    if logger is not None:
        logger.info(f"[DDS] sim_mode={is_sim_mode()} domain={domain_id} iface={dds_iface}")
    ChannelFactoryInitialize(domain_id, dds_iface)
    return domain_id, dds_iface


class MotorState:
    def __init__(self):
        self.q = None
        self.dq = None

class DataBuffer:
    def __init__(self):
        self.data = None
        self.lock = threading.Lock()
    def GetData(self):
        with self.lock:
            return self.data
    def SetData(self, data):
        with self.lock:
            self.data = data

class G1_29_JointArmIndex(IntEnum):
    # Left arm
    kLeftShoulderPitch = 15
    kLeftShoulderRoll  = 16
    kLeftShoulderYaw   = 17
    kLeftElbow         = 18
    kLeftWristRoll     = 19
    kLeftWristPitch    = 20
    kLeftWristyaw      = 21
    # Right arm
    kRightShoulderPitch = 22
    kRightShoulderRoll  = 23
    kRightShoulderYaw   = 24
    kRightElbow         = 25
    kRightWristRoll     = 26
    kRightWristPitch    = 27
    kRightWristYaw      = 28

class G1_29_JointWristIndex(IntEnum):
    kLeftWristRoll  = 19
    kLeftWristPitch = 20
    kLeftWristyaw   = 21
    kRightWristRoll  = 26
    kRightWristPitch = 27
    kRightWristYaw   = 28

class G1_29_JointWeakIndex(IntEnum):
    kLeftAnklePitch    = 4
    kRightAnklePitch   = 10
    kLeftShoulderPitch = 15
    kLeftShoulderRoll  = 16
    kLeftShoulderYaw   = 17
    kLeftElbow         = 18
    kRightShoulderPitch = 22
    kRightShoulderRoll  = 23
    kRightShoulderYaw   = 24
    kRightElbow         = 25

class G1_29_JointIndex(IntEnum):
    # Left leg
    kLeftHipPitch   = 0
    kLeftHipRoll    = 1
    kLeftHipYaw     = 2
    kLeftKnee       = 3
    kLeftAnklePitch = 4
    kLeftAnkleRoll  = 5
    # Right leg
    kRightHipPitch   = 6
    kRightHipRoll    = 7
    kRightHipYaw     = 8
    kRightKnee       = 9
    kRightAnklePitch = 10
    kRightAnkleRoll  = 11
    # Torso
    kWaistYaw   = 12
    kWaistRoll  = 13
    kWaistPitch = 14
    # Left arm
    kLeftShoulderPitch = 15
    kLeftShoulderRoll  = 16
    kLeftShoulderYaw   = 17
    kLeftElbow         = 18
    kLeftWristRoll     = 19
    kLeftWristPitch    = 20
    kLeftWristyaw      = 21
    # Right arm
    kRightShoulderPitch = 22
    kRightShoulderRoll  = 23
    kRightShoulderYaw   = 24
    kRightElbow         = 25
    kRightWristRoll     = 26
    kRightWristPitch    = 27
    kRightWristYaw      = 28
    # not used
    kNotUsedJoint0 = 29
    kNotUsedJoint1 = 30
    kNotUsedJoint2 = 31
    kNotUsedJoint3 = 32
    kNotUsedJoint4 = 33
    kNotUsedJoint5 = 34
