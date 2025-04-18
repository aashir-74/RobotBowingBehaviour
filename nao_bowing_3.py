from controller import Robot
import math

# configration constants

LINK_LENGTH_THIGH = 0.25  
LINK_LENGTH_SHIN  = 0.25 

# maximum vertical lowering of the hip for a full bow
DELTA_H_MAX = 0.02

# bbow target offsets for a natural and smooth bow
BOW_ANGLE_HIP   = -0.8
BOW_ANGLE_KNEE  =  0.6
BOW_ANGLE_ANKLE = -0.3

# head pitch parameters
MAX_HEAD_PITCH = 0.514872  # maximum allowed nao head angle
BOW_ANGLE_HEAD = 1.2       

# ssensor feedback correction gain for head adjustment
SENSOR_CORRECTION_GAIN = 0.5  

# motion parameters
INTERPOLATION_STEPS = 50   
HOLD_STEPS = 30            

# easing function
def smoothstep(t):
    """
    ssmoothstep function for non-linear interpolation.
    
    """
    return 3 * t**2 - 2 * t**3



# inverse kinematics and motion functions
def compute_leg_ik(bow_factor, initial_hip, initial_knee, initial_ankle):
    """
    Compute leg joint angles using a two-link planar inverse kinematics approach.
    
    """
    L1 = LINK_LENGTH_THIGH
    L2 = LINK_LENGTH_SHIN

    # effective leg length (distance from hip to ankle) given the bow.
    d = (L1 + L2) - (bow_factor * DELTA_H_MAX)

    # inverse Kinematics for a two-link chain.
    cos_theta1 = (L1**2 + d**2 - L2**2) / (2 * L1 * d)
    cos_theta1 = max(min(cos_theta1, 1.0), -1.0)
    theta1 = -math.acos(cos_theta1) 

    cos_theta2 = (L1**2 + L2**2 - d**2) / (2 * L1 * L2)
    cos_theta2 = max(min(cos_theta2, 1.0), -1.0)
    theta2 = math.pi - math.acos(cos_theta2)

    # computing ankle angle to ensure the foot remains flat
    theta3 = -(theta1 + theta2)

    # compute angles at full bow for offset calculation
    d_full = (L1 + L2) - DELTA_H_MAX
    cos_theta1_full = (L1**2 + d_full**2 - L2**2) / (2 * L1 * d_full)
    cos_theta1_full = max(min(cos_theta1_full, 1.0), -1.0)
    theta1_full = -math.acos(cos_theta1_full)
    
    cos_theta2_full = (L1**2 + L2**2 - d_full**2) / (2 * L1 * L2)
    cos_theta2_full = max(min(cos_theta2_full, 1.0), -1.0)
    theta2_full = math.pi - math.acos(cos_theta2_full)
    
    theta3_full = -(theta1_full + theta2_full)

    # determining additional offsets so that at bow_factor = 1 the angles match the targets
    hip_offset   = (initial_hip + BOW_ANGLE_HIP)   - (initial_hip + theta1_full)
    knee_offset  = (initial_knee + BOW_ANGLE_KNEE) - (initial_knee + theta2_full)
    ankle_offset = (initial_ankle + BOW_ANGLE_ANKLE) - (initial_ankle + theta3_full)

    # blending the computed IK angles with the offsets in proportion to bow_factor.
    final_hip   = initial_hip   + theta1   + hip_offset   * bow_factor
    final_knee  = initial_knee  + theta2   + knee_offset  * bow_factor
    final_ankle = initial_ankle + theta3   + ankle_offset * bow_factor

    return final_hip, final_knee, final_ankle

def update_pose(bow_factor, head_pitch_motor, lhip_pitch_motor, rhip_pitch_motor,
                lknee_pitch_motor, rknee_pitch_motor, lankle_pitch_motor, rankle_pitch_motor,
                initial_head, initial_lhip, initial_rhip, initial_lknee,
                initial_rknee, initial_lankle, initial_rankle, inertial_unit):
    """
    updates the head and leg joint positions for a given bow factor  and 
    usess sensor feedback from the imu to adjust the robo head
    """
    # computingg desired head pitch based on interpolation
    head_pitch_val = initial_head + bow_factor * BOW_ANGLE_HEAD
    head_pitch_val = min(head_pitch_val, MAX_HEAD_PITCH)

    # apply sensor feedback correction to the head pitch.
    measured_orientation = inertial_unit.getRollPitchYaw() 
    measured_pitch = measured_orientation[1]
    
    # compute error between desired and measured pitch and adjust command
    error = head_pitch_val - measured_pitch
    corrected_head = head_pitch_val + SENSOR_CORRECTION_GAIN * error
    corrected_head = min(corrected_head, MAX_HEAD_PITCH)
    head_pitch_motor.setPosition(corrected_head)



    # calculating leg joint angles using ik
    lhip, lknee, lankle = compute_leg_ik(bow_factor, initial_lhip, initial_lknee, initial_lankle)
    rhip, rknee, rankle = compute_leg_ik(bow_factor, initial_rhip, initial_rknee, initial_rankle)

    lhip_pitch_motor.setPosition(lhip)
    rhip_pitch_motor.setPosition(rhip)
    lknee_pitch_motor.setPosition(lknee)
    rknee_pitch_motor.setPosition(rknee)
    lankle_pitch_motor.setPosition(lankle)
    rankle_pitch_motor.setPosition(rankle)


def transition_to_bow(robot, time_step, head_pitch_motor, lhip_pitch_motor,
                      rhip_pitch_motor, lknee_pitch_motor, rknee_pitch_motor, lankle_pitch_motor,
                      rankle_pitch_motor, initial_head, initial_lhip, initial_rhip,
                      initial_lknee, initial_rknee, initial_lankle, initial_rankle, inertial_unit):
    """
    ggradually making the robot transition from an upright posture to the deep bow posture using a non-linear (smoothstep)interpolation
    """
    for i in range(INTERPOLATION_STEPS):
        # compuuting a linear factor and apply smoothstep.
        t = (i + 1) / INTERPOLATION_STEPS
        bow_factor = smoothstep(t)
        update_pose(bow_factor, head_pitch_motor, lhip_pitch_motor, rhip_pitch_motor,
                    lknee_pitch_motor, rknee_pitch_motor, lankle_pitch_motor, rankle_pitch_motor,
                    initial_head, initial_lhip, initial_rhip, initial_lknee,
                    initial_rknee, initial_lankle, initial_rankle, inertial_unit)
        if robot.step(time_step) == -1:
            return False
    return True


def transition_to_upright(robot, time_step, head_pitch_motor, lhip_pitch_motor,
                          rhip_pitch_motor, lknee_pitch_motor, rknee_pitch_motor, lankle_pitch_motor,
                          rankle_pitch_motor, initial_head, initial_lhip, initial_rhip,
                          initial_lknee, initial_rknee, initial_lankle, initial_rankle, inertial_unit):
    """
    gradually transitions the robot from the deep bbow posture back to the upright posture by
    using a non-linear(smoothstep) interpolation.
    """
    
    
  
    
    for i in range(INTERPOLATION_STEPS):
        t = (INTERPOLATION_STEPS - i - 1) / INTERPOLATION_STEPS
        bow_factor = smoothstep(t)
        update_pose(bow_factor, head_pitch_motor, lhip_pitch_motor, rhip_pitch_motor,
                    lknee_pitch_motor, rknee_pitch_motor, lankle_pitch_motor, rankle_pitch_motor,
                    initial_head, initial_lhip, initial_rhip, initial_lknee,
                    initial_rknee, initial_lankle, initial_rankle, inertial_unit)
        if robot.step(time_step) == -1:
            return False
    return True



def hold_pose(robot, time_step):
    """
    holding the current posture//stance for a specified number of steps
    """
    for _ in range(HOLD_STEPS):
        if robot.step(time_step) == -1:
            return False
    return True


def main():
   
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())

    # getting device handles for key joints and with error checking
    head_pitch_motor = robot.getDevice("HeadPitch")
    if head_pitch_motor is None:
        print("Error: 'HeadPitch' motor not found.")
        return

    lhip_pitch_motor = robot.getDevice("LHipPitch")
    if lhip_pitch_motor is None:
        print("Error: 'LHipPitch' motor not found.")
        return

    rhip_pitch_motor = robot.getDevice("RHipPitch")
    if rhip_pitch_motor is None:
        print("Error: 'RHipPitch' motor not found.")
        return

    lknee_pitch_motor = robot.getDevice("LKneePitch")
    if lknee_pitch_motor is None:
        print("Error: 'LKneePitch' motor not found.")
        return

    rknee_pitch_motor = robot.getDevice("RKneePitch")
    if rknee_pitch_motor is None:
        print("Error: 'RKneePitch' motor not found.")
        return

    lankle_pitch_motor = robot.getDevice("LAnklePitch")
    if lankle_pitch_motor is None:
        print("Error: 'LAnklePitch' motor not found.")
        return

    rankle_pitch_motor = robot.getDevice("RAnklePitch")
    if rankle_pitch_motor is None:
        print("Error: 'RAnklePitch' motor not found.")
        return

    # getting and enable the imu 
    inertial_unit = robot.getDevice("inertial unit")
    if inertial_unit is None:
        print("Error: 'inertial unit' sensor not found.")
        return
    inertial_unit.enable(time_step)


    # saves initial positions for all joints.
    initial_head   = head_pitch_motor.getTargetPosition()
    initial_lhip   = lhip_pitch_motor.getTargetPosition()
    initial_rhip   = rhip_pitch_motor.getTargetPosition()
    initial_lknee  = lknee_pitch_motor.getTargetPosition()
    initial_rknee  = rknee_pitch_motor.getTargetPosition()
    initial_lankle = lankle_pitch_motor.getTargetPosition()
    initial_rankle = rankle_pitch_motor.getTargetPosition()


    # maiin control loop for continuous bowing motion
    while robot.step(time_step) != -1:
        if not transition_to_bow(robot, time_step, head_pitch_motor, lhip_pitch_motor, rhip_pitch_motor,
                                 lknee_pitch_motor, rknee_pitch_motor, lankle_pitch_motor, rankle_pitch_motor,
                                 initial_head, initial_lhip, initial_rhip, initial_lknee,
                                 initial_rknee, initial_lankle, initial_rankle, inertial_unit):
            return

        if not hold_pose(robot, time_step):
            return

        if not transition_to_upright(robot, time_step, head_pitch_motor, lhip_pitch_motor, rhip_pitch_motor,
                                     lknee_pitch_motor, rknee_pitch_motor, lankle_pitch_motor, rankle_pitch_motor,
                                     initial_head, initial_lhip, initial_rhip, initial_lknee,
                                     initial_rknee, initial_lankle, initial_rankle, inertial_unit):
            return

        if not hold_pose(robot, time_step):
            return



if __name__ == "__main__":
    main()
