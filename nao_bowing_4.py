from controller import Robot
import math

#==configration Constants==

#link lenggths for the two-link leg model (meters)
LINK_LENGTH_THIGH = 0.25  
LINK_LENGTH_SHIN  = 0.25  

#maximum vertical lowerring of the hip for a full bow
DELTA_H_MAX = 0.02

#bow target offsets for a natural bow gesture for a legs
BOW_ANGLE_HIP   = -0.8
BOW_ANGLE_KNEE  =  0.6
BOW_ANGLE_ANKLE = -0.3

#head pitch parameters
MAX_HEAD_PITCH = 0.514872  
BOW_ANGLE_HEAD = 1.2       

#sensor feedback correctiion gain for head pitch adjustment
SENSOR_CORRECTION_GAIN = 0.5 

#motion parameters
INTERPOLATION_STEPS = 50 
HOLD_STEPS = 30            

# arm (full body) bowing offsetts
BOW_OFFSET_L_SHOULDER_PITCH =  0.3
BOW_OFFSET_R_SHOULDER_PITCH =  0.3
BOW_OFFSET_L_SHOULDER_ROLL  = -0.2
BOW_OFFSET_R_SHOULDER_ROLL  =  0.2
BOW_OFFSET_LELBOYAW         =  0.0 
BOW_OFFSET_LELBOROLL        = -0.5
BOW_OFFSET_RELBOWAY         =  0.0 
BOW_OFFSET_RELBOROLL        = -0.5

#side step parameters
SIDE_STEP_ANGLE = 0.15 
SIDE_STEP_STEPS = 50   

#easing funcctions
def smoothstep(t):
    """
    smoothstep non-linear interpolation
    """
    return 3 * t**2 - 2 * t**3

def ease_in_out_sine(t):
    """
    sinusoidal easing function providingg smooth acceleration and deceleration
    """
    return 0.5 * (1 - math.cos(math.pi * t))

#motor command clamping
def clamp_motor(motor, value):
    """
    clamps the requested motor value to the maximmum allowed range of the motor
    """
    min_val = motor.getMinPosition()
    max_val = motor.getMaxPosition()
    return max(min_val, min(value, max_val))

# ik for legs
def compute_leg_ik(bow_factor, initial_hip, initial_knee, initial_ankle):
    """
    Computing leg joint angles using a two link planar iinverse kinematics approach
    """
    L1 = LINK_LENGTH_THIGH
    L2 = LINK_LENGTH_SHIN

    #effective leg length given the bow
    d = (L1 + L2) - (bow_factor * DELTA_H_MAX)

    #inverse kinematics calculation:
    cos_theta1 = (L1**2 + d**2 - L2**2) / (2 * L1 * d)
    cos_theta1 = max(min(cos_theta1, 1.0), -1.0)
    theta1 = -math.acos(cos_theta1)  

    cos_theta2 = (L1**2 + L2**2 - d**2) / (2 * L1 * L2)
    cos_theta2 = max(min(cos_theta2, 1.0), -1.0)
    theta2 = math.pi - math.acos(cos_theta2)

    theta3 = -(theta1 + theta2) 

    #calculates angles at full bow for determinig offsets
    d_full = (L1 + L2) - DELTA_H_MAX
    cos_theta1_full = (L1**2 + d_full**2 - L2**2) / (2 * L1 * d_full)
    cos_theta1_full = max(min(cos_theta1_full, 1.0), -1.0)
    theta1_full = -math.acos(cos_theta1_full)
    
    cos_theta2_full = (L1**2 + L2**2 - d_full**2) / (2 * L1 * L2)
    cos_theta2_full = max(min(cos_theta2_full, 1.0), -1.0)
    theta2_full = math.pi - math.acos(cos_theta2_full)
    
    theta3_full = -(theta1_full + theta2_full)

    #offsets to match target bow angles at full bow
    hip_offset   = (initial_hip + BOW_ANGLE_HIP)   - (initial_hip + theta1_full)
    knee_offset  = (initial_knee + BOW_ANGLE_KNEE) - (initial_knee + theta2_full)
    ankle_offset = (initial_ankle + BOW_ANGLE_ANKLE) - (initial_ankle + theta3_full)

    #blending computed ik angles with offsets by bow factor
    final_hip   = initial_hip   + theta1   + hip_offset   * bow_factor
    final_knee  = initial_knee  + theta2   + knee_offset  * bow_factor
    final_ankle = initial_ankle + theta3   + ankle_offset * bow_factor

    return final_hip, final_knee, final_ankle

#arm pose update function
def update_arm_pose(bow_factor,
                    l_shoulder_pitch, l_shoulder_roll, l_elbow_yaw, l_elbow_roll,
                    r_shoulder_pitch, r_shoulder_roll, r_elbow_yaw, r_elbow_roll,
                    init_l_shoulder_pitch, init_l_shoulder_roll, init_l_elbow_yaw, init_l_elbow_roll,
                    init_r_shoulder_pitch, init_r_shoulder_roll, init_r_elbow_yaw, init_r_elbow_roll):
    """
    interpolates arm joint positions from initial values to target offsets
    """
    target = init_l_shoulder_pitch + BOW_OFFSET_L_SHOULDER_PITCH * bow_factor
    l_shoulder_pitch.setPosition(clamp_motor(l_shoulder_pitch, target))
    
    target = init_l_shoulder_roll + BOW_OFFSET_L_SHOULDER_ROLL * bow_factor
    l_shoulder_roll.setPosition(clamp_motor(l_shoulder_roll, target))
    
    target = init_l_elbow_yaw + BOW_OFFSET_LELBOYAW * bow_factor
    l_elbow_yaw.setPosition(clamp_motor(l_elbow_yaw, target))
    
    target = init_l_elbow_roll + BOW_OFFSET_LELBOROLL * bow_factor
    l_elbow_roll.setPosition(clamp_motor(l_elbow_roll, target))

    target = init_r_shoulder_pitch + BOW_OFFSET_R_SHOULDER_PITCH * bow_factor
    r_shoulder_pitch.setPosition(clamp_motor(r_shoulder_pitch, target))
    
    target = init_r_shoulder_roll + BOW_OFFSET_R_SHOULDER_ROLL * bow_factor
    r_shoulder_roll.setPosition(clamp_motor(r_shoulder_roll, target))
    
    target = init_r_elbow_yaw + BOW_OFFSET_RELBOWAY * bow_factor
    r_elbow_yaw.setPosition(clamp_motor(r_elbow_yaw, target))
    
    target = init_r_elbow_roll + BOW_OFFSET_RELBOROLL * bow_factor
    r_elbow_roll.setPosition(clamp_motor(r_elbow_roll, target))

#unified pose update function
def update_pose(bow_factor, head_pitch_motor, lhip_pitch_motor, rhip_pitch_motor,
                lknee_pitch_motor, rknee_pitch_motor, lankle_pitch_motor, rankle_pitch_motor,
                initial_head, initial_lhip, initial_rhip, initial_lknee,
                initial_rknee, initial_lankle, initial_rankle, inertial_unit,
                l_shoulder_pitch, l_shoulder_roll, l_elbow_yaw, l_elbow_roll,
                r_shoulder_pitch, r_shoulder_roll, r_elbow_yaw, r_elbow_roll,
                init_l_shoulder_pitch, init_l_shoulder_roll, init_l_elbow_yaw, init_l_elbow_roll,
                init_r_shoulder_pitch, init_r_shoulder_roll, init_r_elbow_yaw, init_r_elbow_roll):
    """
    updates head, leg, and arm joint positions for the specified bow factor
    sensor feedback for head pitch
    """
    #update head with sensor feedback
    desired_head = initial_head + bow_factor * BOW_ANGLE_HEAD
    desired_head = min(desired_head, MAX_HEAD_PITCH)
    
    measured_pitch = inertial_unit.getRollPitchYaw()[1]
    error = desired_head - measured_pitch
    corrected_head = desired_head + SENSOR_CORRECTION_GAIN * error
    corrected_head = min(corrected_head, MAX_HEAD_PITCH)
    head_pitch_motor.setPosition(corrected_head)
    
    #update leg joints using inverse kinematics
    lhip, lknee, lankle = compute_leg_ik(bow_factor, initial_lhip, initial_lknee, initial_lankle)
    rhip, rknee, rankle = compute_leg_ik(bow_factor, initial_rhip, initial_rknee, initial_rankle)
    
    lhip = clamp_motor(lhip_pitch_motor, lhip)
    rhip = clamp_motor(rhip_pitch_motor, rhip)
    lknee = clamp_motor(lknee_pitch_motor, lknee)
    rknee = clamp_motor(rknee_pitch_motor, rknee)
    lankle = clamp_motor(lankle_pitch_motor, lankle)
    rankle = clamp_motor(rankle_pitch_motor, rankle)
    
    lhip_pitch_motor.setPosition(lhip)
    rhip_pitch_motor.setPosition(rhip)
    lknee_pitch_motor.setPosition(lknee)
    rknee_pitch_motor.setPosition(rknee)
    lankle_pitch_motor.setPosition(lankle)
    rankle_pitch_motor.setPosition(rankle)
    
    #upddating arm joints
    update_arm_pose(bow_factor,
                    l_shoulder_pitch, l_shoulder_roll, l_elbow_yaw, l_elbow_roll,
                    r_shoulder_pitch, r_shoulder_roll, r_elbow_yaw, r_elbow_roll,
                    init_l_shoulder_pitch, init_l_shoulder_roll, init_l_elbow_yaw, init_l_elbow_roll,
                    init_r_shoulder_pitch, init_r_shoulder_roll, init_r_elbow_yaw, init_r_elbow_roll)

#transition functions
def transition_to_bow(robot, time_step,
                      head_pitch_motor, lhip_pitch_motor, rhip_pitch_motor,
                      lknee_pitch_motor, rknee_pitch_motor, lankle_pitch_motor, rankle_pitch_motor,
                      initial_head, initial_lhip, initial_rhip, initial_lknee,
                      initial_rknee, initial_lankle, initial_rankle,
                      inertial_unit,
                      l_shoulder_pitch, l_shoulder_roll, l_elbow_yaw, l_elbow_roll,
                      r_shoulder_pitch, r_shoulder_roll, r_elbow_yaw, r_elbow_roll,
                      init_l_shoulder_pitch, init_l_shoulder_roll, init_l_elbow_yaw, init_l_elbow_roll,
                      init_r_shoulder_pitch, init_r_shoulder_roll, init_r_elbow_yaw, init_r_elbow_roll):
    """
    gradually and smothly transitions from the upright posture to the deep bow posture
    """
    for i in range(INTERPOLATION_STEPS):
        t = (i + 1) / INTERPOLATION_STEPS
        bow_factor = smoothstep(t)
        update_pose(bow_factor,
                    head_pitch_motor, lhip_pitch_motor, rhip_pitch_motor,
                    lknee_pitch_motor, rknee_pitch_motor, lankle_pitch_motor, rankle_pitch_motor,
                    initial_head, initial_lhip, initial_rhip, initial_lknee,
                    initial_rknee, initial_lankle, initial_rankle, inertial_unit,
                    l_shoulder_pitch, l_shoulder_roll, l_elbow_yaw, l_elbow_roll,
                    r_shoulder_pitch, r_shoulder_roll, r_elbow_yaw, r_elbow_roll,
                    init_l_shoulder_pitch, init_l_shoulder_roll, init_l_elbow_yaw, init_l_elbow_roll,
                    init_r_shoulder_pitch, init_r_shoulder_roll, init_r_elbow_yaw, init_r_elbow_roll)
        if robot.step(time_step) == -1:
            return False
    return True

def transition_to_upright(robot, time_step,
                          head_pitch_motor, lhip_pitch_motor, rhip_pitch_motor,
                          lknee_pitch_motor, rknee_pitch_motor, lankle_pitch_motor, rankle_pitch_motor,
                          initial_head, initial_lhip, initial_rhip, initial_lknee,
                          initial_rknee, initial_lankle, initial_rankle,
                          inertial_unit,
                          l_shoulder_pitch, l_shoulder_roll, l_elbow_yaw, l_elbow_roll,
                          r_shoulder_pitch, r_shoulder_roll, r_elbow_yaw, r_elbow_roll,
                          init_l_shoulder_pitch, init_l_shoulder_roll, init_l_elbow_yaw, init_l_elbow_roll,
                          init_r_shoulder_pitch, init_r_shoulder_roll, init_r_elbow_yaw, init_r_elbow_roll):
    """
    smoothly aand gradually transitions from the deep bow posture back to the upright posture
    """
    for i in range(INTERPOLATION_STEPS):
        t = (INTERPOLATION_STEPS - i - 1) / INTERPOLATION_STEPS
        bow_factor = smoothstep(t)
        update_pose(bow_factor,
                    head_pitch_motor, lhip_pitch_motor, rhip_pitch_motor,
                    lknee_pitch_motor, rknee_pitch_motor, lankle_pitch_motor, rankle_pitch_motor,
                    initial_head, initial_lhip, initial_rhip, initial_lknee,
                    initial_rknee, initial_lankle, initial_rankle, inertial_unit,
                    l_shoulder_pitch, l_shoulder_roll, l_elbow_yaw, l_elbow_roll,
                    r_shoulder_pitch, r_shoulder_roll, r_elbow_yaw, r_elbow_roll,
                    init_l_shoulder_pitch, init_l_shoulder_roll, init_l_elbow_yaw, init_l_elbow_roll,
                    init_r_shoulder_pitch, init_r_shoulder_roll, init_r_elbow_yaw, init_r_elbow_roll)
        if robot.step(time_step) == -1:
            return False
    return True

def hold_pose(robot, time_step):
    """
    holds the  current posture for a given nummber of steps
    """
    for _ in range(HOLD_STEPS):
        if robot.step(time_step) == -1:
            return False
    return True

#improved nonlinear side step trransitions using sinusoidal easing
def transition_to_side_step(robot, time_step, lhip_roll_motor, rhip_roll_motor,
                              initial_lhip_roll, initial_rhip_roll, side_step_angle):
    """
    gradually shifts the hip roll positions to perform a left side step
    using a sinusoidal ease‑in‑out profile forr smooth acceleration and deceleration
    """
    for i in range(SIDE_STEP_STEPS):
        t = (i + 1) / SIDE_STEP_STEPS
        s = ease_in_out_sine(t)
        target_lhip_roll = initial_lhip_roll + side_step_angle
        target_rhip_roll = initial_rhip_roll - side_step_angle
        current_lhip_roll = initial_lhip_roll + (target_lhip_roll - initial_lhip_roll) * s
        current_rhip_roll = initial_rhip_roll + (target_rhip_roll - initial_rhip_roll) * s
        lhip_roll_motor.setPosition(current_lhip_roll)
        rhip_roll_motor.setPosition(current_rhip_roll)
        if robot.step(time_step) == -1:
            return False
    return True

def transition_to_side_step_back(robot, time_step, lhip_roll_motor, rhip_roll_motor,
                                   initial_lhip_roll, initial_rhip_roll, side_step_angle):
    """
    smoothly returns the hip roll positions from the side step back to upright
    using a sinusoidal ease-in-out profile
    """
    for i in range(SIDE_STEP_STEPS):
        t = (i + 1) / SIDE_STEP_STEPS
        s = ease_in_out_sine(t)
        target_lhip_roll = initial_lhip_roll + side_step_angle
        target_rhip_roll = initial_rhip_roll - side_step_angle
        current_lhip_roll = target_lhip_roll - (target_lhip_roll - initial_lhip_roll) * s
        current_rhip_roll = target_rhip_roll - (target_rhip_roll - initial_rhip_roll) * s
        lhip_roll_motor.setPosition(current_lhip_roll)
        rhip_roll_motor.setPosition(current_rhip_roll)
        if robot.step(time_step) == -1:
            return False
    return True

#main control loop
def main():
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())

    #get leg and head device handles
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

    # get and enable inertial unit
    inertial_unit = robot.getDevice("inertial unit")
    if inertial_unit is None:
        print("Error: 'inertial unit' sensor not found.")
        return
    inertial_unit.enable(time_step)

    #retrieve arrm device handles
    l_shoulder_pitch_motor = robot.getDevice("LShoulderPitch")
    l_shoulder_roll_motor  = robot.getDevice("LShoulderRoll")
    l_elbow_yaw_motor      = robot.getDevice("LElbowYaw")
    l_elbow_roll_motor     = robot.getDevice("LElbowRoll")

    r_shoulder_pitch_motor = robot.getDevice("RShoulderPitch")
    r_shoulder_roll_motor  = robot.getDevice("RShoulderRoll")
    r_elbow_yaw_motor      = robot.getDevice("RElbowYaw")
    r_elbow_roll_motor     = robot.getDevice("RElbowRoll")

    if (l_shoulder_pitch_motor is None or l_shoulder_roll_motor is None or
        l_elbow_yaw_motor is None or l_elbow_roll_motor is None or
        r_shoulder_pitch_motor is None or r_shoulder_roll_motor is None or
        r_elbow_yaw_motor is None or r_elbow_roll_motor is None):
        print("Error: One or more arm motors not found.")
        return

    #retrieve hip roll motors for side step
    lhip_roll_motor = robot.getDevice("LHipRoll")
    if lhip_roll_motor is None:
        print("Error: 'LHipRoll' motor not found.")
        return

    rhip_roll_motor = robot.getDevice("RHipRoll")
    if rhip_roll_motor is None:
        print("Error: 'RHipRoll' motor not found.")
        return

    #record initial upright   ppositions
    initial_head   = head_pitch_motor.getTargetPosition()
    initial_lhip   = lhip_pitch_motor.getTargetPosition()
    initial_rhip   = rhip_pitch_motor.getTargetPosition()
    initial_lknee  = lknee_pitch_motor.getTargetPosition()
    initial_rknee  = rknee_pitch_motor.getTargetPosition()
    initial_lankle = lankle_pitch_motor.getTargetPosition()
    initial_rankle = rankle_pitch_motor.getTargetPosition()

    init_l_shoulder_pitch = l_shoulder_pitch_motor.getTargetPosition()
    init_l_shoulder_roll  = l_shoulder_roll_motor.getTargetPosition()
    init_l_elbow_yaw      = l_elbow_yaw_motor.getTargetPosition()
    init_l_elbow_roll     = l_elbow_roll_motor.getTargetPosition()

    init_r_shoulder_pitch = r_shoulder_pitch_motor.getTargetPosition()
    init_r_shoulder_roll  = r_shoulder_roll_motor.getTargetPosition()
    init_r_elbow_yaw      = r_elbow_yaw_motor.getTargetPosition()
    init_r_elbow_roll     = r_elbow_roll_motor.getTargetPosition()

    #record initial positions for the hip roll (upright position)
    initial_lhip_roll = lhip_roll_motor.getTargetPosition()
    initial_rhip_roll = rhip_roll_motor.getTargetPosition()

    #main contrrol loop
    while robot.step(time_step) != -1:
        #bowing phase (transition to bow, hold, then return upright)
        if not transition_to_bow(robot, time_step,
                                 head_pitch_motor, lhip_pitch_motor, rhip_pitch_motor,
                                 lknee_pitch_motor, rknee_pitch_motor, lankle_pitch_motor, rankle_pitch_motor,
                                 initial_head, initial_lhip, initial_rhip, initial_lknee,
                                 initial_rknee, initial_lankle, initial_rankle,
                                 inertial_unit,
                                 l_shoulder_pitch_motor, l_shoulder_roll_motor, l_elbow_yaw_motor, l_elbow_roll_motor,
                                 r_shoulder_pitch_motor, r_shoulder_roll_motor, r_elbow_yaw_motor, r_elbow_roll_motor,
                                 init_l_shoulder_pitch, init_l_shoulder_roll, init_l_elbow_yaw, init_l_elbow_roll,
                                 init_r_shoulder_pitch, init_r_shoulder_roll, init_r_elbow_yaw, init_r_elbow_roll):
            return

        if not hold_pose(robot, time_step):
            return

        if not transition_to_upright(robot, time_step,
                                     head_pitch_motor, lhip_pitch_motor, rhip_pitch_motor,
                                     lknee_pitch_motor, rknee_pitch_motor, lankle_pitch_motor, rankle_pitch_motor,
                                     initial_head, initial_lhip, initial_rhip, initial_lknee,
                                     initial_rknee, initial_lankle, initial_rankle,
                                     inertial_unit,
                                     l_shoulder_pitch_motor, l_shoulder_roll_motor, l_elbow_yaw_motor, l_elbow_roll_motor,
                                     r_shoulder_pitch_motor, r_shoulder_roll_motor, r_elbow_yaw_motor, r_elbow_roll_motor,
                                     init_l_shoulder_pitch, init_l_shoulder_roll, init_l_elbow_yaw, init_l_elbow_roll,
                                     init_r_shoulder_pitch, init_r_shoulder_roll, init_r_elbow_yaw, init_r_elbow_roll):
            return

        if not hold_pose(robot, time_step):
            return

        #side step phase
        if not transition_to_side_step(robot, time_step,
                                        lhip_roll_motor, rhip_roll_motor,
                                        initial_lhip_roll, initial_rhip_roll,
                                        SIDE_STEP_ANGLE):
            return

        if not hold_pose(robot, time_step):
            return

        if not transition_to_side_step_back(robot, time_step,
                                             lhip_roll_motor, rhip_roll_motor,
                                             initial_lhip_roll, initial_rhip_roll,
                                             SIDE_STEP_ANGLE):
            return

        if not hold_pose(robot, time_step):
            return

if __name__ == "__main__":
    main()
