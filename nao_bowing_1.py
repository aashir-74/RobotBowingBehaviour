"""Python Controller for Bowing Behavior in NAO Robot in Webots using keyframes and inverse kinematics"""



from controller import Robot
import math

def compute_leg_ik(bow_factor, initial_hip, initial_knee, initial_ankle):
    """
    Compute leg joint angles using a two-link planar inverse kinematics approach.
    
    Args:
      bow_factor: A value between 0 (upright) and 1 (full bow).
      initial_hip, initial_knee, initial_ankle: Initial (upright) joint positions.
    
    Returns:
      A tuple (final_hip, final_knee, final_ankle) representing the computed joint angles.
    """
    # Approximate link lengths (thigh and shin) in meters.
    L1 = 0.25  # Thigh length
    L2 = 0.25  # Shin length

    # Maximum vertical lowering of the hip for a full bow (bow_factor = 1).
    delta_h_max = 0.02  # meters

    # Effective leg length (distance from hip to ankle) given the bow.
    # At upright: L1 + L2, and it decreases as the robot bows.
    d = (L1 + L2) - (bow_factor * delta_h_max)

    # --- Inverse Kinematics for a two-link chain ---
    # Compute hip angle (theta1) relative to vertical.
    cos_theta1 = (L1**2 + d**2 - L2**2) / (2 * L1 * d)
    cos_theta1 = max(min(cos_theta1, 1.0), -1.0)
    theta1 = -math.acos(cos_theta1)  # Negative value for forward bending

    # Compute knee angle (theta2).
    cos_theta2 = (L1**2 + L2**2 - d**2) / (2 * L1 * L2)
    cos_theta2 = max(min(cos_theta2, 1.0), -1.0)
    theta2 = math.pi - math.acos(cos_theta2)

    # Compute ankle angle (theta3) to ensure the foot remains flat.
    theta3 = -(theta1 + theta2)

    # --- Adjust IK solution with offsets to match original target angles ---
    # Original target adjustments (from provided code):
    bow_angle_hip   = -0.8   # Desired change in hip pitch at full bow
    bow_angle_knee  =  0.6   # Desired change in knee pitch at full bow
    bow_angle_ankle = -0.3   # Desired change in ankle pitch at full bow

    # Compute computed angles at full bow (bow_factor = 1)
    d_full = (L1 + L2) - delta_h_max
    cos_theta1_full = (L1**2 + d_full**2 - L2**2) / (2 * L1 * d_full)
    cos_theta1_full = max(min(cos_theta1_full, 1.0), -1.0)
    theta1_full = -math.acos(cos_theta1_full)
    cos_theta2_full = (L1**2 + L2**2 - d_full**2) / (2 * L1 * L2)
    cos_theta2_full = max(min(cos_theta2_full, 1.0), -1.0)
    theta2_full = math.pi - math.acos(cos_theta2_full)
    theta3_full = -(theta1_full + theta2_full)

    # Determine additional offsets needed so that at bow_factor=1 the angles match the targets.
    hip_offset   = (initial_hip + bow_angle_hip)   - (initial_hip + theta1_full)
    knee_offset  = (initial_knee + bow_angle_knee) - (initial_knee + theta2_full)
    ankle_offset = (initial_ankle + bow_angle_ankle) - (initial_ankle + theta3_full)

    # Blend the computed IK angles with the offsets in proportion to bow_factor.
    final_hip   = initial_hip   + theta1   + hip_offset   * bow_factor
    final_knee  = initial_knee  + theta2   + knee_offset  * bow_factor
    final_ankle = initial_ankle + theta3   + ankle_offset * bow_factor

    return final_hip, final_knee, final_ankle

def main():
    # Create the Robot instance.
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())

    # Retrieve device handles for key joints.
    head_pitch_motor  = robot.getDevice("HeadPitch")
    lhip_pitch_motor  = robot.getDevice("LHipPitch")
    rhip_pitch_motor  = robot.getDevice("RHipPitch")
    lknee_pitch_motor = robot.getDevice("LKneePitch")
    rknee_pitch_motor = robot.getDevice("RKneePitch")
    lankle_pitch_motor= robot.getDevice("LAnklePitch")
    rankle_pitch_motor= robot.getDevice("RAnklePitch")

    # Save initial (upright) positions for all joints.
    initial_head    = head_pitch_motor.getTargetPosition()
    initial_lhip    = lhip_pitch_motor.getTargetPosition()
    initial_rhip    = rhip_pitch_motor.getTargetPosition()
    initial_lknee   = lknee_pitch_motor.getTargetPosition()
    initial_rknee   = rknee_pitch_motor.getTargetPosition()
    initial_lankle  = lankle_pitch_motor.getTargetPosition()
    initial_rankle  = rankle_pitch_motor.getTargetPosition()

    # Define maximum allowed head pitch angle.
    MAX_HEAD_PITCH = 0.514872

    # Define target adjustment for head pitch (in radians).
    bow_angle_head = 1.2

    # Define the number of interpolation steps for smooth transitions.
    steps = 50

    # Main control loop for the continuous bowing motion.
    while robot.step(time_step) != -1:
        # --- Transition into the deep bow posture ---
        for i in range(steps):
            bow_factor = (i + 1) / steps  # Gradually increase from 0 to 1.
            
            # Compute head pitch with clamping.
            head_pitch_val = initial_head + bow_factor * bow_angle_head
            if head_pitch_val > MAX_HEAD_PITCH:
                head_pitch_val = MAX_HEAD_PITCH
            head_pitch_motor.setPosition(head_pitch_val)

            # Compute leg joint angles using the IK function.
            lhip, lknee, lankle = compute_leg_ik(bow_factor, initial_lhip, initial_lknee, initial_lankle)
            rhip, rknee, rankle = compute_leg_ik(bow_factor, initial_rhip, initial_rknee, initial_rankle)

            # Set leg joint positions.
            lhip_pitch_motor.setPosition(lhip)
            rhip_pitch_motor.setPosition(rhip)
            lknee_pitch_motor.setPosition(lknee)
            rknee_pitch_motor.setPosition(rknee)
            lankle_pitch_motor.setPosition(lankle)
            rankle_pitch_motor.setPosition(rankle)

            if robot.step(time_step) == -1:
                return

        # Hold the deep bow posture briefly.
        hold_steps = 30
        for _ in range(hold_steps):
            if robot.step(time_step) == -1:
                return

        # --- Transition back to the upright posture ---
        for i in range(steps):
            bow_factor = (steps - i - 1) / steps  # Gradually decrease from 1 to 0.

            # Compute head pitch (reverse interpolation).
            head_pitch_val = initial_head + bow_factor * bow_angle_head
            if head_pitch_val > MAX_HEAD_PITCH:
                head_pitch_val = MAX_HEAD_PITCH
            head_pitch_motor.setPosition(head_pitch_val)

            # Compute leg joint angles using the IK function.
            lhip, lknee, lankle = compute_leg_ik(bow_factor, initial_lhip, initial_lknee, initial_lankle)
            rhip, rknee, rankle = compute_leg_ik(bow_factor, initial_rhip, initial_rknee, initial_rankle)

            # Set leg joint positions.
            lhip_pitch_motor.setPosition(lhip)
            rhip_pitch_motor.setPosition(rhip)
            lknee_pitch_motor.setPosition(lknee)
            rknee_pitch_motor.setPosition(rknee)
            lankle_pitch_motor.setPosition(lankle)
            rankle_pitch_motor.setPosition(rankle)

            if robot.step(time_step) == -1:
                return

        # Hold the upright posture briefly.
        for _ in range(hold_steps):
            if robot.step(time_step) == -1:
                return

if __name__ == "__main__":
    main()
