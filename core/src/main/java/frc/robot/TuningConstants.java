package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.google.inject.Injector;
import frc.robot.common.IMechanism;
import frc.robot.mechanisms.*;

/**
 * All constants related to tuning the operation of the robot.
 * 
 * @author Will
 * 
 */
public class TuningConstants
{
    public static final boolean COMPETITION_ROBOT = false;
    public static final boolean THROW_EXCEPTIONS = !TuningConstants.COMPETITION_ROBOT;

    public static List<IMechanism> GetActiveMechanisms(Injector injector)
    {
        List<IMechanism> mechanismList = new ArrayList<IMechanism>();
        mechanismList.add(injector.getInstance(DriveTrainMechanism.class));
        mechanismList.add(injector.getInstance(PowerManager.class));
        mechanismList.add(injector.getInstance(PositionManager.class));
        mechanismList.add(injector.getInstance(CompressorMechanism.class));
        mechanismList.add(injector.getInstance(ElevatorMechanism.class));
        mechanismList.add(injector.getInstance(GrabberMechanism.class));
        mechanismList.add(injector.getInstance(ClimberMechanism.class));
        //mechanismList.add(injector.getInstance(VisionManager.class));
        //mechanismList.add(injector.getInstance(OffboardVisionManager.class));
        //mechanismList.add(injector.getInstance(IndicatorLightManager.class));
        return mechanismList;
    }

    //================================================== Autonomous ==============================================================

    public static final boolean CANCEL_AUTONOMOUS_ROUTINE_ON_DISABLE = true;

    public static final double DRIVETRAIN_POSITIONAL_ACCEPTABLE_DELTA = 1.0;

    // Acceptable vision centering range values in degrees
    public static final double MAX_VISION_CENTERING_RANGE_DEGREES = 5.0;

    // Navx Turn Constants
    public static final double MAX_NAVX_TURN_RANGE_DEGREES = 4.0;
    public static final double MAX_NAVX_FAST_TURN_RANGE_DEGREES = 5.0;
    public static final double NAVX_TURN_COMPLETE_TIME = 0.4;
    public static final double NAVX_TURN_COMPLETE_CURRENT_VELOCITY_DELTA = 0;
    public static final double NAVX_TURN_COMPLETE_DESIRED_VELOCITY_DELTA = 0;

    // Navx Turn PID Constants
    public static final double NAVX_TURN_PID_KP = 0.25;
    public static final double NAVX_TURN_PID_KI = 0.0;
    public static final double NAVX_TURN_PID_KD = 1.0;
    public static final double NAVX_TURN_PID_KF = 0.0;
    public static final double NAVX_TURN_PID_KS = 1.0;
    public static final double NAVX_TURN_PID_MIN = -0.8;
    public static final double NAVX_TURN_PID_MAX = 0.8;
    public static final double NAVX_FAST_TURN_PID_KP = 0.0045;
    public static final double NAVX_FAST_TURN_PID_KI = 0.0;
    public static final double NAVX_FAST_TURN_PID_KD = 0.0;
    public static final double NAVX_FAST_TURN_PID_KF = 0.0;
    public static final double NAVX_FAST_TURN_PID_KS = 1.0;
    public static final double NAVX_FAST_TURN_PID_MIN = -0.8;
    public static final double NAVX_FAST_TURN_PID_MAX = 0.8;

    // Acceptable vision distance from tape in inches
    public static final double MAX_VISION_ACCEPTABLE_FORWARD_DISTANCE = 30.0;

    // PID settings for Centering the robot on a vision target from one stationary place
    public static final double VISION_STATIONARY_CENTERING_PID_KP = 0.08;
    public static final double VISION_STATIONARY_CENTERING_PID_KI = 0.0;
    public static final double VISION_STATIONARY_CENTERING_PID_KD = 0.08;
    public static final double VISION_STATIONARY_CENTERING_PID_KF = 0.0;
    public static final double VISION_STATIONARY_CENTERING_PID_KS = 1.0;
    public static final double VISION_STATIONARY_CENTERING_PID_MIN = -0.3;
    public static final double VISION_STATIONARY_CENTERING_PID_MAX = 0.3;

    // PID settings for Centering the robot on a vision target
    public static final double VISION_MOVING_CENTERING_PID_KP = 0.015;
    public static final double VISION_MOVING_CENTERING_PID_KI = 0.0;
    public static final double VISION_MOVING_CENTERING_PID_KD = 0.0;
    public static final double VISION_MOVING_CENTERING_PID_KF = 0.0;
    public static final double VISION_MOVING_CENTERING_PID_KS = 1.0;
    public static final double VISION_MOVING_CENTERING_PID_MIN = -0.3;
    public static final double VISION_MOVING_CENTERING_PID_MAX = 0.3;

    // PID settings for Advancing the robot towards a vision target
    public static final double VISION_ADVANCING_PID_KP = 0.005;
    public static final double VISION_ADVANCING_PID_KI = 0.0;
    public static final double VISION_ADVANCING_PID_KD = 0.0;
    public static final double VISION_ADVANCING_PID_KF = 0.0;
    public static final double VISION_ADVANCING_PID_KS = 1.0;
    public static final double VISION_ADVANCING_PID_MIN = -0.3;
    public static final double VISION_ADVANCING_PID_MAX = 0.3;

    //================================================== DriveTrain ==============================================================

    // Drivetrain PID keys/default values:
    public static final boolean DRIVETRAIN_USE_PID = true;
    public static final boolean DRIVETRAIN_USE_CROSS_COUPLING = false;
    public static final boolean DRIVETRAIN_USE_HEADING_CORRECTION = true;

    // Velocity PID (right)
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KP = 0.3;
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KI = 0.0;
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KD = 0.0;
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KF = 0.2; // .2 ==> ~ 1023 / 5000 (100% control authority)
    public static final double DRIVETRAIN_VELOCITY_PID_RIGHT_KS = 4500.0;

    // Velocity PID (left)
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KP = 0.3;
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KI = 0.0;
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KD = 0.0;
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KF = 0.2; // .2 ==> ~ 1023 / 5000 (100% control authority)
    public static final double DRIVETRAIN_VELOCITY_PID_LEFT_KS = 4500.0;

    // Path PID (right)
    public static final double DRIVETRAIN_PATH_PID_RIGHT_KP = 0.0002;
    public static final double DRIVETRAIN_PATH_PID_RIGHT_KI = 0.0;
    public static final double DRIVETRAIN_PATH_PID_RIGHT_KD = 0.0;
    public static final double DRIVETRAIN_PATH_PID_RIGHT_KF = 0.0;
    public static final double DRIVETRAIN_PATH_PID_RIGHT_KV = 1.0;
    public static final double DRIVETRAIN_PATH_PID_RIGHT_KCC = 0.0;
    public static final double DRIVETRAIN_PATH_RIGHT_HEADING_CORRECTION = 0.0;
    public static final double DRIVETRAIN_PATH_RIGHT_MAX_VELOCITY_INCHES_PER_SECOND = 10.0 * TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KS * HardwareConstants.DRIVETRAIN_RIGHT_PULSE_DISTANCE; // gets the max speed in inches per second (ticks per 100ms times inches per tick times 10)

    // Path PID (left)
    public static final double DRIVETRAIN_PATH_PID_LEFT_KP = 0.0002;
    public static final double DRIVETRAIN_PATH_PID_LEFT_KI = 0.0;
    public static final double DRIVETRAIN_PATH_PID_LEFT_KD = 0.0;
    public static final double DRIVETRAIN_PATH_PID_LEFT_KF = 0.0;
    public static final double DRIVETRAIN_PATH_PID_LEFT_KV = 1.0;
    public static final double DRIVETRAIN_PATH_PID_LEFT_KCC = 0.0;
    public static final double DRIVETRAIN_PATH_LEFT_HEADING_CORRECTION = 0.0;
    public static final double DRIVETRAIN_PATH_LEFT_MAX_VELOCITY_INCHES_PER_SECOND = 10.0 * TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KS * HardwareConstants.DRIVETRAIN_LEFT_PULSE_DISTANCE; // gets the max speed in inches per second (ticks per 100ms times inches per tick times 10)

    // Position PID (right)
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KP = 0.0002;
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KI = 0.0;
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KD = 0.0;
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KF = 0.0;
    public static final double DRIVETRAIN_POSITION_PID_RIGHT_KCC = 0.0001;

    // Position PID (left)
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KP = 0.0002;
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KI = 0.0;
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KD = 0.0;
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KF = 0.0;
    public static final double DRIVETRAIN_POSITION_PID_LEFT_KCC = 0.0001;

    // Brake PID (right)
    public static final double DRIVETRAIN_BRAKE_PID_RIGHT_KP = 0.0004;
    public static final double DRIVETRAIN_BRAKE_PID_RIGHT_KI = 0.0;
    public static final double DRIVETRAIN_BRAKE_PID_RIGHT_KD = 0.0;
    public static final double DRIVETRAIN_BRAKE_PID_RIGHT_KF = 0.0;

    // Brake PID (left)
    public static final double DRIVETRAIN_BRAKE_PID_LEFT_KP = 0.0004;
    public static final double DRIVETRAIN_BRAKE_PID_LEFT_KI = 0.0;
    public static final double DRIVETRAIN_BRAKE_PID_LEFT_KD = 0.0;
    public static final double DRIVETRAIN_BRAKE_PID_LEFT_KF = 0.0;

    // Drivetrain choices for one-stick drive
    public static final double DRIVETRAIN_K1 = 1.4;
    public static final double DRIVETRAIN_K2 = 0.5;

    // Drivetrain deadzone/max power levels
    public static final double DRIVETRAIN_X_DEAD_ZONE = .05;
    public static final double DRIVETRAIN_Y_DEAD_ZONE = .1;
    public static final double DRIVETRAIN_MAX_POWER_LEVEL = 1.0;// max power level (velocity)
    public static final double DRIVETRAIN_LEFT_POSITIONAL_NON_PID_MULTIPLICAND = HardwareConstants.DRIVETRAIN_LEFT_PULSE_DISTANCE / 60.0;
    public static final double DRIVETRAIN_RIGHT_POSITIONAL_NON_PID_MULTIPLICAND = HardwareConstants.DRIVETRAIN_RIGHT_PULSE_DISTANCE / 60.0;
    public static final double DRIVETRAIN_MAX_POWER_POSITIONAL_NON_PID = 0.2;// max power level (positional, non-PID)

    public static final double DRIVETRAIN_CROSS_COUPLING_ZERO_ERROR_RANGE = 100.0; // (in ticks)
    public static final double DRIVETRAIN_PATH_MAX_POWER_LEVEL = 1.0;
    public static final double DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL = 0.90; // 0.85
    public static final double DRIVETRAIN_BRAKE_MAX_POWER_LEVEL = 0.6;
    public static final double DRIVETRAIN_VELOCITY_MAX_POWER_LEVEL = 1.0;

    public static final boolean DRIVETRAIN_REGULAR_MODE_SQUARING = false;
    public static final boolean DRIVETRAIN_SIMPLE_MODE_SQUARING = false;

    public static final double DRIVETRAIN_ENCODER_ODOMETRY_ANGLE_CORRECTION = 1.0; // account for turning weirdness (any degree offset in the angle)

    //================================================== Elevator ==============================================================

    public static final double ELEVATOR_CLIMBING_MOVEMENT_TIME_THRESHOLD = 10.0;
    public static final double ELEVATOR_CLIMBING_HEIGHT_ERROR_THRESHOLD = 1.0;

    // Sensors
    public static final boolean ELEVATOR_FORWARD_LIMIT_SWITCH_ENABLED = true;
    public static final boolean ELEVATOR_FORWARD_LIMIT_SWITCH_NORMALLY_OPEN = true;
    public static final boolean ELEVATOR_REVERSE_LIMIT_SWITCH_ENABLED = true;
    public static final boolean ELEVATOR_REVERSE_LIMIT_SWITCH_NORMALLY_OPEN = true;

    // MotionMagic
    public static final boolean ELEVATOR_USE_PID = true;
    public static final boolean ELEVATOR_USE_MOTION_MAGIC = true;

    public static final double ELEVATOR_MM_POSITION_PID_KP = 0.15;
    public static final double ELEVATOR_MM_POSITION_PID_KI = 0.0;
    public static final double ELEVATOR_MM_POSITION_PID_KD = 0.0;
    public static final double ELEVATOR_MM_POSITION_PID_KF = 0.18;
    public static final int ELEVATOR_MM_POSITION_PID_CRUISE_VELOC = 3000;
    public static final int ELEVATOR_MM_POSITION_PID_ACCEL = 2000;

    // PID
    public static final double ELEVATOR_POSITION_PID_KP = 0.05;
    public static final double ELEVATOR_POSITION_PID_KI = 0.0;
    public static final double ELEVATOR_POSITION_PID_KD = 100.0;
    public static final double ELEVATOR_POSITION_PID_KF = 0.0;

    public static final double ELEVATOR_DEBUG_UP_POWER_LEVEL = 0.6;
    public static final double ELEVATOR_DEBUG_DOWN_POWER_LEVEL = -0.2;
    public static final double ELEVATOR_MOVE_VELOCITY = 10.0;

    // Positions
    public static final double ELEVATOR_BOTTOM_POSITION = 0.0;
    public static final double ELEVATOR_HATCH_2_POSITION = 30.0; // 30 inches
    public static final double ELEVATOR_HATCH_3_POSITION = 60.0;
    public static final double ELEVATOR_CARGO_1_POSITION = 15.0;
    public static final double ELEVATOR_CARGO_2_POSITION = 45.0;
    public static final double ELEVATOR_CARGO_3_POSITION = 65.0;
    public static final double ELEVATOR_CARGO_LOAD_POSITION = 10.0;

    //======================================================== Grabber =====================================

    public static final double GRABBER_SET_WRIST_TIME_THRESHOLD = 10.0;
    public static final double GRABBER_CARGO_INTAKE_OUTTAKE_OVERRIDE_TIME = 10.0;
    public static final double GRABBER_KICK_PANEL_DURATION = 1.0;
    
    // Cargo intake/outtake motor power
    public static final double GRABBER_CARGO_INTAKE_MOTOR_POWER = -0.6;
    public static final double GRABBER_CARGO_OUTTAKE_MOTOR_POWER = 0.6;

    //================================================== Climber ==============================================================

    public static final double CLIMBER_CLIMB_COMPLETED_VOLTAGE = 3.0;

    public static final boolean CLIMBER_ARMS_USE_PID = false; 
    public static final boolean CLIMBER_CAM_USE_PID = false;

    // Arms
    public static final double CLIMBER_ARMS_RETRACTED_POSITION = 0.0;
    public static final double CLIMBER_ARMS_LOW_CLIMB_POSITION = 10.0;
    public static final double CLIMBER_ARMS_HIGH_CLIMB_POSITION = 20.0;

    public static final boolean CLIMBER_ARMS_FORWARD_LIMIT_SWITCH_ENABLED = true;
    public static final boolean CLIMBER_ARMS_FORWARD_LIMIT_SWITCH_NORMALLY_OPEN = true;
    public static final boolean CLIMBER_ARMS_REVERSE_LIMIT_SWITCH_ENABLED = true;
    public static final boolean CLIMBER_ARMS_REVERSE_LIMIT_SWITCH_NORMALLY_OPEN = true;

    public static final double CLIMBER_ARMS_POSITION_MAX = 100000.0;
    public static final double CLIMBER_ARMS_DEBUG_FORWARD_POWER_LEVEL = 0.6;
    public static final double CLIMBER_ARMS_DEBUG_BACKWARDS_POWER_LEVEL = -0.6;

    public static final double CLIMBER_ARMS_MOVE_VELOCITY = 10.0;

    public static final double CLIMBER_ARMS_POSITION_PID_KP = 0.3;
    public static final double CLIMBER_ARMS_POSITION_PID_KI = 0.0;
    public static final double CLIMBER_ARMS_POSITION_PID_KD = 0.0;
    public static final double CLIMBER_ARMS_POSITION_PID_KF = 0.0;

    public static final double CLIMBER_ARMS_MOVEMENT_TIME_THRESHOLD = 10.0;
    public static final double CLIMBER_ARMS_POSITION_ERROR_THRESHOLD = 1.0;

    // Cam
    public static final double CLIMBER_CAM_STORED_POSITION = 0.0;
    public static final double CLIMBER_CAM_LOW_CLIMB_POSITION = 10.0;
    public static final double CLIMBER_CAM_HIGH_CLIMB_POSITION = 20.0;

    public static final double CLIMBER_CAM_DEBUG_FORWARD_POWER_LEVEL = 0.8;
    public static final double CLIMBER_CAM_DEBUG_BACKWARDS_POWER_LEVEL = -0.8;

    public static final double CLIMBER_CAM_MOVE_VELOCITY = 10.0;

    public static final double CLIMBER_CAM_POSITION_PID_KP = 0.3;
    public static final double CLIMBER_CAM_POSITION_PID_KI = 0.0;
    public static final double CLIMBER_CAM_POSITION_PID_KD = 0.0;
    public static final double CLIMBER_CAM_POSITION_PID_KF = 0.0;

    public static final double CLIMBER_CAM_MOVEMENT_TIME_THRESHOLD = 10.0;
    public static final double CLIMBER_CAM_POSITION_ERROR_THRESHOLD = 1.0;
}
