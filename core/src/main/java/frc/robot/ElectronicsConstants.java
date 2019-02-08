package frc.robot;

/**
 * All constants describing how the electronics are plugged together.
 * 
 * @author Will
 * 
 */
public class ElectronicsConstants
{
    // change INVERT_X_AXIS to true if positive on the joystick isn't to the right, and negative isn't to the left
    public static final boolean INVERT_X_AXIS = false;

    // change INVERT_Y_AXIS to true if positive on the joystick isn't forward, and negative isn't backwards.
    public static final boolean INVERT_Y_AXIS = true;

    // change INVERT_THROTTLE_AXIS to true if positive on the joystick isn't forward, and negative isn't backwards.
    public static final boolean INVERT_THROTTLE_AXIS = true;

    public static final int PCM_A_MODULE = 0;
    public static final int PCM_B_MODULE = 1;

    public static final int JOYSTICK_DRIVER_PORT = 0;
    public static final int JOYSTICK_CO_DRIVER_PORT = 1;

    //================================================== Auto ==============================================================

    public static final int AUTO_DIP_SWITCH_A_DIGITAL_CHANNEL = -1;
    public static final int AUTO_DIP_SWITCH_B_DIGITAL_CHANNEL = -1;
    public static final int AUTO_DIP_SWITCH_C_DIGITAL_CHANNEL = -1;
    public static final int AUTO_DIP_SWITCH_D_DIGITAL_CHANNEL = -1;

    //================================================== Vision ==============================================================

    public static final int VISION_RING_LIGHT_PWM_CHANNEL = 0;

    //================================================== DriveTrain ==============================================================

    public static final int DRIVETRAIN_LEFT_MASTER_CAN_ID = 1;
    public static final int DRIVETRAIN_LEFT_FOLLOWER1_CAN_ID = 2;
    public static final int DRIVETRAIN_LEFT_FOLLOWER2_CAN_ID = 3;
    public static final int DRIVETRAIN_RIGHT_MASTER_CAN_ID = 4;
    public static final int DRIVETRAIN_RIGHT_FOLLOWER1_CAN_ID = 5;
    public static final int DRIVETRAIN_RIGHT_FOLLOWER2_CAN_ID = 6;

    //================================================== Elevator ==============================================================

    public static final int ELEVATOR_MOTOR_MASTER_CAN_ID = 7;
    public static final int ELEVATOR_MOTOR_FOLLOWER_CAN_ID = 8;

    //================================================== Grabber ============================================================

    // Hatch ejection DoubleSolenoid
    public static final int GRABBER_KICKER_FORWARD_PCM_CHANNEL = 0;
    public static final int GRABBER_KICKER_REVERSE_PCM_CHANNEL = 1;

    // Cargo intake/outtake TalonSRX
    public static final int GRABBER_CARGO_MOTOR_CAN_ID = 9;

    // Wrist actuators
    public static final int GRABBER_WRIST_INNER_FORWARD_PCM_CHANNEL = 2;
    public static final int GRABBER_WRIST_INNER_REVERSE_PCM_CHANNEL = 3;
    
    public static final int GRABBER_WRIST_OUTER_FORWARD_PCM_CHANNEL = 4;
    public static final int GRABBER_WRIST_OUTER_REVERSE_PCM_CHANNEL = 5;

    //Limit switches
    public static final int GRABBER_CARGO_LIMIT_SWITCH_1_DIGITAL_CHANNEL = 0;
    public static final int GRABBER_CARGO_LIMIT_SWITCH_2_DIGITAL_CHANNEL = 1;
    public static final int GRABBER_HATCH_LIMIT_SWITCH_DIGITAL_CHANNEL = 2;

    //================================================== Climber ==============================================================

    public static final int CLIMBER_ARMS_MOTOR_MASTER_CAN_ID = 10;
    public static final int CLIMBER_ARMS_MOTOR_FOLLOWER_CAN_ID = 11;

    public static final int CLIMBER_CAM_MOTOR_MASTER_CAN_ID = 12;
    public static final int CLIMBER_CAM_MOTOR_FOLLOWER_CAN_ID = 13;

    public static final int CLIMBER_CAM_LIMIT_SWITCH_DIGITAL_CHANNEL = 3;
    public static final int CLIMBER_HEIGHT_SENSOR_ANALOG_CHANNEL = 0;
}
