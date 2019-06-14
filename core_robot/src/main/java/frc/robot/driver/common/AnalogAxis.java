package frc.robot.driver.common;

/**
 * All constants that describe how the name of each axis on the joystick maps to its axis number.
 * 
 * Axis guide:
 * -----------------------
 * Logitech Xtreme 3D Pro:
 * 0 - X (stick, X)
 * 1 - Y (stick, Y)
 * 2 - Twist (stick, twist)
 * 3 - Throttle: -1 to 1
 * -----------------------
 * XBox One Controller:
 * 0 - LS_X (left stick, X)
 * 1 - LS_Y (left stick, Y)
 * 2 - LT (left trigger): 0 to 1
 * 3 - RT (right trigger): 0 to 1
 * 4 - RS_X (right stick, X)
 * 5 - RS_Y (right stick, Y)
 * -----------------------
 * PS4 Controller:
 * 0 - LS_X (left stick, X)
 * 1 - LS_Y (left stick, Y)
 * 2 - RS_X (right stick, X)
 * 3 - LT (left trigger, L2): -1 to 1
 * 4 - RT (right trigger, R2): -1 to 1
 * 5 - RS_Y (right stick, Y)
 * 
 */
public enum AnalogAxis
{
    NONE(-1),
    JOYSTICK_X(0),
    JOYSTICK_Y(1),
    JOYSTICK_TWIST(2),
    JOYSTICK_THROTTLE(3),
    XBONE_LEFT_STICK_X(0),
    XBONE_LEFT_STICK_Y(1),
    XBONE_LEFT_TRIGGER(2),
    XBONE_RIGHT_TRIGGER(3),
    XBONE_RIGHT_STICK_X(4),
    XBONE_RIGHT_STICK_Y(5),
    PS4_LEFT_STICK_X(0),
    PS4_LEFT_STICK_Y(1),
    PS4_LEFT_TRIGGER(3),
    PS4_RIGHT_TRIGGER(4),
    PS4_RIGHT_STICK_X(2),
    PS4_RIGHT_STICK_Y(5);

    public final int Value;
    private AnalogAxis(int value)
    {
        this.Value = value;
    }    
}