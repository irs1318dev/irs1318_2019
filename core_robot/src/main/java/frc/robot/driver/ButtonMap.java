package frc.robot.driver;

import javax.inject.Singleton;

import frc.robot.ElectronicsConstants;
import frc.robot.TuningConstants;
import frc.robot.driver.common.AnalogAxis;
import frc.robot.driver.common.IButtonMap;
import frc.robot.driver.common.UserInputDeviceButton;
import frc.robot.driver.common.buttons.ButtonType;
import frc.robot.driver.common.descriptions.AnalogOperationDescription;
import frc.robot.driver.common.descriptions.DigitalOperationDescription;
import frc.robot.driver.common.descriptions.MacroOperationDescription;
import frc.robot.driver.common.descriptions.ShiftDescription;
import frc.robot.driver.common.descriptions.UserInputDevice;
import frc.robot.driver.controltasks.*;

@Singleton
public class ButtonMap implements IButtonMap
{
    private static ShiftDescription[] ShiftButtonSchema = new ShiftDescription[]
    {
        new ShiftDescription(
            Shift.Debug,
            UserInputDevice.Driver,
            UserInputDeviceButton.JOYSTICK_STICK_TRIGGER_BUTTON),
        new ShiftDescription(
            Shift.ButtonPadDebug,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_16),
    };

    public static AnalogOperationDescription[] AnalogOperationSchema = new AnalogOperationDescription[]
    {
        // Operations for the drive train
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainMoveForward,
            UserInputDevice.Driver,
            AnalogAxis.JOYSTICK_Y,
            ElectronicsConstants.INVERT_Y_AXIS,
            TuningConstants.DRIVETRAIN_Y_DEAD_ZONE),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainTurn,
            UserInputDevice.Driver,
            AnalogAxis.JOYSTICK_X,
            ElectronicsConstants.INVERT_X_AXIS,
            TuningConstants.DRIVETRAIN_X_DEAD_ZONE),    
    };

    public static DigitalOperationDescription[] DigitalOperationSchema = new DigitalOperationDescription[]
    {
        // Operations for vision
        new DigitalOperationDescription(
            DigitalOperation.VisionDisable,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_6,
            Shift.Union(Shift.AllShifts),
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.VisionEnableOffboardStream,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_11,
            Shift.Union(Shift.AllShifts),
            Shift.None,
            ButtonType.Toggle),

        // Operations for the elevator
        new DigitalOperationDescription(
            DigitalOperation.ElevatorForceUp,
            UserInputDevice.Driver,
            UserInputDeviceButton.JOYSTICK_BASE_TOP_RIGHT_BUTTON,
            Shift.Debug,
            Shift.Debug,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.ElevatorForceDown,
            UserInputDevice.Driver,
            UserInputDeviceButton.JOYSTICK_BASE_MIDDLE_RIGHT_BUTTON,
            Shift.Debug,
            Shift.Debug,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.ElevatorMoveUp,
            UserInputDevice.Driver,
            UserInputDeviceButton.JOYSTICK_BASE_TOP_RIGHT_BUTTON,
            Shift.Debug,
            Shift.None,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.ElevatorMoveDown,
            UserInputDevice.Driver,
            UserInputDeviceButton.JOYSTICK_BASE_MIDDLE_RIGHT_BUTTON,
            Shift.Debug,
            Shift.None,
            ButtonType.Simple),  
        new DigitalOperationDescription(
            DigitalOperation.ElevatorBottomPosition,
            UserInputDevice.Driver,
            UserInputDeviceButton.JOYSTICK_BASE_BOTTOM_LEFT_BUTTON,
            Shift.Debug,
            Shift.None,
            ButtonType.Click), 
        new DigitalOperationDescription(
            DigitalOperation.ElevatorHatch2Position,
            UserInputDevice.Driver,
            UserInputDeviceButton.JOYSTICK_BASE_MIDDLE_LEFT_BUTTON,
            Shift.Debug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.ElevatorHatch3Position,
            UserInputDevice.Driver,
            UserInputDeviceButton.JOYSTICK_BASE_TOP_LEFT_BUTTON,
            Shift.Debug,
            Shift.None,
            ButtonType.Click), 
        new DigitalOperationDescription(
            DigitalOperation.ElevatorCargo1Position,
            UserInputDevice.Driver,
            UserInputDeviceButton.JOYSTICK_BASE_BOTTOM_LEFT_BUTTON,
            Shift.Debug,
            Shift.Debug,
            ButtonType.Click), 
        new DigitalOperationDescription(
            DigitalOperation.ElevatorCargo2Position,
            UserInputDevice.Driver,
            UserInputDeviceButton.JOYSTICK_BASE_MIDDLE_LEFT_BUTTON,
            Shift.Debug,
            Shift.Debug,
            ButtonType.Click), 
        new DigitalOperationDescription(
            DigitalOperation.ElevatorCargo3Position,
            UserInputDevice.Driver,
            UserInputDeviceButton.JOYSTICK_BASE_TOP_LEFT_BUTTON,
            Shift.Debug,
            Shift.Debug,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.ElevatorCargoLoadPosition,
            UserInputDevice.Driver,
            UserInputDeviceButton.JOYSTICK_BASE_BOTTOM_RIGHT_BUTTON,
            Shift.Debug,
            Shift.Debug,
            ButtonType.Click),  

        // Operations for grabber mechanism
        new DigitalOperationDescription(
            DigitalOperation.GrabberIntakeCargo,
            UserInputDevice.Driver,
            UserInputDeviceButton.JOYSTICK_STICK_BOTTOM_LEFT_BUTTON,
            Shift.Debug,
            Shift.None,
            ButtonType.Simple), 
        new DigitalOperationDescription(
            DigitalOperation.GrabberOuttakeCargo,
            UserInputDevice.Driver,
            UserInputDeviceButton.JOYSTICK_STICK_TOP_LEFT_BUTTON,
            Shift.Debug,
            Shift.None,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.GrabberKickPanel,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_13,
            Shift.Union(Shift.AllShifts),
            Shift.None,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.GrabberPointBeak,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_12,
            Shift.Union(Shift.AllShifts),
            Shift.None,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.GrabberWristStartPosition,
            UserInputDevice.Driver,
            180,
            Shift.Debug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.GrabberWristHatchPosition,
            UserInputDevice.Driver,
            90,
            Shift.Debug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.GrabberWristCargoPosition,
            UserInputDevice.Driver,
            270,
            Shift.Debug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.GrabberWristFloorPosition,
            UserInputDevice.Driver,
            0,
            Shift.Debug,
            Shift.None,
            ButtonType.Click),
    };

    public static MacroOperationDescription[] MacroSchema = new MacroOperationDescription[]
    {
        // Brake mode macro
        new MacroOperationDescription(
            MacroOperation.PIDBrake,
            UserInputDevice.Driver,
            UserInputDeviceButton.JOYSTICK_STICK_THUMB_BUTTON,
            ButtonType.Simple,
            () -> new PIDBrakeTask(),
            new IOperation[]
            {
                DigitalOperation.DriveTrainUsePositionalMode,
                DigitalOperation.DriveTrainUseBrakeMode,
                AnalogOperation.DriveTrainLeftPosition,
                AnalogOperation.DriveTrainRightPosition,
            }),

        // Driving Macros
        new MacroOperationDescription(
            MacroOperation.TurnInPlaceRight,
            UserInputDevice.Driver,
            90,
            Shift.Debug,
            Shift.Debug,
            ButtonType.Toggle,
            () -> new NavxTurnTask(true, 180, 3.0, true, false),
            new IOperation[]
            {
                DigitalOperation.DriveTrainUsePositionalMode,
                DigitalOperation.DriveTrainUseBrakeMode,
                AnalogOperation.DriveTrainLeftPosition,
                AnalogOperation.DriveTrainRightPosition,
                AnalogOperation.DriveTrainLeftVelocity,
                AnalogOperation.DriveTrainRightVelocity,
                AnalogOperation.DriveTrainTurn,
                AnalogOperation.DriveTrainMoveForward,
                DigitalOperation.DriveTrainSimpleMode,
            },
            new IOperation[]
            {
                DigitalOperation.DriveTrainUsePositionalMode,
                DigitalOperation.DriveTrainUseBrakeMode,
                AnalogOperation.DriveTrainLeftPosition,
                AnalogOperation.DriveTrainRightPosition,
            }),
        new MacroOperationDescription(
            MacroOperation.TurnInPlaceLeft,
            UserInputDevice.Driver,
            270,
            Shift.Debug,
            Shift.Debug,
            ButtonType.Toggle,
            () -> new NavxTurnTask(true, -180, TuningConstants.NAVX_FAST_TURN_TIMEOUT, true, true),
            new IOperation[]
            {
                DigitalOperation.DriveTrainUsePositionalMode,
                DigitalOperation.DriveTrainUseBrakeMode,
                AnalogOperation.DriveTrainLeftPosition,
                AnalogOperation.DriveTrainRightPosition,
                AnalogOperation.DriveTrainTurn,
                AnalogOperation.DriveTrainMoveForward,
                DigitalOperation.DriveTrainSimpleMode,
            },
            new IOperation[]
            {
                DigitalOperation.DriveTrainUsePositionalMode,
                DigitalOperation.DriveTrainUseBrakeMode,
                AnalogOperation.DriveTrainLeftPosition,
                AnalogOperation.DriveTrainRightPosition,
            }),
        new MacroOperationDescription(
            MacroOperation.FollowSomePath,
            UserInputDevice.None,
            UserInputDeviceButton.NONE,
            ButtonType.Toggle,
            () -> new FollowPathTask("/Paths/Circle 40 inch radius.csv"),
            new IOperation[]
            {
                DigitalOperation.DriveTrainUsePositionalMode,
                DigitalOperation.DriveTrainUseBrakeMode,
                AnalogOperation.DriveTrainLeftPosition,
                AnalogOperation.DriveTrainRightPosition,
                AnalogOperation.DriveTrainLeftVelocity,
                AnalogOperation.DriveTrainRightVelocity,
                AnalogOperation.DriveTrainHeadingCorrection,
                DigitalOperation.DriveTrainUsePathMode,
                AnalogOperation.DriveTrainTurn,
                AnalogOperation.DriveTrainMoveForward,
                DigitalOperation.DriveTrainSimpleMode,
            },
            new IOperation[]
            {
                DigitalOperation.DriveTrainUsePositionalMode,
                DigitalOperation.DriveTrainUseBrakeMode,
                AnalogOperation.DriveTrainLeftPosition,
                AnalogOperation.DriveTrainRightPosition,
            }),

        // Climbing Macros
        new MacroOperationDescription(
            MacroOperation.ClimbHabAndReset,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_15,
            Shift.Union(Shift.AllShifts),
            Shift.ButtonPadDebug,
            ButtonType.Toggle,
            () -> ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new ElevatorPositionTask(DigitalOperation.ElevatorBottomPosition),
                        new GrabberSetWristPositionTask(DigitalOperation.GrabberWristStartPosition),
                        new ClimberArmsPositionTask(DigitalOperation.ClimberArmsPrepClimbPosition)),
                    ConcurrentTask.AllTasks(
                        new ClimberArmsPositionTask(1.0, DigitalOperation.ClimberArmsHighClimbPosition),
                        new ClimberCamPositionTask(1.0, DigitalOperation.ClimberCamHighClimbPosition)),
                    ConcurrentTask.AllTasks(
                        new ClimberCamPositionTask(1.0, DigitalOperation.ClimberCamOutOfWayPosition),
                        new ClimberArmsPositionTask(0.25, DigitalOperation.ClimberArmsRetractedPosition),
                        new DriveUntilSensorTask(-0.15, 1.0)),
                    ConcurrentTask.AllTasks(
                        SequentialTask.Sequence(
                            new ElevatorPositionTask(1.0, DigitalOperation.ElevatorCamReturnPosition),
                            new ClimberCamPositionTask(1.0, DigitalOperation.ClimberCamStoredPosition),
                            new ElevatorPositionTask(DigitalOperation.ElevatorBottomPosition)),
                        new DriveDistanceTimedTask(-6.0, 1.5))),
                new VisionDisableTask(),
                new CompressorDisableTask()),
            new IOperation[]
            {
                DigitalOperation.CompressorForceDisable,
                DigitalOperation.VisionForceDisable,
                DigitalOperation.DriveTrainUsePositionalMode,
                DigitalOperation.DriveTrainUseBrakeMode,
                AnalogOperation.DriveTrainLeftPosition,
                AnalogOperation.DriveTrainRightPosition,
                AnalogOperation.DriveTrainLeftVelocity,
                AnalogOperation.DriveTrainRightVelocity,
                AnalogOperation.DriveTrainHeadingCorrection,
                DigitalOperation.DriveTrainUsePathMode,
                AnalogOperation.DriveTrainTurn,
                AnalogOperation.DriveTrainMoveForward,
                DigitalOperation.DriveTrainSimpleMode,
                DigitalOperation.DriveTrainUseSimplePathMode,
                DigitalOperation.ClimberArmsRetractedPosition,
                DigitalOperation.ClimberArmsPrepClimbPosition,
                DigitalOperation.ClimberArmsHighClimbPosition,
                DigitalOperation.ClimberCamStoredPosition,
                DigitalOperation.ClimberCamLowClimbPosition,
                DigitalOperation.ClimberCamHighClimbPosition,
                DigitalOperation.ClimberCamOutOfWayPosition,
                DigitalOperation.ClimberArmsMoveBackward,
                DigitalOperation.ClimberArmsMoveForward,
                DigitalOperation.ClimberCamMoveBackward,
                DigitalOperation.ClimberCamMoveForward,
                DigitalOperation.ClimberArmsForceBackward,
                DigitalOperation.ClimberArmsForceForward,
                DigitalOperation.ClimberCamForceBackward,
                DigitalOperation.ClimberCamForceForward,
                DigitalOperation.ElevatorBottomPosition,
                DigitalOperation.ElevatorHatch2Position,
                DigitalOperation.ElevatorHatch3Position,
                DigitalOperation.ElevatorCargo1Position,
                DigitalOperation.ElevatorCargo2Position,
                DigitalOperation.ElevatorCargo3Position,
                DigitalOperation.ElevatorCargoLoadPosition,
                DigitalOperation.ElevatorCamReturnPosition,
                DigitalOperation.ElevatorMoveUp,
                DigitalOperation.ElevatorMoveDown,
                DigitalOperation.ElevatorForceUp,
                DigitalOperation.ElevatorForceDown,
                DigitalOperation.GrabberWristStartPosition,
                DigitalOperation.GrabberWristHatchPosition,
                DigitalOperation.GrabberWristCargoPosition,
                DigitalOperation.GrabberWristFloorPosition
            },
            new IOperation[]
            {
                DigitalOperation.DriveTrainUsePositionalMode,
                DigitalOperation.DriveTrainUseBrakeMode,
                AnalogOperation.DriveTrainLeftPosition,
                AnalogOperation.DriveTrainRightPosition,
                DigitalOperation.ClimberArmsMoveBackward,
                DigitalOperation.ClimberArmsMoveForward,
                DigitalOperation.ClimberCamMoveBackward,
                DigitalOperation.ClimberCamMoveForward,
                DigitalOperation.ClimberArmsForceBackward,
                DigitalOperation.ClimberArmsForceForward,
                DigitalOperation.ClimberCamForceBackward,
                DigitalOperation.ClimberCamForceForward,
            }),
        new MacroOperationDescription(
            MacroOperation.ClimbHabLeaveTailDown,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_10,
            Shift.Union(Shift.AllShifts),
            Shift.ButtonPadDebug,
            ButtonType.Toggle,
            () -> ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new ElevatorPositionTask(DigitalOperation.ElevatorBottomPosition),
                        new GrabberSetWristPositionTask(DigitalOperation.GrabberWristStartPosition),
                        new ClimberArmsPositionTask(DigitalOperation.ClimberArmsPrepClimbPosition)),
                    ConcurrentTask.AllTasks(
                        new ClimberArmsPositionTask(1.0, DigitalOperation.ClimberArmsHighClimbPosition),
                        new ClimberCamPositionTask(1.0, DigitalOperation.ClimberCamHighClimbPosition)),
                    ConcurrentTask.AllTasks(
                        new ClimberCamPositionTask(1.0, DigitalOperation.ClimberCamOutOfWayPosition),
                        new ClimberArmsPositionTask(0.25, DigitalOperation.ClimberArmsRetractedPosition),
                        new DriveUntilSensorTask(-0.15, 1.0))),
                new VisionDisableTask(),
                new CompressorDisableTask()),
            new IOperation[]
            {
                DigitalOperation.CompressorForceDisable,
                DigitalOperation.VisionForceDisable,
                DigitalOperation.DriveTrainUsePositionalMode,
                DigitalOperation.DriveTrainUseBrakeMode,
                AnalogOperation.DriveTrainLeftPosition,
                AnalogOperation.DriveTrainRightPosition,
                AnalogOperation.DriveTrainLeftVelocity,
                AnalogOperation.DriveTrainRightVelocity,
                AnalogOperation.DriveTrainHeadingCorrection,
                DigitalOperation.DriveTrainUsePathMode,
                AnalogOperation.DriveTrainTurn,
                AnalogOperation.DriveTrainMoveForward,
                DigitalOperation.DriveTrainSimpleMode,
                DigitalOperation.DriveTrainUseSimplePathMode,
                DigitalOperation.ClimberArmsRetractedPosition,
                DigitalOperation.ClimberArmsPrepClimbPosition,
                DigitalOperation.ClimberArmsHighClimbPosition,
                DigitalOperation.ClimberCamStoredPosition,
                DigitalOperation.ClimberCamLowClimbPosition,
                DigitalOperation.ClimberCamHighClimbPosition,
                DigitalOperation.ClimberCamOutOfWayPosition,
                DigitalOperation.ClimberArmsMoveBackward,
                DigitalOperation.ClimberArmsMoveForward,
                DigitalOperation.ClimberCamMoveBackward,
                DigitalOperation.ClimberCamMoveForward,
                DigitalOperation.ClimberArmsForceBackward,
                DigitalOperation.ClimberArmsForceForward,
                DigitalOperation.ClimberCamForceBackward,
                DigitalOperation.ClimberCamForceForward,
                DigitalOperation.ElevatorBottomPosition,
                DigitalOperation.ElevatorHatch2Position,
                DigitalOperation.ElevatorHatch3Position,
                DigitalOperation.ElevatorCargo1Position,
                DigitalOperation.ElevatorCargo2Position,
                DigitalOperation.ElevatorCargo3Position,
                DigitalOperation.ElevatorCargoLoadPosition,
                DigitalOperation.ElevatorCamReturnPosition,
                DigitalOperation.ElevatorMoveUp,
                DigitalOperation.ElevatorMoveDown,
                DigitalOperation.ElevatorForceUp,
                DigitalOperation.ElevatorForceDown,
                DigitalOperation.GrabberWristStartPosition,
                DigitalOperation.GrabberWristHatchPosition,
                DigitalOperation.GrabberWristCargoPosition,
                DigitalOperation.GrabberWristFloorPosition
            },
            new IOperation[]
            {
                DigitalOperation.DriveTrainUsePositionalMode,
                DigitalOperation.DriveTrainUseBrakeMode,
                AnalogOperation.DriveTrainLeftPosition,
                AnalogOperation.DriveTrainRightPosition,
                DigitalOperation.ClimberArmsMoveBackward,
                DigitalOperation.ClimberArmsMoveForward,
                DigitalOperation.ClimberCamMoveBackward,
                DigitalOperation.ClimberCamMoveForward,
                DigitalOperation.ClimberArmsForceBackward,
                DigitalOperation.ClimberArmsForceForward,
                DigitalOperation.ClimberCamForceBackward,
                DigitalOperation.ClimberCamForceForward,
            }),
        new MacroOperationDescription(
            MacroOperation.ClimbPiecewiseA,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_11,
            Shift.Union(Shift.AllShifts),
            Shift.ButtonPadDebug,
            ButtonType.Toggle,
            () -> ConcurrentTask.AllTasks(
                new ElevatorPositionTask(DigitalOperation.ElevatorBottomPosition),
                new GrabberSetWristPositionTask(DigitalOperation.GrabberWristStartPosition),
                new ClimberArmsPositionTask(DigitalOperation.ClimberArmsHighClimbPosition)),
            new IOperation[]
            {
                DigitalOperation.DriveTrainUsePositionalMode,
                DigitalOperation.DriveTrainUseBrakeMode,
                AnalogOperation.DriveTrainLeftPosition,
                AnalogOperation.DriveTrainRightPosition,
                AnalogOperation.DriveTrainLeftVelocity,
                AnalogOperation.DriveTrainRightVelocity,
                AnalogOperation.DriveTrainHeadingCorrection,
                DigitalOperation.DriveTrainUsePathMode,
                AnalogOperation.DriveTrainTurn,
                AnalogOperation.DriveTrainMoveForward,
                DigitalOperation.DriveTrainSimpleMode,
                DigitalOperation.DriveTrainUseSimplePathMode,
                DigitalOperation.ClimberArmsRetractedPosition,
                DigitalOperation.ClimberArmsPrepClimbPosition,
                DigitalOperation.ClimberArmsHighClimbPosition,
                DigitalOperation.ClimberCamStoredPosition,
                DigitalOperation.ClimberCamLowClimbPosition,
                DigitalOperation.ClimberCamHighClimbPosition,
                DigitalOperation.ClimberCamOutOfWayPosition,
                DigitalOperation.ClimberArmsMoveBackward,
                DigitalOperation.ClimberArmsMoveForward,
                DigitalOperation.ClimberCamMoveBackward,
                DigitalOperation.ClimberCamMoveForward,
                DigitalOperation.ClimberArmsForceBackward,
                DigitalOperation.ClimberArmsForceForward,
                DigitalOperation.ClimberCamForceBackward,
                DigitalOperation.ClimberCamForceForward,
                DigitalOperation.ElevatorBottomPosition,
                DigitalOperation.ElevatorHatch2Position,
                DigitalOperation.ElevatorHatch3Position,
                DigitalOperation.ElevatorCargo1Position,
                DigitalOperation.ElevatorCargo2Position,
                DigitalOperation.ElevatorCargo3Position,
                DigitalOperation.ElevatorCargoLoadPosition,
                DigitalOperation.ElevatorCamReturnPosition,
                DigitalOperation.ElevatorMoveUp,
                DigitalOperation.ElevatorMoveDown,
                DigitalOperation.ElevatorForceUp,
                DigitalOperation.ElevatorForceDown,
                DigitalOperation.GrabberWristStartPosition,
                DigitalOperation.GrabberWristHatchPosition,
                DigitalOperation.GrabberWristCargoPosition,
                DigitalOperation.GrabberWristFloorPosition
            },
            new IOperation[]
            {
                DigitalOperation.ClimberArmsMoveBackward,
                DigitalOperation.ClimberArmsMoveForward,
                DigitalOperation.ClimberCamMoveBackward,
                DigitalOperation.ClimberCamMoveForward,
                DigitalOperation.ClimberArmsForceBackward,
                DigitalOperation.ClimberArmsForceForward,
                DigitalOperation.ClimberCamForceBackward,
                DigitalOperation.ClimberCamForceForward,
                DigitalOperation.ElevatorBottomPosition,
                DigitalOperation.ElevatorHatch2Position,
                DigitalOperation.ElevatorHatch3Position,
                DigitalOperation.ElevatorCargo1Position,
                DigitalOperation.ElevatorCargo2Position,
                DigitalOperation.ElevatorCargo3Position,
                DigitalOperation.ElevatorCargoLoadPosition,
                DigitalOperation.ElevatorCamReturnPosition,
                DigitalOperation.ElevatorMoveUp,
                DigitalOperation.ElevatorMoveDown,
                DigitalOperation.ElevatorForceUp,
                DigitalOperation.ElevatorForceDown
            }),
        new MacroOperationDescription(
            MacroOperation.ClimbPiecewiseB,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_12,
            Shift.Union(Shift.AllShifts),
            Shift.ButtonPadDebug,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new DriveDistancePositionTimedTask(-0.2, -12.0, 0.75),
                new ClimberCamPositionTask(0.25, DigitalOperation.ClimberCamHighClimbPosition),
                new ClimberArmsPositionTask(DigitalOperation.ClimberArmsPrepClimbPosition),
                new DriveUntilSensorTask(-0.2, 2.0)),
            new IOperation[]
            {
                DigitalOperation.DriveTrainUsePositionalMode,
                DigitalOperation.DriveTrainUseBrakeMode,
                AnalogOperation.DriveTrainLeftPosition,
                AnalogOperation.DriveTrainRightPosition,
                AnalogOperation.DriveTrainLeftVelocity,
                AnalogOperation.DriveTrainRightVelocity,
                AnalogOperation.DriveTrainHeadingCorrection,
                DigitalOperation.DriveTrainUsePathMode,
                AnalogOperation.DriveTrainTurn,
                AnalogOperation.DriveTrainMoveForward,
                DigitalOperation.DriveTrainSimpleMode,
                DigitalOperation.DriveTrainUseSimplePathMode,
                DigitalOperation.ClimberArmsRetractedPosition,
                DigitalOperation.ClimberArmsPrepClimbPosition,
                DigitalOperation.ClimberArmsHighClimbPosition,
                DigitalOperation.ClimberCamStoredPosition,
                DigitalOperation.ClimberCamLowClimbPosition,
                DigitalOperation.ClimberCamHighClimbPosition,
                DigitalOperation.ClimberCamOutOfWayPosition,
                DigitalOperation.ClimberArmsMoveBackward,
                DigitalOperation.ClimberArmsMoveForward,
                DigitalOperation.ClimberCamMoveBackward,
                DigitalOperation.ClimberCamMoveForward,
                DigitalOperation.ClimberArmsForceBackward,
                DigitalOperation.ClimberArmsForceForward,
                DigitalOperation.ClimberCamForceBackward,
                DigitalOperation.ClimberCamForceForward,
                DigitalOperation.ElevatorBottomPosition,
                DigitalOperation.ElevatorHatch2Position,
                DigitalOperation.ElevatorHatch3Position,
                DigitalOperation.ElevatorCargo1Position,
                DigitalOperation.ElevatorCargo2Position,
                DigitalOperation.ElevatorCargo3Position,
                DigitalOperation.ElevatorCargoLoadPosition,
                DigitalOperation.ElevatorCamReturnPosition,
                DigitalOperation.ElevatorMoveUp,
                DigitalOperation.ElevatorMoveDown,
                DigitalOperation.ElevatorForceUp,
                DigitalOperation.ElevatorForceDown
            },
            new IOperation[]
            {
                DigitalOperation.ClimberArmsMoveBackward,
                DigitalOperation.ClimberArmsMoveForward,
                DigitalOperation.ClimberCamMoveBackward,
                DigitalOperation.ClimberCamMoveForward,
                DigitalOperation.ClimberArmsForceBackward,
                DigitalOperation.ClimberArmsForceForward,
                DigitalOperation.ClimberCamForceBackward,
                DigitalOperation.ClimberCamForceForward,
                DigitalOperation.ElevatorBottomPosition,
                DigitalOperation.ElevatorHatch2Position,
                DigitalOperation.ElevatorHatch3Position,
                DigitalOperation.ElevatorCargo1Position,
                DigitalOperation.ElevatorCargo2Position,
                DigitalOperation.ElevatorCargo3Position,
                DigitalOperation.ElevatorCargoLoadPosition,
                DigitalOperation.ElevatorCamReturnPosition,
                DigitalOperation.ElevatorMoveUp,
                DigitalOperation.ElevatorMoveDown,
                DigitalOperation.ElevatorForceUp,
                DigitalOperation.ElevatorForceDown
            }),
        new MacroOperationDescription(
            MacroOperation.ClimbPiecewiseC,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_13,
            Shift.Union(Shift.AllShifts),
            Shift.ButtonPadDebug,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new ClimberCamPositionTask(1.0, DigitalOperation.ClimberCamOutOfWayPosition),
                new ClimberArmsPositionTask(0.25, DigitalOperation.ClimberArmsRetractedPosition),
                new DriveDistanceTimedTask(-12.0, 1.25)),
            new IOperation[]
            {
                DigitalOperation.DriveTrainUsePositionalMode,
                DigitalOperation.DriveTrainUseBrakeMode,
                AnalogOperation.DriveTrainLeftPosition,
                AnalogOperation.DriveTrainRightPosition,
                AnalogOperation.DriveTrainLeftVelocity,
                AnalogOperation.DriveTrainRightVelocity,
                AnalogOperation.DriveTrainHeadingCorrection,
                DigitalOperation.DriveTrainUsePathMode,
                AnalogOperation.DriveTrainTurn,
                AnalogOperation.DriveTrainMoveForward,
                DigitalOperation.DriveTrainSimpleMode,
                DigitalOperation.DriveTrainUseSimplePathMode,
                DigitalOperation.ClimberArmsRetractedPosition,
                DigitalOperation.ClimberArmsPrepClimbPosition,
                DigitalOperation.ClimberArmsHighClimbPosition,
                DigitalOperation.ClimberCamStoredPosition,
                DigitalOperation.ClimberCamLowClimbPosition,
                DigitalOperation.ClimberCamHighClimbPosition,
                DigitalOperation.ClimberCamOutOfWayPosition,
                DigitalOperation.ClimberArmsMoveBackward,
                DigitalOperation.ClimberArmsMoveForward,
                DigitalOperation.ClimberCamMoveBackward,
                DigitalOperation.ClimberCamMoveForward,
                DigitalOperation.ClimberArmsForceBackward,
                DigitalOperation.ClimberArmsForceForward,
                DigitalOperation.ClimberCamForceBackward,
                DigitalOperation.ClimberCamForceForward,
                DigitalOperation.ElevatorBottomPosition,
                DigitalOperation.ElevatorHatch2Position,
                DigitalOperation.ElevatorHatch3Position,
                DigitalOperation.ElevatorCargo1Position,
                DigitalOperation.ElevatorCargo2Position,
                DigitalOperation.ElevatorCargo3Position,
                DigitalOperation.ElevatorCargoLoadPosition,
                DigitalOperation.ElevatorCamReturnPosition,
                DigitalOperation.ElevatorMoveUp,
                DigitalOperation.ElevatorMoveDown,
                DigitalOperation.ElevatorForceUp,
                DigitalOperation.ElevatorForceDown
            },
            new IOperation[]
            {
                DigitalOperation.ClimberArmsMoveBackward,
                DigitalOperation.ClimberArmsMoveForward,
                DigitalOperation.ClimberCamMoveBackward,
                DigitalOperation.ClimberCamMoveForward,
                DigitalOperation.ClimberArmsForceBackward,
                DigitalOperation.ClimberArmsForceForward,
                DigitalOperation.ClimberCamForceBackward,
                DigitalOperation.ClimberCamForceForward,
                DigitalOperation.ElevatorBottomPosition,
                DigitalOperation.ElevatorHatch2Position,
                DigitalOperation.ElevatorHatch3Position,
                DigitalOperation.ElevatorCargo1Position,
                DigitalOperation.ElevatorCargo2Position,
                DigitalOperation.ElevatorCargo3Position,
                DigitalOperation.ElevatorCargoLoadPosition,
                DigitalOperation.ElevatorCamReturnPosition,
                DigitalOperation.ElevatorMoveUp,
                DigitalOperation.ElevatorMoveDown,
                DigitalOperation.ElevatorForceUp,
                DigitalOperation.ElevatorForceDown
            }),
        new MacroOperationDescription(
            MacroOperation.ClimbPiecewiseD,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_14,
            Shift.Union(Shift.AllShifts),
            Shift.ButtonPadDebug,
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new ElevatorPositionTask(1.0, DigitalOperation.ElevatorCamReturnPosition),
                new ClimberCamPositionTask(1.0, DigitalOperation.ClimberCamStoredPosition),
                new ElevatorPositionTask(DigitalOperation.ElevatorBottomPosition)),
            new IOperation[]
            {
                DigitalOperation.ClimberArmsRetractedPosition,
                DigitalOperation.ClimberArmsPrepClimbPosition,
                DigitalOperation.ClimberArmsHighClimbPosition,
                DigitalOperation.ClimberCamStoredPosition,
                DigitalOperation.ClimberCamLowClimbPosition,
                DigitalOperation.ClimberCamHighClimbPosition,
                DigitalOperation.ClimberCamOutOfWayPosition,
                DigitalOperation.ClimberArmsMoveBackward,
                DigitalOperation.ClimberArmsMoveForward,
                DigitalOperation.ClimberCamMoveBackward,
                DigitalOperation.ClimberCamMoveForward,
                DigitalOperation.ClimberArmsForceBackward,
                DigitalOperation.ClimberArmsForceForward,
                DigitalOperation.ClimberCamForceBackward,
                DigitalOperation.ClimberCamForceForward,
                DigitalOperation.ElevatorBottomPosition,
                DigitalOperation.ElevatorHatch2Position,
                DigitalOperation.ElevatorHatch3Position,
                DigitalOperation.ElevatorCargo1Position,
                DigitalOperation.ElevatorCargo2Position,
                DigitalOperation.ElevatorCargo3Position,
                DigitalOperation.ElevatorCargoLoadPosition,
                DigitalOperation.ElevatorCamReturnPosition,
                DigitalOperation.ElevatorMoveUp,
                DigitalOperation.ElevatorMoveDown,
                DigitalOperation.ElevatorForceUp,
                DigitalOperation.ElevatorForceDown
            },
            new IOperation[]
            {
                DigitalOperation.ClimberArmsMoveBackward,
                DigitalOperation.ClimberArmsMoveForward,
                DigitalOperation.ClimberCamMoveBackward,
                DigitalOperation.ClimberCamMoveForward,
                DigitalOperation.ClimberArmsForceBackward,
                DigitalOperation.ClimberArmsForceForward,
                DigitalOperation.ClimberCamForceBackward,
                DigitalOperation.ClimberCamForceForward,
                DigitalOperation.ElevatorBottomPosition,
                DigitalOperation.ElevatorHatch2Position,
                DigitalOperation.ElevatorHatch3Position,
                DigitalOperation.ElevatorCargo1Position,
                DigitalOperation.ElevatorCargo2Position,
                DigitalOperation.ElevatorCargo3Position,
                DigitalOperation.ElevatorCargoLoadPosition,
                DigitalOperation.ElevatorCamReturnPosition,
                DigitalOperation.ElevatorMoveUp,
                DigitalOperation.ElevatorMoveDown,
                DigitalOperation.ElevatorForceUp,
                DigitalOperation.ElevatorForceDown
            }),

        // Vision Macros
        new MacroOperationDescription(
            MacroOperation.VisionCenterAndAdvanceRocket,
            UserInputDevice.Driver,
            180,
            Shift.Debug,
            Shift.Debug,
            ButtonType.Toggle,
            () -> new VisionAdvanceAndCenterTask(DigitalOperation.VisionEnableRocket),
            new IOperation[]
            {
                DigitalOperation.VisionDisable,
                DigitalOperation.VisionEnableCargoShip,
                DigitalOperation.VisionEnableRocket,
                DigitalOperation.DriveTrainUsePositionalMode,
                AnalogOperation.DriveTrainLeftPosition,
                AnalogOperation.DriveTrainRightPosition,
                AnalogOperation.DriveTrainTurn,
                AnalogOperation.DriveTrainMoveForward
            }),
        new MacroOperationDescription(
            MacroOperation.VisionCenterAndAdvanceCargoShip,
            UserInputDevice.Driver,
            0,
            Shift.Debug,
            Shift.Debug,
            ButtonType.Toggle,
            () -> new VisionAdvanceAndCenterTask(DigitalOperation.VisionEnableCargoShip),
            new IOperation[]
            {
                DigitalOperation.VisionDisable,
                DigitalOperation.VisionEnableCargoShip,
                DigitalOperation.VisionEnableRocket,
                DigitalOperation.DriveTrainUsePositionalMode,
                AnalogOperation.DriveTrainLeftPosition,
                AnalogOperation.DriveTrainRightPosition,
                AnalogOperation.DriveTrainTurn,
                AnalogOperation.DriveTrainMoveForward
            }),
        new MacroOperationDescription(
            MacroOperation.VisionCenterAndAdvanceCargoShip,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_7,
            Shift.Union(Shift.AllShifts),
            Shift.Debug,
            ButtonType.Toggle,
            () -> new VisionAdvanceAndCenterTask(DigitalOperation.VisionEnableCargoShip),
            new IOperation[]
            {
                DigitalOperation.VisionDisable,
                DigitalOperation.VisionEnableCargoShip,
                DigitalOperation.VisionEnableRocket,
                DigitalOperation.DriveTrainUsePositionalMode,
                AnalogOperation.DriveTrainLeftPosition,
                AnalogOperation.DriveTrainRightPosition,
                AnalogOperation.DriveTrainTurn,
                AnalogOperation.DriveTrainMoveForward
            }),
        new MacroOperationDescription(
            MacroOperation.VisionFastCenterAndAdvanceCargoShip,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_7,
            Shift.Union(Shift.AllShifts),
            Shift.None,
            ButtonType.Toggle,
            () -> new VisionFastAdvanceAndCenterTask(DigitalOperation.VisionEnableCargoShip),
            new IOperation[]
            {
                DigitalOperation.VisionDisable,
                DigitalOperation.VisionEnableCargoShip,
                DigitalOperation.VisionEnableRocket,
                DigitalOperation.DriveTrainUsePositionalMode,
                AnalogOperation.DriveTrainLeftPosition,
                AnalogOperation.DriveTrainRightPosition,
                AnalogOperation.DriveTrainTurn,
                AnalogOperation.DriveTrainMoveForward
            }),
        new MacroOperationDescription(
            MacroOperation.VisionScanning,
            UserInputDevice.Driver,
            UserInputDeviceButton.JOYSTICK_BASE_BOTTOM_RIGHT_BUTTON,
            Shift.Debug,
            Shift.None,
            ButtonType.Toggle,
            () -> new VisionScanningTask(),
            new IOperation[]
            {
                DigitalOperation.VisionEnableOffboardProcessing,
                AnalogOperation.DriveTrainTurn,
                DigitalOperation.VisionDisable
            }),

        // Grabber Macro
        new MacroOperationDescription(
            MacroOperation.GrabberKickPanelRepeatedlyTask,
            UserInputDevice.Driver,
            UserInputDeviceButton.JOYSTICK_STICK_TOP_RIGHT_BUTTON,
            Shift.Debug,
            Shift.Debug,
            ButtonType.Simple,
            () -> new GrabberKickPanelRepeatedlyTask(),
            new IOperation[]
            {
                DigitalOperation.GrabberKickPanel,
            }),
        new MacroOperationDescription(
            MacroOperation.GrabberKickPanel,
            UserInputDevice.Driver,
            UserInputDeviceButton.JOYSTICK_STICK_TOP_RIGHT_BUTTON,
            Shift.Debug,
            Shift.None,
            ButtonType.Simple,
            () -> ConcurrentTask.AllTasks(
                new GrabberKickPanelTask(true),
                new GrabberPointBeakTask(true)),
            new IOperation[]
            {
                DigitalOperation.GrabberKickPanel,
                DigitalOperation.GrabberPointBeak,
            }),
        new MacroOperationDescription(
            MacroOperation.GrabberPointBeak,
            UserInputDevice.Driver,
            UserInputDeviceButton.JOYSTICK_STICK_BOTTOM_RIGHT_BUTTON,
            Shift.Debug,
            Shift.None,
            ButtonType.Simple,
            () -> new GrabberPointBeakTask(true),
            new IOperation[]
            {
                DigitalOperation.GrabberPointBeak,
            }),
        new MacroOperationDescription(
            MacroOperation.GrabberIntakeCargo,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_14,
            Shift.Union(Shift.AllShifts),
            Shift.None,
            ButtonType.Simple,
            () -> new GrabberCargoIntakeOuttakeTask(DigitalOperation.GrabberIntakeCargo, false),
            new IOperation[]
            {
                DigitalOperation.GrabberIntakeCargo,
                DigitalOperation.GrabberOuttakeCargo,
            }),
        new MacroOperationDescription(
            MacroOperation.GrabberOuttakeCargo,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_15,
            Shift.Union(Shift.AllShifts),
            Shift.None,
            ButtonType.Simple,
            () -> new GrabberCargoIntakeOuttakeTask(DigitalOperation.GrabberOuttakeCargo, false),
            new IOperation[]
            {
                DigitalOperation.GrabberIntakeCargo,
                DigitalOperation.GrabberOuttakeCargo,
            }),


        // Elevator Macros
        new MacroOperationDescription(
            MacroOperation.ElevatorMoveUp,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_12,
            Shift.Union(Shift.AllShifts),
            Shift.Debug,
            ButtonType.Simple,
            () -> new ElevatorMovementTask(DigitalOperation.ElevatorMoveUp),
            new IOperation[]
            {
                DigitalOperation.ElevatorMoveUp,
                DigitalOperation.ElevatorMoveDown,
                DigitalOperation.ElevatorForceUp,
                DigitalOperation.ElevatorForceDown,
            }),
        new MacroOperationDescription(
            MacroOperation.ElevatorMoveDown,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_13,
            Shift.Union(Shift.AllShifts),
            Shift.Debug,
            ButtonType.Simple,
            () -> new ElevatorMovementTask(DigitalOperation.ElevatorMoveDown),
            new IOperation[]
            {
                DigitalOperation.ElevatorMoveUp,
                DigitalOperation.ElevatorMoveDown,
                DigitalOperation.ElevatorForceUp,
                DigitalOperation.ElevatorForceDown,
            }),
        new MacroOperationDescription(
            MacroOperation.ElevatorForceUp,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_14,
            Shift.Union(Shift.AllShifts),
            Shift.Debug,
            ButtonType.Simple,
            () -> new ElevatorMovementTask(DigitalOperation.ElevatorForceUp),
            new IOperation[]
            {
                DigitalOperation.ElevatorMoveUp,
                DigitalOperation.ElevatorMoveDown,
                DigitalOperation.ElevatorForceUp,
                DigitalOperation.ElevatorForceDown,                               
            }),
        new MacroOperationDescription(
            MacroOperation.ElevatorForceDown,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_15,
            Shift.Union(Shift.AllShifts),
            Shift.Debug,
            ButtonType.Simple,
            () -> new ElevatorMovementTask(DigitalOperation.ElevatorForceDown),
            new IOperation[]
            {
                DigitalOperation.ElevatorMoveUp,
                DigitalOperation.ElevatorMoveDown,
                DigitalOperation.ElevatorForceUp,
                DigitalOperation.ElevatorForceDown,                               
            }),
        new MacroOperationDescription(
            MacroOperation.ElevatorBottomPosition,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_5,
            Shift.Union(Shift.AllShifts),
            Shift.None,
            ButtonType.Toggle,
            () -> new ElevatorPositionTask(DigitalOperation.ElevatorBottomPosition),
            new IOperation[]
            {
                DigitalOperation.ElevatorBottomPosition,
                DigitalOperation.ElevatorHatch2Position,
                DigitalOperation.ElevatorHatch3Position,
                DigitalOperation.ElevatorCargo1Position,
                DigitalOperation.ElevatorCargo2Position,
                DigitalOperation.ElevatorCargo3Position,
                DigitalOperation.ElevatorCargoLoadPosition, 
                DigitalOperation.ElevatorCamReturnPosition                  
            }),
        new MacroOperationDescription(
            MacroOperation.ElevatorHatch2Position,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_4,
            Shift.Union(Shift.AllShifts),
            Shift.None,
            ButtonType.Toggle,
            () -> new ElevatorPositionTask(DigitalOperation.ElevatorHatch2Position),
            new IOperation[]
            {
                DigitalOperation.ElevatorBottomPosition,
                DigitalOperation.ElevatorHatch2Position,
                DigitalOperation.ElevatorHatch3Position,
                DigitalOperation.ElevatorCargo1Position,
                DigitalOperation.ElevatorCargo2Position,
                DigitalOperation.ElevatorCargo3Position,
                DigitalOperation.ElevatorCargoLoadPosition,
                DigitalOperation.ElevatorCamReturnPosition                    
            }),
        new MacroOperationDescription(
            MacroOperation.ElevatorHatch3Position,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_3,
            Shift.Union(Shift.AllShifts),
            Shift.None,
            ButtonType.Toggle,
            () -> new ElevatorPositionTask(DigitalOperation.ElevatorHatch3Position),
            new IOperation[]
            {
                DigitalOperation.ElevatorBottomPosition,
                DigitalOperation.ElevatorHatch2Position,
                DigitalOperation.ElevatorHatch3Position,
                DigitalOperation.ElevatorCargo1Position,
                DigitalOperation.ElevatorCargo2Position,
                DigitalOperation.ElevatorCargo3Position,
                DigitalOperation.ElevatorCargoLoadPosition, 
                DigitalOperation.ElevatorCamReturnPosition,                  
            }),
        new MacroOperationDescription(
            MacroOperation.ElevatorCargo1PositionShifted,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_5,
            Shift.Union(Shift.AllShifts),
            Shift.Debug,
            ButtonType.Toggle,
            () -> ConcurrentTask.AllTasks(
                new ElevatorPositionTask(DigitalOperation.ElevatorCargo1Position),
                new GrabberSetWristPositionTask(DigitalOperation.GrabberWristCargoPosition)),
            new IOperation[]
            {
                DigitalOperation.ElevatorBottomPosition,
                DigitalOperation.ElevatorHatch2Position,
                DigitalOperation.ElevatorHatch3Position,
                DigitalOperation.ElevatorCargo1Position,
                DigitalOperation.ElevatorCargo2Position,
                DigitalOperation.ElevatorCargo3Position,
                DigitalOperation.ElevatorCargoLoadPosition,
                DigitalOperation.ElevatorCamReturnPosition,
                DigitalOperation.GrabberWristStartPosition,
                DigitalOperation.GrabberWristHatchPosition,
                DigitalOperation.GrabberWristCargoPosition,
                DigitalOperation.GrabberWristFloorPosition,
            
            }),
        new MacroOperationDescription(
            MacroOperation.ElevatorCargo2PositionShifted,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_4,
            Shift.Union(Shift.AllShifts),
            Shift.Debug,
            ButtonType.Toggle,
            () -> ConcurrentTask.AllTasks(
                new ElevatorPositionTask(DigitalOperation.ElevatorCargo2Position),
                new GrabberSetWristPositionTask(DigitalOperation.GrabberWristCargoPosition)),
            new IOperation[]
            {
                DigitalOperation.ElevatorBottomPosition,
                DigitalOperation.ElevatorHatch2Position,
                DigitalOperation.ElevatorHatch3Position,
                DigitalOperation.ElevatorCargo1Position,
                DigitalOperation.ElevatorCargo2Position,
                DigitalOperation.ElevatorCargo3Position,
                DigitalOperation.ElevatorCargoLoadPosition,
                DigitalOperation.ElevatorCamReturnPosition,
                DigitalOperation.GrabberWristStartPosition,
                DigitalOperation.GrabberWristHatchPosition,
                DigitalOperation.GrabberWristCargoPosition,
                DigitalOperation.GrabberWristFloorPosition,
            }),
        new MacroOperationDescription(
            MacroOperation.ElevatorCargo3PositionShifted,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_3,
            Shift.Union(Shift.AllShifts),
            Shift.Debug,
            ButtonType.Toggle,
            () -> ConcurrentTask.AllTasks(
                new ElevatorPositionTask(DigitalOperation.ElevatorCargo3Position),
                new GrabberSetWristPositionTask(DigitalOperation.GrabberWristCargoPosition)),
            new IOperation[]
            {
                DigitalOperation.ElevatorBottomPosition,
                DigitalOperation.ElevatorHatch2Position,
                DigitalOperation.ElevatorHatch3Position,
                DigitalOperation.ElevatorCargo1Position,
                DigitalOperation.ElevatorCargo2Position,
                DigitalOperation.ElevatorCargo3Position,
                DigitalOperation.ElevatorCargoLoadPosition,
                DigitalOperation.ElevatorCamReturnPosition,
                DigitalOperation.GrabberWristStartPosition,
                DigitalOperation.GrabberWristHatchPosition,
                DigitalOperation.GrabberWristCargoPosition,
                DigitalOperation.GrabberWristFloorPosition,
            }),
        new MacroOperationDescription(
            MacroOperation.ElevatorCargoLoadPositionShifted,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_2,
            Shift.Union(Shift.AllShifts),
            Shift.Debug,
            ButtonType.Toggle,
            () -> ConcurrentTask.AllTasks(
                new ElevatorPositionTask(DigitalOperation.ElevatorCargoLoadPosition),
                new GrabberSetWristPositionTask(DigitalOperation.GrabberWristCargoPosition)),
            new IOperation[]
            {
                DigitalOperation.ElevatorBottomPosition,
                DigitalOperation.ElevatorHatch2Position,
                DigitalOperation.ElevatorHatch3Position,
                DigitalOperation.ElevatorCargo1Position,
                DigitalOperation.ElevatorCargo2Position,
                DigitalOperation.ElevatorCargo3Position,
                DigitalOperation.ElevatorCargoLoadPosition,
                DigitalOperation.ElevatorCamReturnPosition,
                DigitalOperation.GrabberWristStartPosition,
                DigitalOperation.GrabberWristHatchPosition,
                DigitalOperation.GrabberWristCargoPosition,
                DigitalOperation.GrabberWristFloorPosition
            }),

        // Climber Macros
        new MacroOperationDescription(
            MacroOperation.ClimberArmsMoveForward,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_6,
            Shift.Union(Shift.AllShifts),
            Shift.ButtonPadDebug,
            ButtonType.Simple,
            () -> new ClimberArmsMovementTask(DigitalOperation.ClimberArmsMoveForward),
            new IOperation[]
            {
                DigitalOperation.ClimberArmsMoveForward,
                DigitalOperation.ClimberArmsMoveBackward,
                DigitalOperation.ClimberArmsForceForward,
                DigitalOperation.ClimberArmsForceBackward,
                DigitalOperation.ClimberArmsForceZero
            }),
        new MacroOperationDescription(
            MacroOperation.ClimberArmsMoveBackward,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_7,
            Shift.Union(Shift.AllShifts),
            Shift.ButtonPadDebug,
            ButtonType.Simple,
            () -> new ClimberArmsMovementTask(DigitalOperation.ClimberArmsMoveBackward),
            new IOperation[]
            {
                DigitalOperation.ClimberArmsMoveForward,
                DigitalOperation.ClimberArmsMoveBackward,
                DigitalOperation.ClimberArmsForceForward,
                DigitalOperation.ClimberArmsForceBackward,
                DigitalOperation.ClimberArmsForceZero
            }),
        new MacroOperationDescription(
            MacroOperation.ClimberArmsForceForward,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_1,
            Shift.Union(Shift.AllShifts),
            Shift.ButtonPadDebug,
            ButtonType.Simple,
            () -> new ClimberArmsMovementTask(DigitalOperation.ClimberArmsForceForward),
            new IOperation[]
            {
                DigitalOperation.ClimberArmsMoveForward,
                DigitalOperation.ClimberArmsMoveBackward,
                DigitalOperation.ClimberArmsForceForward,
                DigitalOperation.ClimberArmsForceBackward,
                DigitalOperation.ClimberArmsForceZero
            }),
        new MacroOperationDescription(
            MacroOperation.ClimberArmsForceBackward,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_2,
            Shift.Union(Shift.AllShifts),
            Shift.ButtonPadDebug,
            ButtonType.Simple,
            () -> new ClimberArmsMovementTask(DigitalOperation.ClimberArmsForceBackward),
            new IOperation[]
            {
                DigitalOperation.ClimberArmsMoveForward,
                DigitalOperation.ClimberArmsMoveBackward,
                DigitalOperation.ClimberArmsForceForward,
                DigitalOperation.ClimberArmsForceBackward,
                DigitalOperation.ClimberArmsForceZero
            }),
        new MacroOperationDescription(
            MacroOperation.ClimberArmsForceZero,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_5,
            Shift.Union(Shift.AllShifts),
            Shift.ButtonPadDebug,
            ButtonType.Simple,
            () -> new ClimberArmsMovementTask(DigitalOperation.ClimberArmsForceZero),
            new IOperation[]
            {
                DigitalOperation.ClimberArmsMoveForward,
                DigitalOperation.ClimberArmsMoveBackward,
                DigitalOperation.ClimberArmsForceForward,
                DigitalOperation.ClimberArmsForceBackward,
                DigitalOperation.ClimberArmsForceZero
            }),
        new MacroOperationDescription(
            MacroOperation.ClimberCamMoveForward,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_8,
            Shift.Union(Shift.AllShifts),
            Shift.ButtonPadDebug,
            ButtonType.Simple,
            () -> new ClimberCamMovementTask(DigitalOperation.ClimberCamMoveForward),
            new IOperation[]
            {
                DigitalOperation.ClimberCamMoveForward,
                DigitalOperation.ClimberCamMoveBackward,
                DigitalOperation.ClimberCamForceForward,
                DigitalOperation.ClimberCamForceBackward
            }),
        new MacroOperationDescription(
            MacroOperation.ClimberCamMoveBackward,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_9,
            Shift.Union(Shift.AllShifts),
            Shift.ButtonPadDebug,
            ButtonType.Simple,
            () -> new ClimberCamMovementTask(DigitalOperation.ClimberCamMoveBackward),
            new IOperation[]
            {
                DigitalOperation.ClimberCamMoveForward,
                DigitalOperation.ClimberCamMoveBackward,
                DigitalOperation.ClimberCamForceForward,
                DigitalOperation.ClimberCamForceBackward                            
            }),
        new MacroOperationDescription(
            MacroOperation.ClimberCamForceForward,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_3,
            Shift.Union(Shift.AllShifts),
            Shift.ButtonPadDebug,
            ButtonType.Simple,
            () -> new ClimberCamMovementTask(DigitalOperation.ClimberCamForceForward),
            new IOperation[]
            {
                DigitalOperation.ClimberCamMoveForward,
                DigitalOperation.ClimberCamMoveBackward,
                DigitalOperation.ClimberCamForceForward,
                DigitalOperation.ClimberCamForceBackward                            
            }),
        new MacroOperationDescription(
            MacroOperation.ClimberCamForceBackward,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_4,
            Shift.Union(Shift.AllShifts),
            Shift.ButtonPadDebug,
            ButtonType.Simple,
            () -> new ClimberCamMovementTask(DigitalOperation.ClimberCamForceBackward),
            new IOperation[]
            {
                DigitalOperation.ClimberCamMoveForward,
                DigitalOperation.ClimberCamMoveBackward,
                DigitalOperation.ClimberCamForceForward,
                DigitalOperation.ClimberCamForceBackward                            
            }),

        // Elevator and Arm Macros
        new MacroOperationDescription(
            MacroOperation.ConfigCargoLoading,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_8,
            Shift.Union(Shift.AllShifts),
            Shift.None,
            ButtonType.Toggle,
            () -> ConcurrentTask.AllTasks(
                new ElevatorPositionTask(DigitalOperation.ElevatorCargoLoadPosition),
                new GrabberSetWristPositionTask(DigitalOperation.GrabberWristCargoPosition)),
            new IOperation[]
            {
                DigitalOperation.ElevatorBottomPosition,
                DigitalOperation.ElevatorHatch2Position,
                DigitalOperation.ElevatorHatch3Position,
                DigitalOperation.ElevatorCargo1Position,
                DigitalOperation.ElevatorCargo2Position,
                DigitalOperation.ElevatorCargo3Position,
                DigitalOperation.ElevatorCargoLoadPosition,
                DigitalOperation.ElevatorCamReturnPosition,
                DigitalOperation.GrabberWristStartPosition,
                DigitalOperation.GrabberWristHatchPosition,
                DigitalOperation.GrabberWristCargoPosition,
                DigitalOperation.GrabberWristFloorPosition,
            },
            new IOperation[]
            {
                DigitalOperation.ElevatorBottomPosition,
                DigitalOperation.ElevatorHatch2Position,
                DigitalOperation.ElevatorHatch3Position,
                DigitalOperation.ElevatorCargo1Position,
                DigitalOperation.ElevatorCargo2Position,
                DigitalOperation.ElevatorCargo3Position,
                DigitalOperation.ElevatorCargoLoadPosition,
                DigitalOperation.ElevatorCamReturnPosition,
                DigitalOperation.GrabberWristStartPosition,
                DigitalOperation.GrabberWristHatchPosition,
                DigitalOperation.GrabberWristCargoPosition,
                DigitalOperation.GrabberWristFloorPosition,
            }),
        new MacroOperationDescription(
            MacroOperation.ConfigCargoGround,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_9,
            Shift.Union(Shift.AllShifts),
            Shift.None,
            ButtonType.Toggle,
            () -> ConcurrentTask.AllTasks(
                new ElevatorPositionTask(DigitalOperation.ElevatorBottomPosition),
                new GrabberSetWristPositionTask(DigitalOperation.GrabberWristFloorPosition)),
            new IOperation[]
            {
                DigitalOperation.ElevatorBottomPosition,
                DigitalOperation.ElevatorHatch2Position,
                DigitalOperation.ElevatorHatch3Position,
                DigitalOperation.ElevatorCargo1Position,
                DigitalOperation.ElevatorCargo2Position,
                DigitalOperation.ElevatorCargo3Position,
                DigitalOperation.ElevatorCargoLoadPosition,
                DigitalOperation.ElevatorCamReturnPosition,
                DigitalOperation.GrabberWristStartPosition,
                DigitalOperation.GrabberWristHatchPosition,
                DigitalOperation.GrabberWristCargoPosition,
                DigitalOperation.GrabberWristFloorPosition,
            },
            new IOperation[]
            {
                DigitalOperation.ElevatorBottomPosition,
                DigitalOperation.ElevatorHatch2Position,
                DigitalOperation.ElevatorHatch3Position,
                DigitalOperation.ElevatorCargo1Position,
                DigitalOperation.ElevatorCargo2Position,
                DigitalOperation.ElevatorCargo3Position,
                DigitalOperation.ElevatorCargoLoadPosition,
                DigitalOperation.ElevatorCamReturnPosition,
                DigitalOperation.GrabberWristStartPosition,
                DigitalOperation.GrabberWristHatchPosition,
                DigitalOperation.GrabberWristCargoPosition,
                DigitalOperation.GrabberWristFloorPosition,
            }),
        new MacroOperationDescription(
            MacroOperation.ConfigHatchLoading,
            UserInputDevice.CoDriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_10,
            Shift.Union(Shift.AllShifts),
            Shift.None,
            ButtonType.Toggle,
            () -> ConcurrentTask.AllTasks(
                new ElevatorPositionTask(DigitalOperation.ElevatorBottomPosition),
                new GrabberSetWristPositionTask(DigitalOperation.GrabberWristHatchPosition)),
            new IOperation[]
            {
                DigitalOperation.ElevatorBottomPosition,
                DigitalOperation.ElevatorHatch2Position,
                DigitalOperation.ElevatorHatch3Position,
                DigitalOperation.ElevatorCargo1Position,
                DigitalOperation.ElevatorCargo2Position,
                DigitalOperation.ElevatorCargo3Position,
                DigitalOperation.ElevatorCargoLoadPosition,
                DigitalOperation.ElevatorCamReturnPosition,
                DigitalOperation.GrabberWristStartPosition,
                DigitalOperation.GrabberWristHatchPosition,
                DigitalOperation.GrabberWristCargoPosition,
                DigitalOperation.GrabberWristFloorPosition,
            },
            new IOperation[]
            {
                DigitalOperation.ElevatorBottomPosition,
                DigitalOperation.ElevatorHatch2Position,
                DigitalOperation.ElevatorHatch3Position,
                DigitalOperation.ElevatorCargo1Position,
                DigitalOperation.ElevatorCargo2Position,
                DigitalOperation.ElevatorCargo3Position,
                DigitalOperation.ElevatorCargoLoadPosition,
                DigitalOperation.ElevatorCamReturnPosition,
                DigitalOperation.GrabberWristStartPosition,
                DigitalOperation.GrabberWristHatchPosition,
                DigitalOperation.GrabberWristCargoPosition,
                DigitalOperation.GrabberWristFloorPosition,
            }),
    };

    @Override
    public ShiftDescription[] getShiftSchema()
    {
        return ButtonMap.ShiftButtonSchema;
    }

    @Override
    public AnalogOperationDescription[] getAnalogOperationSchema()
    {
        return ButtonMap.AnalogOperationSchema;
    }

    @Override
    public DigitalOperationDescription[] getDigitalOperationSchema()
    {
        return ButtonMap.DigitalOperationSchema;
    }

    @Override
    public MacroOperationDescription[] getMacroOperationSchema()
    {
        return ButtonMap.MacroSchema;
    }
}
