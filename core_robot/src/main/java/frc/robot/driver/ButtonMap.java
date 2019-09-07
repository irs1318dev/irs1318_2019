package frc.robot.driver;

import java.util.HashMap;
import java.util.Map;

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
import frc.robot.driver.common.descriptions.OperationDescription;
import frc.robot.driver.common.descriptions.ShiftDescription;
import frc.robot.driver.common.descriptions.UserInputDevice;
import frc.robot.driver.controltasks.*;

@Singleton
public class ButtonMap implements IButtonMap
{
    @SuppressWarnings("serial")
    private static Map<Shift, ShiftDescription> ShiftButtons = new HashMap<Shift, ShiftDescription>()
    {
        {
            put(
                Shift.DriverDebug,
                new ShiftDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.XBONE_LEFT_BUTTON));
            put(
                Shift.DriverTestingDebug,
                new ShiftDescription(
                    UserInputDevice.Driver,
                    0)); // POV forward/up
            put(
                Shift.OperatorDebug,
                new ShiftDescription(
                    UserInputDevice.Operator,
                    UserInputDeviceButton.PS4_OPTIONS_BUTTON));
        }
    };

    @SuppressWarnings("serial")
    public static Map<Operation, OperationDescription> OperationSchema = new HashMap<Operation, OperationDescription>()
    {
        {
            // Operations for vision
            put(
                Operation.VisionDisable,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    Shift.Any,
                    ButtonType.Click));
            put(
                Operation.VisionForceDisable,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    Shift.Any,
                    ButtonType.Click));
            put(
                Operation.VisionEnableCargoShip,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    Shift.Any,
                    ButtonType.Click));
            put(
                Operation.VisionEnableRocket,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    Shift.Any,
                    ButtonType.Click));
            put(
                Operation.VisionEnableOffboardStream,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    Shift.Any,
                    ButtonType.Toggle));
            put(
                Operation.VisionEnableOffboardProcessing,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    0,
                    Shift.DriverDebug,
                    ButtonType.Toggle));

            // Operations for the compressor
            put(
                Operation.CompressorForceDisable,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    Shift.Any,
                    ButtonType.Click));

            // Operations for the drive train
            put(
                Operation.DriveTrainDisablePID,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Click));
            put(
                Operation.DriveTrainEnablePID,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Click));
            put(
                Operation.DriveTrainMoveForward,
                new AnalogOperationDescription(
                    UserInputDevice.Driver,
                    AnalogAxis.XBONE_LEFT_STICK_Y,
                    ElectronicsConstants.INVERT_Y_AXIS,
                    TuningConstants.DRIVETRAIN_Y_DEAD_ZONE));
            put(
                Operation.DriveTrainTurn,
                new AnalogOperationDescription(
                    UserInputDevice.Driver,
                    AnalogAxis.XBONE_RIGHT_STICK_X,
                    ElectronicsConstants.INVERT_X_AXIS,
                    TuningConstants.DRIVETRAIN_X_DEAD_ZONE));
            put(
                Operation.DriveTrainSimpleMode,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Toggle));
            put(
                Operation.DriveTrainUsePositionalMode,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Toggle));
            put(
                Operation.DriveTrainUseBrakeMode,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Toggle));
            put(
                Operation.DriveTrainLeftPosition,
                new AnalogOperationDescription(
                    UserInputDevice.None,
                    AnalogAxis.NONE,
                    false,
                    0.0));
            put(
                Operation.DriveTrainRightPosition,
                new AnalogOperationDescription(
                    UserInputDevice.None,
                    AnalogAxis.NONE,
                    false,
                    0.0));
            put(
                Operation.DriveTrainLeftVelocity,
                new AnalogOperationDescription(
                    UserInputDevice.None,
                    AnalogAxis.NONE,
                    false,
                    0.0));
            put(
                Operation.DriveTrainRightVelocity,
                new AnalogOperationDescription(
                    UserInputDevice.None,
                    AnalogAxis.NONE,
                    false,
                    0.0));
            put(
                Operation.DriveTrainHeadingCorrection,
                new AnalogOperationDescription(
                    UserInputDevice.None,
                    AnalogAxis.NONE,
                    false,
                    0.0));
            put(
                Operation.DriveTrainSwapFrontOrientation,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Toggle));
            put(
                Operation.DriveTrainUseSimplePathMode,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Toggle));
            put(
                Operation.DriveTrainUsePathMode,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Toggle));
            put(
                Operation.PositionStartingAngle,
                new AnalogOperationDescription(
                    UserInputDevice.None,
                    AnalogAxis.NONE,
                    false,
                    0.0));

            // Operations for the elevator
            put(
                Operation.ElevatorForceUp,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    Shift.Any,
                    ButtonType.Simple));
            put(
                Operation.ElevatorForceDown,
                new DigitalOperationDescription(
                    UserInputDevice.Operator,
                    UserInputDeviceButton.PS4_SHARE_BUTTON,
                    Shift.Any,
                    ButtonType.Simple));
            put(
                Operation.ElevatorMoveUp,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    Shift.Any,
                    ButtonType.Simple));
            put(
                Operation.ElevatorMoveDown,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    Shift.Any,
                    ButtonType.Simple));  
            put(
                Operation.ElevatorBottomPosition,
                new DigitalOperationDescription(
                    UserInputDevice.Operator,
                    270,
                    Shift.None,
                    ButtonType.Click)); 
            put(
                Operation.ElevatorHatch2Position,
                new DigitalOperationDescription(
                    UserInputDevice.Operator,
                    0,
                    Shift.None,
                    ButtonType.Click));
            put(
                Operation.ElevatorHatch3Position,
                new DigitalOperationDescription(
                    UserInputDevice.Operator,
                    90,
                    Shift.None,
                    ButtonType.Click)); 
            put(
                Operation.ElevatorCargo1Position,
                new DigitalOperationDescription(
                    UserInputDevice.Operator,
                    270,
                    Shift.OperatorDebug,
                    ButtonType.Click)); 
            put(
                Operation.ElevatorCargo2Position,
                new DigitalOperationDescription(
                    UserInputDevice.Operator,
                    0,
                    Shift.OperatorDebug,
                    ButtonType.Click)); 
            put(
                Operation.ElevatorCargo3Position,
                new DigitalOperationDescription(
                    UserInputDevice.Operator,
                    90,
                    Shift.OperatorDebug,
                    ButtonType.Click));
            put(
                Operation.ElevatorCargoLoadPosition,
                new DigitalOperationDescription(
                    UserInputDevice.Operator,
                    180,
                    Shift.OperatorDebug,
                    ButtonType.Click));  
            put(
                Operation.ElevatorCamReturnPosition,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    Shift.Any,
                    ButtonType.Click));

            // Operations for grabber mechanism
            put(
                Operation.GrabberIntakeCargo,
                new DigitalOperationDescription(
                    UserInputDevice.Operator,
                    UserInputDeviceButton.PS4_LEFT_BUTTON,
                    Shift.Any,
                    ButtonType.Simple)); 
            put(
                Operation.GrabberOuttakeCargo,
                new DigitalOperationDescription(
                    UserInputDevice.Operator,
                    AnalogAxis.PS4_LEFT_TRIGGER,
                    0.5,
                    1.0,
                    Shift.Any,
                    ButtonType.Simple));
            put(
                Operation.GrabberPointBeak,
                new DigitalOperationDescription(
                    UserInputDevice.Operator,
                    UserInputDeviceButton.PS4_RIGHT_BUTTON,
                    Shift.Any,
                    ButtonType.Simple));
            put(
                Operation.GrabberKickPanel,
                new DigitalOperationDescription(
                    UserInputDevice.Operator,
                    AnalogAxis.PS4_RIGHT_TRIGGER,
                    0.5,
                    1.0,
                    Shift.None,
                    ButtonType.Simple));
            put(
                Operation.GrabberWristStartPosition,
                new DigitalOperationDescription(
                    UserInputDevice.Operator,
                    UserInputDeviceButton.PS4_X_BUTTON,
                    Shift.Any,
                    ButtonType.Click));
            put(
                Operation.GrabberWristHatchPosition,
                new DigitalOperationDescription(
                    UserInputDevice.Operator,
                    UserInputDeviceButton.PS4_CIRCLE_BUTTON,
                    Shift.Any,
                    ButtonType.Click));
            put(
                Operation.GrabberWristCargoPosition,
                new DigitalOperationDescription(
                    UserInputDevice.Operator,
                    UserInputDeviceButton.PS4_SQUARE_BUTTON,
                    Shift.Any,
                    ButtonType.Click));
            put(
                Operation.GrabberWristFloorPosition,
                new DigitalOperationDescription(
                    UserInputDevice.Operator,
                    UserInputDeviceButton.PS4_TRIANGLE_BUTTON,
                    Shift.Any,
                    ButtonType.Click));

            // Operations for the climber
            put(
                Operation.ClimberArmsMoveForward,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Simple));
            put(
                Operation.ClimberArmsMoveBackward,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Simple));
            put(
                Operation.ClimberCamMoveForward,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Simple));
            put(
                Operation.ClimberCamMoveBackward,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Simple));
            put(
                Operation.ClimberArmsRetractedPosition,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Simple));
            put(
                Operation.ClimberArmsPrepClimbPosition,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Simple));
            put(
                Operation.ClimberArmsHighClimbPosition,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Simple));
            put(
                Operation.ClimberCamStoredPosition,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Simple));
            put(
                Operation.ClimberCamLowClimbPosition,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Simple));
            put(
                Operation.ClimberCamHighClimbPosition,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Simple));
            put(
                Operation.ClimberCamOutOfWayPosition,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Simple));
            put(
                Operation.ClimberArmsForceForward,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.XBONE_LEFT_BUTTON,
                    Shift.DriverTestingDebug,
                    ButtonType.Simple));
            put(
                Operation.ClimberArmsForceBackward,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    AnalogAxis.XBONE_LEFT_TRIGGER,
                    0.5,
                    1.0,
                    Shift.DriverTestingDebug,
                    ButtonType.Simple));
            put(
                Operation.ClimberArmsForceZero,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Simple));
            put(
                Operation.ClimberCamForceForward,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.XBONE_RIGHT_BUTTON,
                    Shift.DriverTestingDebug,
                    ButtonType.Simple));
            put(
                Operation.ClimberCamForceBackward,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    AnalogAxis.XBONE_RIGHT_TRIGGER,
                    0.5,
                    1.0,
                    Shift.DriverTestingDebug,
                    ButtonType.Simple));
        }
    };

    @SuppressWarnings("serial")
    public static Map<MacroOperation, MacroOperationDescription> MacroSchema = new HashMap<MacroOperation, MacroOperationDescription>()
    {
        {
            // Brake mode macro
            put(
                MacroOperation.PIDBrake,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    AnalogAxis.XBONE_LEFT_TRIGGER,
                    0.5,
                    1.0,
                    Shift.Any,
                    ButtonType.Simple,
                    () -> new PIDBrakeTask(),
                    new Operation[]
                    {
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainUseBrakeMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                    }));

            // Driving Macros
            put(
                MacroOperation.TurnInPlaceRight,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.XBONE_B_BUTTON,
                    Shift.Any,
                    ButtonType.Toggle,
                    () -> new NavxTurnTask(true, 180, 3.0, true, false),
                    new Operation[]
                    {
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainUseBrakeMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                        Operation.DriveTrainLeftVelocity,
                        Operation.DriveTrainRightVelocity,
                        Operation.DriveTrainTurn,
                        Operation.DriveTrainMoveForward,
                        Operation.DriveTrainSimpleMode,
                    },
                    new Operation[]
                    {
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainUseBrakeMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                    }));
            put(
                MacroOperation.TurnInPlaceLeft,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.XBONE_X_BUTTON,
                    Shift.Any,
                    ButtonType.Toggle,
                    () -> new NavxTurnTask(true, -180, TuningConstants.NAVX_FAST_TURN_TIMEOUT, true, true),
                    new Operation[]
                    {
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainUseBrakeMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                        Operation.DriveTrainTurn,
                        Operation.DriveTrainMoveForward,
                        Operation.DriveTrainSimpleMode,
                    },
                    new Operation[]
                    {
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainUseBrakeMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                    }));
            put(
                MacroOperation.FollowSomePath,
                new MacroOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    Shift.Any,
                    ButtonType.Toggle,
                    () -> new FollowPathTask("/Paths/Circle 40 inch radius.csv"),
                    new Operation[]
                    {
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainUseBrakeMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                        Operation.DriveTrainLeftVelocity,
                        Operation.DriveTrainRightVelocity,
                        Operation.DriveTrainHeadingCorrection,
                        Operation.DriveTrainUsePathMode,
                        Operation.DriveTrainTurn,
                        Operation.DriveTrainMoveForward,
                        Operation.DriveTrainSimpleMode,
                    },
                    new Operation[]
                    {
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainUseBrakeMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                    }));

            // Climbing Macros
            put(
                MacroOperation.ClimbHabAndReset,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.XBONE_RIGHT_BUTTON,
                    Shift.DriverDebug,
                    ButtonType.Toggle,
                    () -> ConcurrentTask.AnyTasks(
                        SequentialTask.Sequence(
                            ConcurrentTask.AllTasks(
                                new ElevatorPositionTask(Operation.ElevatorBottomPosition),
                                new GrabberSetWristPositionTask(Operation.GrabberWristStartPosition),
                                new ClimberArmsPositionTask(Operation.ClimberArmsPrepClimbPosition)),
                            ConcurrentTask.AllTasks(
                                new ClimberArmsPositionTask(1.0, Operation.ClimberArmsHighClimbPosition),
                                new ClimberCamPositionTask(1.0, Operation.ClimberCamHighClimbPosition)),
                            ConcurrentTask.AllTasks(
                                new ClimberCamPositionTask(1.0, Operation.ClimberCamOutOfWayPosition),
                                new ClimberArmsPositionTask(0.25, Operation.ClimberArmsRetractedPosition),
                                new DriveUntilSensorTask(-0.15, 1.0)),
                            ConcurrentTask.AllTasks(
                                SequentialTask.Sequence(
                                    new ElevatorPositionTask(1.0, Operation.ElevatorCamReturnPosition),
                                    new ClimberCamPositionTask(1.0, Operation.ClimberCamStoredPosition),
                                    new ElevatorPositionTask(Operation.ElevatorBottomPosition)),
                                new DriveDistanceTimedTask(-6.0, 1.5))),
                        new VisionDisableTask(),
                        new CompressorDisableTask()),
                    new Operation[]
                    {
                        Operation.CompressorForceDisable,
                        Operation.VisionForceDisable,
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainUseBrakeMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                        Operation.DriveTrainLeftVelocity,
                        Operation.DriveTrainRightVelocity,
                        Operation.DriveTrainHeadingCorrection,
                        Operation.DriveTrainUsePathMode,
                        Operation.DriveTrainTurn,
                        Operation.DriveTrainMoveForward,
                        Operation.DriveTrainSimpleMode,
                        Operation.DriveTrainUseSimplePathMode,
                        Operation.ClimberArmsRetractedPosition,
                        Operation.ClimberArmsPrepClimbPosition,
                        Operation.ClimberArmsHighClimbPosition,
                        Operation.ClimberCamStoredPosition,
                        Operation.ClimberCamLowClimbPosition,
                        Operation.ClimberCamHighClimbPosition,
                        Operation.ClimberCamOutOfWayPosition,
                        Operation.ClimberArmsMoveBackward,
                        Operation.ClimberArmsMoveForward,
                        Operation.ClimberCamMoveBackward,
                        Operation.ClimberCamMoveForward,
                        Operation.ClimberArmsForceBackward,
                        Operation.ClimberArmsForceForward,
                        Operation.ClimberCamForceBackward,
                        Operation.ClimberCamForceForward,
                        Operation.ElevatorBottomPosition,
                        Operation.ElevatorHatch2Position,
                        Operation.ElevatorHatch3Position,
                        Operation.ElevatorCargo1Position,
                        Operation.ElevatorCargo2Position,
                        Operation.ElevatorCargo3Position,
                        Operation.ElevatorCargoLoadPosition,
                        Operation.ElevatorCamReturnPosition,
                        Operation.ElevatorMoveUp,
                        Operation.ElevatorMoveDown,
                        Operation.ElevatorForceUp,
                        Operation.ElevatorForceDown,
                        Operation.GrabberWristStartPosition,
                        Operation.GrabberWristHatchPosition,
                        Operation.GrabberWristCargoPosition,
                        Operation.GrabberWristFloorPosition
                    },
                    new Operation[]
                    {
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainUseBrakeMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                        Operation.ClimberArmsMoveBackward,
                        Operation.ClimberArmsMoveForward,
                        Operation.ClimberCamMoveBackward,
                        Operation.ClimberCamMoveForward,
                        Operation.ClimberArmsForceBackward,
                        Operation.ClimberArmsForceForward,
                        Operation.ClimberCamForceBackward,
                        Operation.ClimberCamForceForward,
                    }));
            put(
                MacroOperation.ClimbHabLeaveTailDown,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    AnalogAxis.XBONE_RIGHT_TRIGGER,
                    0.5,
                    1.0,
                    Shift.DriverDebug,
                    ButtonType.Toggle,
                    () -> ConcurrentTask.AnyTasks(
                        SequentialTask.Sequence(
                            ConcurrentTask.AllTasks(
                                new ElevatorPositionTask(Operation.ElevatorBottomPosition),
                                new GrabberSetWristPositionTask(Operation.GrabberWristStartPosition),
                                new ClimberArmsPositionTask(Operation.ClimberArmsPrepClimbPosition)),
                            ConcurrentTask.AllTasks(
                                new ClimberArmsPositionTask(1.0, Operation.ClimberArmsHighClimbPosition),
                                new ClimberCamPositionTask(1.0, Operation.ClimberCamHighClimbPosition)),
                            ConcurrentTask.AllTasks(
                                new ClimberCamPositionTask(1.0, Operation.ClimberCamOutOfWayPosition),
                                new ClimberArmsPositionTask(0.25, Operation.ClimberArmsRetractedPosition),
                                new DriveUntilSensorTask(-0.15, 1.0))),
                        new VisionDisableTask(),
                        new CompressorDisableTask()),
                    new Operation[]
                    {
                        Operation.CompressorForceDisable,
                        Operation.VisionForceDisable,
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainUseBrakeMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                        Operation.DriveTrainLeftVelocity,
                        Operation.DriveTrainRightVelocity,
                        Operation.DriveTrainHeadingCorrection,
                        Operation.DriveTrainUsePathMode,
                        Operation.DriveTrainTurn,
                        Operation.DriveTrainMoveForward,
                        Operation.DriveTrainSimpleMode,
                        Operation.DriveTrainUseSimplePathMode,
                        Operation.ClimberArmsRetractedPosition,
                        Operation.ClimberArmsPrepClimbPosition,
                        Operation.ClimberArmsHighClimbPosition,
                        Operation.ClimberCamStoredPosition,
                        Operation.ClimberCamLowClimbPosition,
                        Operation.ClimberCamHighClimbPosition,
                        Operation.ClimberCamOutOfWayPosition,
                        Operation.ClimberArmsMoveBackward,
                        Operation.ClimberArmsMoveForward,
                        Operation.ClimberCamMoveBackward,
                        Operation.ClimberCamMoveForward,
                        Operation.ClimberArmsForceBackward,
                        Operation.ClimberArmsForceForward,
                        Operation.ClimberCamForceBackward,
                        Operation.ClimberCamForceForward,
                        Operation.ElevatorBottomPosition,
                        Operation.ElevatorHatch2Position,
                        Operation.ElevatorHatch3Position,
                        Operation.ElevatorCargo1Position,
                        Operation.ElevatorCargo2Position,
                        Operation.ElevatorCargo3Position,
                        Operation.ElevatorCargoLoadPosition,
                        Operation.ElevatorCamReturnPosition,
                        Operation.ElevatorMoveUp,
                        Operation.ElevatorMoveDown,
                        Operation.ElevatorForceUp,
                        Operation.ElevatorForceDown,
                        Operation.GrabberWristStartPosition,
                        Operation.GrabberWristHatchPosition,
                        Operation.GrabberWristCargoPosition,
                        Operation.GrabberWristFloorPosition
                    },
                    new Operation[]
                    {
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainUseBrakeMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                        Operation.ClimberArmsMoveBackward,
                        Operation.ClimberArmsMoveForward,
                        Operation.ClimberCamMoveBackward,
                        Operation.ClimberCamMoveForward,
                        Operation.ClimberArmsForceBackward,
                        Operation.ClimberArmsForceForward,
                        Operation.ClimberCamForceBackward,
                        Operation.ClimberCamForceForward,
                    }));
        
            // Vision Macros
            put(
                MacroOperation.VisionCenterAndAdvanceRocket,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.XBONE_A_BUTTON,
                    Shift.Any,
                    ButtonType.Toggle,
                    () -> new VisionAdvanceAndCenterTask(Operation.VisionEnableRocket),
                    new Operation[]
                    {
                        Operation.VisionDisable,
                        Operation.VisionEnableCargoShip,
                        Operation.VisionEnableRocket,
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                        Operation.DriveTrainTurn,
                        Operation.DriveTrainMoveForward
                    }));
            put(
                MacroOperation.VisionCenterAndAdvanceCargoShip,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.XBONE_Y_BUTTON,
                    Shift.DriverDebug,
                    ButtonType.Toggle,
                    () -> new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
                    new Operation[]
                    {
                        Operation.VisionDisable,
                        Operation.VisionEnableCargoShip,
                        Operation.VisionEnableRocket,
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                        Operation.DriveTrainTurn,
                        Operation.DriveTrainMoveForward
                    }));
            put(
                MacroOperation.VisionFastCenterAndAdvanceCargoShip,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.XBONE_Y_BUTTON,
                    Shift.Any,
                    ButtonType.Toggle,
                    () -> new VisionFastAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
                    new Operation[]
                    {
                        Operation.VisionDisable,
                        Operation.VisionEnableCargoShip,
                        Operation.VisionEnableRocket,
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                        Operation.DriveTrainTurn,
                        Operation.DriveTrainMoveForward
                    }));
            put(
                MacroOperation.VisionScanning,
                new MacroOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    Shift.Any,
                    ButtonType.Toggle,
                    () -> new VisionScanningTask(),
                    new Operation[]
                    {
                        Operation.VisionEnableOffboardProcessing,
                        Operation.DriveTrainTurn,
                        Operation.VisionDisable
                    }));

            // Grabber Macro
            put(
                MacroOperation.GrabberKickPanelRepeatedlyTask,
                new MacroOperationDescription(
                    UserInputDevice.Operator,
                    AnalogAxis.PS4_RIGHT_TRIGGER,
                    0.5,
                    1.0,
                    Shift.OperatorDebug,
                    ButtonType.Simple,
                    () -> new GrabberKickPanelRepeatedlyTask(),
                    new Operation[]
                    {
                        Operation.GrabberKickPanel,
                    }));

            // Climber Macros
            put(
                MacroOperation.ClimberArmsRetractedPosition,
                new MacroOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    Shift.DriverDebug,
                    ButtonType.Toggle,
                    () -> new ClimberArmsPositionTask(Operation.ClimberArmsRetractedPosition),
                    new Operation[]
                    {
                        Operation.ClimberArmsRetractedPosition,
                        Operation.ClimberArmsPrepClimbPosition,
                        Operation.ClimberArmsHighClimbPosition  
                    }));
            put(
                MacroOperation.ClimberArmsLowClimbPosition,
                new MacroOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    Shift.DriverDebug,
                    ButtonType.Toggle,
                    () -> new ClimberArmsPositionTask(Operation.ClimberArmsPrepClimbPosition),
                    new Operation[]
                    {
                        Operation.ClimberArmsRetractedPosition,
                        Operation.ClimberArmsPrepClimbPosition,
                        Operation.ClimberArmsHighClimbPosition                                  
                    }));
            put(
                MacroOperation.ClimberArmsHighClimbPosition,
                new MacroOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    Shift.DriverDebug,
                    ButtonType.Toggle,
                    () -> new ClimberArmsPositionTask(Operation.ClimberArmsHighClimbPosition),
                    new Operation[]
                    {
                        Operation.ClimberArmsRetractedPosition,
                        Operation.ClimberArmsPrepClimbPosition,
                        Operation.ClimberArmsHighClimbPosition                                  
                    }));
            put(
                MacroOperation.ClimberCamStoredPosition,
                new MacroOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    Shift.DriverDebug,
                    ButtonType.Toggle,
                    () -> new ClimberCamPositionTask(0.1, Operation.ClimberCamStoredPosition),
                    new Operation[]
                    {
                        Operation.ClimberCamStoredPosition,
                        Operation.ClimberCamLowClimbPosition,
                        Operation.ClimberCamHighClimbPosition,                                   
                    }));
            put(
                MacroOperation.ClimberCamLowClimbPosition,
                new MacroOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    Shift.DriverDebug,
                    ButtonType.Toggle,
                    () -> new ClimberCamPositionTask(0.1, Operation.ClimberCamLowClimbPosition),
                    new Operation[]
                    {
                        Operation.ClimberCamStoredPosition,
                        Operation.ClimberCamLowClimbPosition,
                        Operation.ClimberCamHighClimbPosition,                                    
                    }));
            put(
                MacroOperation.ClimberCamHighClimbPosition,
                new MacroOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    Shift.DriverDebug,
                    ButtonType.Toggle,
                    () -> new ClimberCamPositionTask(0.1, Operation.ClimberCamHighClimbPosition),
                    new Operation[]
                    {
                        Operation.ClimberCamStoredPosition,
                        Operation.ClimberCamLowClimbPosition,
                        Operation.ClimberCamHighClimbPosition,                                   
                    }));
            put(
                MacroOperation.ClimberArmsMoveForward,
                new MacroOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    Shift.DriverDebug,
                    ButtonType.Simple,
                    () -> new ClimberArmsMovementTask(Operation.ClimberArmsMoveForward),
                    new Operation[]
                    {
                        Operation.ClimberArmsMoveForward,
                        Operation.ClimberArmsMoveBackward,
                        Operation.ClimberArmsForceForward,
                        Operation.ClimberArmsForceBackward,
                        Operation.ClimberArmsForceZero
                    }));
            put(
                MacroOperation.ClimberArmsMoveBackward,
                new MacroOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    Shift.DriverDebug,
                    ButtonType.Simple,
                    () -> new ClimberArmsMovementTask(Operation.ClimberArmsMoveBackward),
                    new Operation[]
                    {
                        Operation.ClimberArmsMoveForward,
                        Operation.ClimberArmsMoveBackward,
                        Operation.ClimberArmsForceForward,
                        Operation.ClimberArmsForceBackward,
                        Operation.ClimberArmsForceZero
                    }));
            put(
                MacroOperation.ClimberArmsForceForward,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    90,
                    Shift.DriverDebug,
                    ButtonType.Simple,
                    () -> new ClimberArmsMovementTask(Operation.ClimberArmsForceForward),
                    new Operation[]
                    {
                        Operation.ClimberArmsMoveForward,
                        Operation.ClimberArmsMoveBackward,
                        Operation.ClimberArmsForceForward,
                        Operation.ClimberArmsForceBackward,
                        Operation.ClimberArmsForceZero
                    }));
            put(
                MacroOperation.ClimberArmsForceBackward,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    270,
                    Shift.DriverDebug,
                    ButtonType.Simple,
                    () -> new ClimberArmsMovementTask(Operation.ClimberArmsForceBackward),
                    new Operation[]
                    {
                        Operation.ClimberArmsMoveForward,
                        Operation.ClimberArmsMoveBackward,
                        Operation.ClimberArmsForceForward,
                        Operation.ClimberArmsForceBackward,
                        Operation.ClimberArmsForceZero
                    }));
            put(
                MacroOperation.ClimberArmsForceZero,
                new MacroOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    Shift.DriverDebug,
                    ButtonType.Simple,
                    () -> new ClimberArmsMovementTask(Operation.ClimberArmsForceZero),
                    new Operation[]
                    {
                        Operation.ClimberArmsMoveForward,
                        Operation.ClimberArmsMoveBackward,
                        Operation.ClimberArmsForceForward,
                        Operation.ClimberArmsForceBackward,
                        Operation.ClimberArmsForceZero
                    }));
            put(
                MacroOperation.ClimberCamMoveForward,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.NONE,
                    Shift.DriverDebug,
                    ButtonType.Simple,
                    () -> new ClimberCamMovementTask(Operation.ClimberCamMoveForward),
                    new Operation[]
                    {
                        Operation.ClimberCamMoveForward,
                        Operation.ClimberCamMoveBackward,
                        Operation.ClimberCamForceForward,
                        Operation.ClimberCamForceBackward
                    }));
            put(
                MacroOperation.ClimberCamMoveBackward,
                new MacroOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    Shift.DriverDebug,
                    ButtonType.Simple,
                    () -> new ClimberCamMovementTask(Operation.ClimberCamMoveBackward),
                    new Operation[]
                    {
                        Operation.ClimberCamMoveForward,
                        Operation.ClimberCamMoveBackward,
                        Operation.ClimberCamForceForward,
                        Operation.ClimberCamForceBackward                            
                    }));
            put(
                MacroOperation.ClimberCamForceForward,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    0,
                    Shift.DriverDebug,
                    ButtonType.Simple,
                    () -> new ClimberCamMovementTask(Operation.ClimberCamForceForward),
                    new Operation[]
                    {
                        Operation.ClimberCamMoveForward,
                        Operation.ClimberCamMoveBackward,
                        Operation.ClimberCamForceForward,
                        Operation.ClimberCamForceBackward                            
                    }));
            put(
                MacroOperation.ClimberCamForceBackward,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    180,
                    Shift.DriverDebug,
                    ButtonType.Simple,
                    () -> new ClimberCamMovementTask(Operation.ClimberCamForceBackward),
                    new Operation[]
                    {
                        Operation.ClimberCamMoveForward,
                        Operation.ClimberCamMoveBackward,
                        Operation.ClimberCamForceForward,
                        Operation.ClimberCamForceBackward                            
                    }));
        }
    };

    @Override
    public Map<Shift, ShiftDescription> getShiftMap()
    {
        return ButtonMap.ShiftButtons;
    }

    @Override
    public Map<Operation, OperationDescription> getOperationSchema()
    {
        return ButtonMap.OperationSchema;
    }

    @Override
    public Map<MacroOperation, MacroOperationDescription> getMacroOperationSchema()
    {
        return ButtonMap.MacroSchema;
    }
}
