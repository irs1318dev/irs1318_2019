package frc.robot.driver;

import java.util.HashMap;
import java.util.Map;

import javax.inject.Singleton;

import frc.robot.ElectronicsConstants;
import frc.robot.TuningConstants;
import frc.robot.common.robotprovider.AnalogAxis;
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
                Shift.Debug,
                new ShiftDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_STICK_TRIGGER_BUTTON));
            put(
                Shift.ButtonPadDebug,
                new ShiftDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_16));
        }
    };

    @SuppressWarnings("serial")
    public static Map<Operation, OperationDescription> OperationSchema = new HashMap<Operation, OperationDescription>()
    {
        {
            // Operations for vision
            put(
                Operation.VisionEnableCargoShip,
                new DigitalOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_11,
                    Shift.None,
                    ButtonType.Toggle));
            put(
                Operation.VisionEnableRocket,
                new DigitalOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_12,
                    Shift.None,
                    ButtonType.Toggle));
            put(
                Operation.VisionEnableOffboardStream,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    0,
                    Shift.Debug,
                    ButtonType.Toggle));
            put(
                Operation.VisionEnableOffboardProcessing,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    0,
                    Shift.Debug,
                    ButtonType.Toggle));

            // Operations for the drive train
            put(
                Operation.DriveTrainDisablePID,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_11,
                    ButtonType.Click));
            put(
                Operation.DriveTrainEnablePID,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_12,
                    ButtonType.Click));
            put(
                Operation.DriveTrainMoveForward,
                new AnalogOperationDescription(
                    UserInputDevice.Driver,
                    AnalogAxis.Y,
                    ElectronicsConstants.INVERT_Y_AXIS,
                    TuningConstants.DRIVETRAIN_Y_DEAD_ZONE));
            put(
                Operation.DriveTrainTurn,
                new AnalogOperationDescription(
                    UserInputDevice.Driver,
                    AnalogAxis.X,
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
                    AnalogAxis.None,
                    false,
                    0.0));
            put(
                Operation.DriveTrainRightPosition,
                new AnalogOperationDescription(
                    UserInputDevice.None,
                    AnalogAxis.None,
                    false,
                    0.0));
            put(
                Operation.DriveTrainLeftVelocity,
                new AnalogOperationDescription(
                    UserInputDevice.None,
                    AnalogAxis.None,
                    false,
                    0.0));
            put(
                Operation.DriveTrainRightVelocity,
                new AnalogOperationDescription(
                    UserInputDevice.None,
                    AnalogAxis.None,
                    false,
                    0.0));
            put(
                Operation.DriveTrainHeadingCorrection,
                new AnalogOperationDescription(
                    UserInputDevice.None,
                    AnalogAxis.None,
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

            // Operations for the elevator
            put(
                Operation.ElevatorForceUp,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_BASE_TOP_RIGHT_BUTTON,
                    Shift.Debug,
                    ButtonType.Simple));
            put(
                Operation.ElevatorForceDown,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_BASE_MIDDLE_RIGHT_BUTTON,
                    Shift.Debug,
                    ButtonType.Simple));
            put(
                Operation.ElevatorMoveUp,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_BASE_TOP_RIGHT_BUTTON,
                    Shift.None,
                    ButtonType.Simple));
            put(
                Operation.ElevatorMoveDown,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_BASE_MIDDLE_RIGHT_BUTTON,
                    Shift.None,
                    ButtonType.Simple));  
            put(
                Operation.ElevatorBottomPosition,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_BASE_BOTTOM_LEFT_BUTTON,
                    Shift.None,
                    ButtonType.Click)); 
            put(
                Operation.ElevatorHatch2Position,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_BASE_MIDDLE_LEFT_BUTTON,
                    Shift.None,
                    ButtonType.Click));
            put(
                Operation.ElevatorHatch3Position,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_BASE_TOP_LEFT_BUTTON,
                    Shift.None,
                    ButtonType.Click)); 
            put(
                Operation.ElevatorCargo1Position,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_BASE_BOTTOM_LEFT_BUTTON,
                    Shift.Debug,
                    ButtonType.Click)); 
            put(
                Operation.ElevatorCargo2Position,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_BASE_MIDDLE_LEFT_BUTTON,
                    Shift.Debug,
                    ButtonType.Click)); 
            put(
                Operation.ElevatorCargo3Position,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_BASE_TOP_LEFT_BUTTON,
                    Shift.Debug,
                    ButtonType.Click));
            put(
                Operation.ElevatorCargoLoadPosition,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_BASE_BOTTOM_RIGHT_BUTTON,
                    Shift.Debug,
                    ButtonType.Click)); 

            // Operations for grabber mechanism
            put(
                Operation.GrabberIntakeCargo,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_STICK_BOTTOM_LEFT_BUTTON,
                    Shift.None,
                    ButtonType.Simple)); 
            put(
                Operation.GrabberOuttakeCargo,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_STICK_TOP_LEFT_BUTTON,
                    Shift.None,
                    ButtonType.Simple));
            put(
                Operation.GrabberKickPanel,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_STICK_BOTTOM_RIGHT_BUTTON,
                    Shift.None,
                    ButtonType.Simple));
            put(
                Operation.GrabberPointFinger,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_STICK_TOP_RIGHT_BUTTON,
                    Shift.None,
                    ButtonType.Simple));
            put(
                Operation.GrabberWristStartPosition,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    180,
                    Shift.None,
                    ButtonType.Click));
            put(
                Operation.GrabberWristHatchPosition,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    90,
                    Shift.None,
                    ButtonType.Click));
            put(
                Operation.GrabberWristCargoPosition,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    270,
                    Shift.None,
                    ButtonType.Click));
            put(
                Operation.GrabberWristFloorPosition,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    0,
                    Shift.None,
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
                Operation.ClimberArmsLowClimbPosition,
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
                Operation.ClimberArmsForceForward,
                new DigitalOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_1,
                    Shift.ButtonPadDebug,
                    ButtonType.Simple));
            put(
                Operation.ClimberArmsForceBackward,
                new DigitalOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_2,
                    Shift.ButtonPadDebug,
                    ButtonType.Simple));
            put(
                Operation.ClimberCamForceForward,
                new DigitalOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_3,
                    Shift.ButtonPadDebug,
                    ButtonType.Simple));
            put(
                Operation.ClimberCamForceBackward,
                new DigitalOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_4,
                    Shift.ButtonPadDebug,
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
                    UserInputDeviceButton.JOYSTICK_STICK_THUMB_BUTTON,
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
                    90,
                    Shift.Debug,
                    ButtonType.Toggle,
                    () -> new NavxTurnTask(true, 180, TuningConstants.NAVX_FAST_TURN_TIMEOUT, true, true),
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
                    270,
                    Shift.Debug,
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
                MacroOperation.ClimbHab2,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_10,
                    Shift.ButtonPadDebug,
                    ButtonType.Toggle,
                    () -> new WaitTask(0.0),
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
                        Operation.ClimberArmsRetractedPosition,
                        Operation.ClimberArmsLowClimbPosition,
                        Operation.ClimberArmsHighClimbPosition,
                        Operation.ClimberCamStoredPosition,
                        Operation.ClimberCamLowClimbPosition,
                        Operation.ClimberCamHighClimbPosition,
                        Operation.ClimberArmsMoveBackward,
                        Operation.ClimberArmsMoveForward,
                        Operation.ClimberCamMoveBackward,
                        Operation.ClimberCamMoveForward,
                        Operation.ClimberArmsForceBackward,
                        Operation.ClimberArmsForceForward,
                        Operation.ClimberCamForceBackward,
                        Operation.ClimberCamForceForward,
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
                MacroOperation.ClimbHab3,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_15,
                    Shift.ButtonPadDebug,
                    ButtonType.Toggle,
                    () -> new WaitTask(0.0),
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
                        Operation.ClimberArmsRetractedPosition,
                        Operation.ClimberArmsLowClimbPosition,
                        Operation.ClimberArmsHighClimbPosition,
                        Operation.ClimberCamStoredPosition,
                        Operation.ClimberCamLowClimbPosition,
                        Operation.ClimberCamHighClimbPosition,
                        Operation.ClimberArmsMoveBackward,
                        Operation.ClimberArmsMoveForward,
                        Operation.ClimberCamMoveBackward,
                        Operation.ClimberCamMoveForward,
                        Operation.ClimberArmsForceBackward,
                        Operation.ClimberArmsForceForward,
                        Operation.ClimberCamForceBackward,
                        Operation.ClimberCamForceForward,
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
                    180,
                    Shift.Debug,
                    ButtonType.Toggle,
                    () -> new VisionAdvanceAndCenterTask(Operation.VisionEnableRocket),
                    new Operation[]
                    {
                        Operation.VisionEnableCargoShip,
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                        Operation.DriveTrainTurn,
                        Operation.DriveTrainMoveForward,
                        Operation.VisionEnableRocket
                    }));
            put(
                MacroOperation.VisionCenterAndAdvanceCargoShip,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    0,
                    Shift.Debug,
                    ButtonType.Toggle,
                    () -> new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
                    new Operation[]
                    {
                        Operation.VisionEnableCargoShip,
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                        Operation.DriveTrainTurn,
                        Operation.DriveTrainMoveForward,
                        Operation.VisionEnableRocket
                    }));
            put(
                MacroOperation.VisionCenterCargoShip,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_13,
                    Shift.None,
                    ButtonType.Toggle,
                    () -> new VisionCenteringTask(Operation.VisionEnableCargoShip),
                    new Operation[]
                    {
                        Operation.VisionEnableCargoShip,
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                        Operation.DriveTrainTurn,
                        Operation.DriveTrainMoveForward,
                        Operation.VisionEnableRocket
                    }));

            // Grabber Macro
            put(
                MacroOperation.GrabberKickPanelRepeatedlyTask,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_STICK_BOTTOM_RIGHT_BUTTON,
                    Shift.Debug,
                    ButtonType.Simple,
                    () -> new GrabberKickPanelRepeatedlyTask(),
                    new Operation[]
                    {
                        Operation.GrabberKickPanel,
                    }));


            // Elevator Macros
            put(
                MacroOperation.ElevatorMoveUp,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_14,
                    Shift.None,
                    ButtonType.Simple,
                    () -> new ElevatorMovementTask(Operation.ElevatorMoveUp),
                    new Operation[]
                    {
                        Operation.ElevatorMoveUp,
                        Operation.ElevatorMoveDown,
                        Operation.ElevatorForceUp,
                        Operation.ElevatorForceDown,
                    }));
            put(
                MacroOperation.ElevatorMoveDown,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_15,
                    Shift.None,
                    ButtonType.Simple,
                    () -> new ElevatorMovementTask(Operation.ElevatorMoveDown),
                    new Operation[]
                    {
                        Operation.ElevatorMoveUp,
                        Operation.ElevatorMoveDown,
                        Operation.ElevatorForceUp,
                        Operation.ElevatorForceDown,
                    }));
            put(
                MacroOperation.ElevatorForceUp,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_15,
                    Shift.Debug,
                    ButtonType.Simple,
                    () -> new ElevatorMovementTask(Operation.ElevatorForceUp),
                    new Operation[]
                    {
                        Operation.ElevatorMoveUp,
                        Operation.ElevatorMoveDown,
                        Operation.ElevatorForceUp,
                        Operation.ElevatorForceDown,                               
                    }));
            put(
                MacroOperation.ElevatorForceDown,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_16,
                    Shift.Debug,
                    ButtonType.Simple,
                    () -> new ElevatorMovementTask(Operation.ElevatorForceDown),
                    new Operation[]
                    {
                        Operation.ElevatorMoveUp,
                        Operation.ElevatorMoveDown,
                        Operation.ElevatorForceUp,
                        Operation.ElevatorForceDown,                               
                    }));
            put(
                MacroOperation.ElevatorBottomPosition,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_5,
                    Shift.None,
                    ButtonType.Toggle,
                    () -> new ElevatorPositionTask(0.2, Operation.ElevatorBottomPosition),
                    new Operation[]
                    {
                        Operation.ElevatorBottomPosition,
                        Operation.ElevatorHatch2Position,
                        Operation.ElevatorHatch3Position,
                        Operation.ElevatorCargo1Position,
                        Operation.ElevatorCargo2Position,
                        Operation.ElevatorCargo3Position,
                        Operation.ElevatorCargoLoadPosition,                    
                    }));
            put(
                MacroOperation.ElevatorHatch2Position,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_4,
                    Shift.None,
                    ButtonType.Toggle,
                    () -> new ElevatorPositionTask(0.2, Operation.ElevatorHatch2Position),
                    new Operation[]
                    {
                        Operation.ElevatorBottomPosition,
                        Operation.ElevatorHatch2Position,
                        Operation.ElevatorHatch3Position,
                        Operation.ElevatorCargo1Position,
                        Operation.ElevatorCargo2Position,
                        Operation.ElevatorCargo3Position,
                        Operation.ElevatorCargoLoadPosition,                    
                    }));
            put(
                MacroOperation.ElevatorHatch3Position,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_3,
                    Shift.None,
                    ButtonType.Toggle,
                    () -> new ElevatorPositionTask(0.2, Operation.ElevatorHatch3Position),
                    new Operation[]
                    {
                        Operation.ElevatorBottomPosition,
                        Operation.ElevatorHatch2Position,
                        Operation.ElevatorHatch3Position,
                        Operation.ElevatorCargo1Position,
                        Operation.ElevatorCargo2Position,
                        Operation.ElevatorCargo3Position,
                        Operation.ElevatorCargoLoadPosition,                    
                    }));
            put(
                MacroOperation.ElevatorCargo1Position,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_10,
                    Shift.None,
                    ButtonType.Toggle,
                    () -> new ElevatorPositionTask(0.2, Operation.ElevatorCargo1Position),
                    new Operation[]
                    {
                        Operation.ElevatorBottomPosition,
                        Operation.ElevatorHatch2Position,
                        Operation.ElevatorHatch3Position,
                        Operation.ElevatorCargo1Position,
                        Operation.ElevatorCargo2Position,
                        Operation.ElevatorCargo3Position,
                        Operation.ElevatorCargoLoadPosition,                    
                    }));
            put(
                MacroOperation.ElevatorCargo2Position,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_9,
                    Shift.None,
                    ButtonType.Toggle,
                    () -> new ElevatorPositionTask(0.2, Operation.ElevatorCargo2Position),
                    new Operation[]
                    {
                        Operation.ElevatorBottomPosition,
                        Operation.ElevatorHatch2Position,
                        Operation.ElevatorHatch3Position,
                        Operation.ElevatorCargo1Position,
                        Operation.ElevatorCargo2Position,
                        Operation.ElevatorCargo3Position,
                        Operation.ElevatorCargoLoadPosition,                    
                    }));
            put(
                MacroOperation.ElevatorCargo3Position,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_8,
                    Shift.None,
                    ButtonType.Toggle,
                    () -> new ElevatorPositionTask(0.2, Operation.ElevatorCargo3Position),
                    new Operation[]
                    {
                        Operation.ElevatorBottomPosition,
                        Operation.ElevatorHatch2Position,
                        Operation.ElevatorHatch3Position,
                        Operation.ElevatorCargo1Position,
                        Operation.ElevatorCargo2Position,
                        Operation.ElevatorCargo3Position,
                        Operation.ElevatorCargoLoadPosition,                    
                    }));
            put(
                MacroOperation.ElevatorCargoLoadPosition,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_7,
                    Shift.None,
                    ButtonType.Toggle,
                    () -> new ElevatorPositionTask(0.2, Operation.ElevatorCargoLoadPosition),
                    new Operation[]
                    {
                        Operation.ElevatorBottomPosition,
                        Operation.ElevatorHatch2Position,
                        Operation.ElevatorHatch3Position,
                        Operation.ElevatorCargo1Position,
                        Operation.ElevatorCargo2Position,
                        Operation.ElevatorCargo3Position,
                        Operation.ElevatorCargoLoadPosition,                    
                    }));
            put(
                MacroOperation.ElevatorCargo1PositionShifted,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_5,
                    Shift.Debug,
                    ButtonType.Toggle,
                    () -> new ElevatorPositionTask(0.2, Operation.ElevatorCargo1Position),
                    new Operation[]
                    {
                        Operation.ElevatorBottomPosition,
                        Operation.ElevatorHatch2Position,
                        Operation.ElevatorHatch3Position,
                        Operation.ElevatorCargo1Position,
                        Operation.ElevatorCargo2Position,
                        Operation.ElevatorCargo3Position,
                        Operation.ElevatorCargoLoadPosition,                    
                    }));
            put(
                MacroOperation.ElevatorCargo2PositionShifted,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_4,
                    Shift.Debug,
                    ButtonType.Toggle,
                    () -> new ElevatorPositionTask(0.2, Operation.ElevatorCargo2Position),
                    new Operation[]
                    {
                        Operation.ElevatorBottomPosition,
                        Operation.ElevatorHatch2Position,
                        Operation.ElevatorHatch3Position,
                        Operation.ElevatorCargo1Position,
                        Operation.ElevatorCargo2Position,
                        Operation.ElevatorCargo3Position,
                        Operation.ElevatorCargoLoadPosition,                    
                    }));
            put(
                MacroOperation.ElevatorCargo3PositionShifted,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_3,
                    Shift.Debug,
                    ButtonType.Toggle,
                    () -> new ElevatorPositionTask(0.2, Operation.ElevatorCargo3Position),
                    new Operation[]
                    {
                        Operation.ElevatorBottomPosition,
                        Operation.ElevatorHatch2Position,
                        Operation.ElevatorHatch3Position,
                        Operation.ElevatorCargo1Position,
                        Operation.ElevatorCargo2Position,
                        Operation.ElevatorCargo3Position,
                        Operation.ElevatorCargoLoadPosition,                    
                    }));
            put(
                MacroOperation.ElevatorCargoLoadPositionShifted,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_2,
                    Shift.Debug,
                    ButtonType.Toggle,
                    () -> new ElevatorPositionTask(0.2, Operation.ElevatorCargoLoadPosition),
                    new Operation[]
                    {
                        Operation.ElevatorBottomPosition,
                        Operation.ElevatorHatch2Position,
                        Operation.ElevatorHatch3Position,
                        Operation.ElevatorCargo1Position,
                        Operation.ElevatorCargo2Position,
                        Operation.ElevatorCargo3Position,
                        Operation.ElevatorCargoLoadPosition,                    
                    }));

            // Climber Macros
            put(
                MacroOperation.ClimberArmsRetractedPosition,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_12,
                    Shift.ButtonPadDebug,
                    ButtonType.Toggle,
                    () -> new ClimberArmsPositionTask(0.2, Operation.ClimberArmsRetractedPosition),
                    new Operation[]
                    {
                        Operation.ClimberArmsRetractedPosition,
                        Operation.ClimberArmsLowClimbPosition,
                        Operation.ClimberArmsHighClimbPosition  
                    }));

            put(
                MacroOperation.ClimberArmsLowClimbPosition,
                new MacroOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_2,
                    Shift.ButtonPadDebug,
                    ButtonType.Toggle,
                    () -> new ClimberArmsPositionTask(0.2, Operation.ClimberArmsLowClimbPosition),
                    new Operation[]
                    {
                        Operation.ClimberArmsRetractedPosition,
                        Operation.ClimberArmsLowClimbPosition,
                        Operation.ClimberArmsHighClimbPosition                                  
                    }));

            put(
                MacroOperation.ClimberArmsHighClimbPosition,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_11,
                    Shift.ButtonPadDebug,
                    ButtonType.Toggle,
                    () -> new ClimberArmsPositionTask(0.2, Operation.ClimberArmsHighClimbPosition),
                    new Operation[]
                    {
                        Operation.ClimberArmsRetractedPosition,
                        Operation.ClimberArmsLowClimbPosition,
                        Operation.ClimberArmsHighClimbPosition                                  
                    }));

            put(
                MacroOperation.ClimberCamStoredPosition,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_14,
                    Shift.ButtonPadDebug,
                    ButtonType.Toggle,
                    () -> new ClimberCamPositionTask(0.2, Operation.ClimberCamStoredPosition),
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
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_2,
                    Shift.ButtonPadDebug,
                    ButtonType.Toggle,
                    () -> new ClimberCamPositionTask(0.2, Operation.ClimberCamLowClimbPosition),
                    new Operation[]
                    {
                        Operation.ClimberCamStoredPosition,
                        Operation.ClimberCamLowClimbPosition,
                        Operation.ClimberCamHighClimbPosition,                                    
                    }));

            put(
                MacroOperation.ClimberCamHighClimbPosition,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_13,
                    Shift.ButtonPadDebug,
                    ButtonType.Toggle,
                    () -> new ClimberCamPositionTask(0.2, Operation.ClimberCamHighClimbPosition),
                    new Operation[]
                    {
                        Operation.ClimberCamStoredPosition,
                        Operation.ClimberCamLowClimbPosition,
                        Operation.ClimberCamHighClimbPosition,                                   
                    }));

            put(
                MacroOperation.ClimberArmsMoveForward,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_6,
                    Shift.ButtonPadDebug,
                    ButtonType.Simple,
                    () -> new ClimberArmsMovementTask(Operation.ClimberArmsMoveForward),
                    new Operation[]
                    {
                        Operation.ClimberArmsMoveForward,
                        Operation.ClimberArmsMoveBackward,
                        Operation.ClimberArmsForceForward,
                        Operation.ClimberArmsForceBackward                
                    }));
            put(
                MacroOperation.ClimberArmsMoveBackward,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_7,
                    Shift.ButtonPadDebug,
                    ButtonType.Simple,
                    () -> new ClimberArmsMovementTask(Operation.ClimberArmsMoveBackward),
                    new Operation[]
                    {
                        Operation.ClimberArmsMoveForward,
                        Operation.ClimberArmsMoveBackward,
                        Operation.ClimberArmsForceForward,
                        Operation.ClimberArmsForceBackward                            
                    }));
            put(
                MacroOperation.ClimberArmsForceForward,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_1,
                    Shift.ButtonPadDebug,
                    ButtonType.Simple,
                    () -> new ClimberArmsMovementTask(Operation.ClimberArmsForceForward),
                    new Operation[]
                    {
                        Operation.ClimberArmsMoveForward,
                        Operation.ClimberArmsMoveBackward,
                        Operation.ClimberArmsForceForward,
                        Operation.ClimberArmsForceBackward                            
                    }));
            put(
                MacroOperation.ClimberArmsForceBackward,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_2,
                    Shift.ButtonPadDebug,
                    ButtonType.Simple,
                    () -> new ClimberArmsMovementTask(Operation.ClimberArmsForceBackward),
                    new Operation[]
                    {
                        Operation.ClimberArmsMoveForward,
                        Operation.ClimberArmsMoveBackward,
                        Operation.ClimberArmsForceForward,
                        Operation.ClimberArmsForceBackward                            
                    }));
            put(
                MacroOperation.ClimberCamMoveForward,
                new MacroOperationDescription(
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_8,
                    Shift.ButtonPadDebug,
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
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_9,
                    Shift.ButtonPadDebug,
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
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_3,
                    Shift.ButtonPadDebug,
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
                    UserInputDevice.CoDriver,
                    UserInputDeviceButton.BUTTON_PAD_BUTTON_4,
                    Shift.ButtonPadDebug,
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
