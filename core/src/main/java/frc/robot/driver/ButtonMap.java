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
        }
    };

    @SuppressWarnings("serial")
    public static Map<Operation, OperationDescription> OperationSchema = new HashMap<Operation, OperationDescription>()
    {
        {
            // Operations for vision
            put(
                Operation.EnableVision,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    90, // POV right
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
                Operation.DriveTrainHeading,
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
                    UserInputDeviceButton.JOYSTICK_STICK_TOP_LEFT_BUTTON,
                    Shift.None,
                    ButtonType.Toggle)); 
            put(
                Operation.GrabberOuttakeCargo,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_STICK_BOTTOM_LEFT_BUTTON,
                    Shift.None,
                    ButtonType.Toggle));
            put(
                Operation.GrabberKickPanel,
                new DigitalOperationDescription(
                    UserInputDevice.Driver,
                    UserInputDeviceButton.JOYSTICK_STICK_BOTTOM_RIGHT_BUTTON,
                    Shift.None,
                    ButtonType.Click));
            put(
                Operation.GrabberStowKicker,
                new DigitalOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Click));
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
            
            // turn in place macros
            put(
                MacroOperation.TurnInPlaceRight,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    90,
                    Shift.Debug,
                    ButtonType.Toggle,
                    () -> new NavxTurnTask(false, 180, true, true),
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
                    () -> new NavxTurnTask(false, -180, true, true),
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
                        Operation.DriveTrainHeading,
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

            //Climbing macros
                //ClimbHab2
                //ClimbHab3

            // Centering macro
            put(
                MacroOperation.VisionCenter,
                new MacroOperationDescription(
                    UserInputDevice.Driver,
                    0,
                    Shift.Debug,
                    ButtonType.Toggle,
                    () -> new VisionCenteringTask(),
                    new Operation[]
                    {
                        Operation.EnableVision,
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                        Operation.DriveTrainTurn,
                        Operation.DriveTrainMoveForward,
                    }));
            put(
                MacroOperation.VisionCenterAndAdvance,
                new MacroOperationDescription(
                    UserInputDevice.None,
                    UserInputDeviceButton.NONE,
                    ButtonType.Toggle,
                    () -> new VisionAdvanceAndCenterTask(),
                    new Operation[]
                    {
                        Operation.EnableVision,
                        Operation.DriveTrainUsePositionalMode,
                        Operation.DriveTrainLeftPosition,
                        Operation.DriveTrainRightPosition,
                        Operation.DriveTrainTurn,
                        Operation.DriveTrainMoveForward,
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
