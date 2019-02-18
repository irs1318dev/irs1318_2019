package frc.robot.driver;

public enum Operation
{
    // Vision operations:
    VisionEnableCargoShip,
    VisionEnableRocket,
    VisionEnableOffboardStream,
    VisionEnableOffboardProcessing,

    // DriveTrain operations:
    DriveTrainEnablePID,
    DriveTrainDisablePID,
    DriveTrainMoveForward,
    DriveTrainTurn,
    DriveTrainSimpleMode,
    DriveTrainUseBrakeMode,
    DriveTrainUsePositionalMode,
    DriveTrainUsePathMode,
    DriveTrainLeftPosition,
    DriveTrainRightPosition,
    DriveTrainLeftVelocity,
    DriveTrainRightVelocity,
    DriveTrainHeadingCorrection,
    DriveTrainSwapFrontOrientation,

    // Elevator operations:
    ElevatorForceUp,
    ElevatorForceDown,
    ElevatorMoveUp,
    ElevatorMoveDown,
    ElevatorBottomPosition,
    ElevatorHatch2Position,
    ElevatorHatch3Position,
    ElevatorCargo1Position,
    ElevatorCargo2Position,
    ElevatorCargo3Position,
    ElevatorCargoLoadPosition,

    // Grabber operations:
    GrabberKickPanel,
    GrabberIntakeCargo,
    GrabberOuttakeCargo,
    GrabberWristStartPosition,
    GrabberWristHatchPosition,
    GrabberWristCargoPosition,
    GrabberWristFloorPosition,

    // Climber operations:
    ClimberArmsRetractedPosition,
    ClimberArmsLowClimbPosition,
    ClimberArmsHighClimbPosition,
    ClimberCamStoredPosition,
    ClimberCamLowClimbPosition,
    ClimberCamHighClimbPosition,
    ClimberArmsMoveForward,
    ClimberArmsMoveBackward,
    ClimberCamMoveForward,
    ClimberCamMoveBackward,
    ClimberArmsForceForward,
    ClimberArmsForceBackward,
    ClimberCamForceForward,
    ClimberCamForceBackward,
}
