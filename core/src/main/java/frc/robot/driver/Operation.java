package frc.robot.driver;

public enum Operation
{
    // Vision operations:
    VisionDisable,
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
    DriveTrainUseSimplePathMode,
    DriveTrainUsePathMode,
    DriveTrainMaxVelocityRatio,
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
    ElevatorCamReturnPosition,

    // Grabber operations:
    GrabberKickPanel,
    GrabberPointBeak,
    GrabberIntakeCargo,
    GrabberOuttakeCargo,
    GrabberWristStartPosition,
    GrabberWristHatchPosition,
    GrabberWristCargoPosition,
    GrabberWristFloorPosition,

    // Climber operations:
    ClimberArmsRetractedPosition,
    ClimberArmsPrepClimbPosition,
    ClimberArmsHighClimbPosition,
    ClimberCamStoredPosition,
    ClimberCamLowClimbPosition,
    ClimberCamHighClimbPosition,
    ClimberCamOutOfWayPosition,
    ClimberArmsMoveForward,
    ClimberArmsMoveBackward,
    ClimberCamMoveForward,
    ClimberCamMoveBackward,
    ClimberArmsForceForward,
    ClimberArmsForceBackward,
    ClimberArmsForceZero,
    ClimberCamForceForward,
    ClimberCamForceBackward,
}
