package frc.robot.driver;

public enum MacroOperation
{
    // DriveTrain operations:
    AutonomousRoutine,
    PIDBrake,
    TurnInPlaceLeft,
    TurnInPlaceRight,
    FollowSomePath,

    // Vision operations:
    VisionCenterAndAdvanceRocket,
    VisionCenterAndAdvanceCargoShip,
    VisionCenterAndAdvanceCargoShip2,
    VisionFastCenterAndAdvanceCargoShip,
    DriveForwardTurnRight,

    // Grabber operation:
    GrabberKickPanelRepeatedlyTask,
    GrabberKickPanel,
    GrabberPointBeak,
    GrabberIntakeCargo,
    GrabberOuttakeCargo,

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
    ClimberArmsForceZero,
    ClimberCamForceForward,
    ClimberCamForceBackward,
    ClimbPiecewiseA,
    ClimbPiecewiseB,
    ClimbPiecewiseC,
    ClimbPiecewiseD,
    ClimbHabAndReset,
    ClimbHabLeaveTailDown,

    // Elevator operations:
    ElevatorBottomPosition,
    ElevatorHatch2Position,
    ElevatorHatch3Position,
    ElevatorCargo1Position,
    ElevatorCargo2Position,
    ElevatorCargo3Position,
    ElevatorCargoLoadPosition,
    ElevatorCargo1PositionShifted,
    ElevatorCargo2PositionShifted,
    ElevatorCargo3PositionShifted,
    ElevatorCargoLoadPositionShifted,
    ElevatorMoveUp,
    ElevatorMoveDown,
    ElevatorForceUp,
    ElevatorForceDown,

    // Elevator and Arm Macros
    ConfigCargoLoading,
    ConfigCargoGround,
    ConfigHatchLoading,
}
