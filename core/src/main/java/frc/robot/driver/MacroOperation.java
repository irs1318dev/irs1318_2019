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
    DriveForwardTurnRight,

    // Climb operations:
    ClimbHab2,
    ClimbHab3,

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

}
