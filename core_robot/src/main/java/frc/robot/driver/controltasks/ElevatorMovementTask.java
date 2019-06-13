package frc.robot.driver.controltasks;

import frc.robot.driver.Operation;
import frc.robot.TuningConstants;

public class ElevatorMovementTask extends CompositeOperationTask
{
    private static final Operation[] possibleOperations =
        {
            Operation.ElevatorMoveUp,
            Operation.ElevatorMoveDown,
            Operation.ElevatorForceUp,
            Operation.ElevatorForceDown
        };

    public ElevatorMovementTask(Operation toPerform)
    {
        super(TuningConstants.ELEVATOR_CLIMBING_MOVEMENT_TIME_THRESHOLD, toPerform, ElevatorMovementTask.possibleOperations);
    }

    @Override
    public boolean hasCompleted()
    {
        return false;
    }
}
