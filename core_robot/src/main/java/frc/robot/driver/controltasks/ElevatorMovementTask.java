package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;
import frc.robot.TuningConstants;

public class ElevatorMovementTask extends CompositeOperationTask
{
    private static final DigitalOperation[] possibleOperations =
        {
            DigitalOperation.ElevatorMoveUp,
            DigitalOperation.ElevatorMoveDown,
            DigitalOperation.ElevatorForceUp,
            DigitalOperation.ElevatorForceDown
        };

    public ElevatorMovementTask(DigitalOperation toPerform)
    {
        super(TuningConstants.ELEVATOR_CLIMBING_MOVEMENT_TIME_THRESHOLD, toPerform, ElevatorMovementTask.possibleOperations);
    }

    @Override
    public boolean hasCompleted()
    {
        return false;
    }
}
