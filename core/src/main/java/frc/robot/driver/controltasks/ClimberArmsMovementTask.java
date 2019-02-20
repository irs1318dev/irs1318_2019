package frc.robot.driver.controltasks;

import frc.robot.driver.Operation;
import frc.robot.TuningConstants;

public class ClimberArmsMovementTask extends CompositeOperationTask
{
    private static final Operation[] possibleOperations =
        {
            Operation.ClimberArmsMoveForward,
            Operation.ClimberArmsMoveBackward,
            Operation.ClimberArmsForceForward,
            Operation.ClimberArmsForceBackward                
        };

    public ClimberArmsMovementTask(Operation toPerform)
    {
        super(TuningConstants.CLIMBER_ARMS_MOVEMENT_TIME_THRESHOLD, toPerform, ClimberArmsMovementTask.possibleOperations);
    }

    @Override
    public boolean hasCompleted()
    {
        return false;
    }
}
