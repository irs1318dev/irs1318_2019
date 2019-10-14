package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;
import frc.robot.TuningConstants;

public class ClimberArmsMovementTask extends CompositeOperationTask
{
    private static final DigitalOperation[] possibleOperations =
        {
            DigitalOperation.ClimberArmsMoveForward,
            DigitalOperation.ClimberArmsMoveBackward,
            DigitalOperation.ClimberArmsForceForward,
            DigitalOperation.ClimberArmsForceBackward,
            DigitalOperation.ClimberArmsForceZero
        };

    public ClimberArmsMovementTask(DigitalOperation toPerform)
    {
        super(TuningConstants.CLIMBER_ARMS_MOVEMENT_TIME_THRESHOLD, toPerform, ClimberArmsMovementTask.possibleOperations);
    }

    @Override
    public boolean hasCompleted()
    {
        return false;
    }
}
