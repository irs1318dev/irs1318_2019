package frc.robot.driver.controltasks;

import frc.robot.driver.Operation;
import frc.robot.TuningConstants;

public class ClimberCamMovementTask extends CompositeOperationTask
{
    private static final Operation[] possibleOperations =
        {
            Operation.ClimberCamMoveForward,
            Operation.ClimberCamMoveBackward,
            Operation.ClimberCamForceForward,
            Operation.ClimberCamForceBackward                
        };
 
    public ClimberCamMovementTask(Operation toPerform)
    {
        super(TuningConstants.CLIMBER_CAM_MOVEMENT_TIME_THRESHOLD, toPerform, ClimberCamMovementTask.possibleOperations);
    }   

    @Override
    public boolean hasCompleted()
    {
        return false;
    }
}
