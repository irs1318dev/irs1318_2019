package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;
import frc.robot.TuningConstants;

public class ClimberCamMovementTask extends CompositeOperationTask
{
    private static final DigitalOperation[] possibleOperations =
        {
            DigitalOperation.ClimberCamMoveForward,
            DigitalOperation.ClimberCamMoveBackward,
            DigitalOperation.ClimberCamForceForward,
            DigitalOperation.ClimberCamForceBackward                
        };
 
    public ClimberCamMovementTask(DigitalOperation toPerform)
    {
        super(TuningConstants.CLIMBER_CAM_MOVEMENT_TIME_THRESHOLD, toPerform, ClimberCamMovementTask.possibleOperations);
    }   

    @Override
    public boolean hasCompleted()
    {
        return false;
    }
}
