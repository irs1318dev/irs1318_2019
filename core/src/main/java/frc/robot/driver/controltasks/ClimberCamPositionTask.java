package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.Operation;
import frc.robot.mechanisms.ClimberMechanism;

public class ClimberCamPositionTask extends CompositeOperationTask
{
    private static final Operation[] possibleOperations =
    {
        Operation.ClimberCamStoredPosition,
        Operation.ClimberCamLowClimbPosition,
        Operation.ClimberCamHighClimbPosition,
    };

    private ClimberMechanism climber;
    private boolean completeWithTime;

    public ClimberCamPositionTask(double duration, Operation toPerform)
    {
        super(duration, toPerform, ClimberCamPositionTask.possibleOperations);
        this.completeWithTime = true;
    }

    public ClimberCamPositionTask(Operation toPerform)
    {
        super(TuningConstants.CLIMBER_CAM_MOVEMENT_TIME_THRESHOLD, toPerform, ClimberCamPositionTask.possibleOperations);
        this.completeWithTime = false;
    }

    @Override
    public void begin()
    {
        super.begin();
        this.getInjector().getInstance(ClimberMechanism.class);
    }

    @Override
    public boolean hasCompleted()
    {
        if (this.completeWithTime)
        {
            return super.hasCompleted();
        }

        if(super.hasCompleted())
        {
            return true;
        }

        if (this.climber.getCamError() < TuningConstants.CLIMBER_CAM_POSITION_ERROR_THRESHOLD)
        {
            return true;
        }

        return false;

    }
}