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

    private boolean completeWithTime;

    private ClimberMechanism climber;
    private boolean firstLoop;

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

        this.climber = this.getInjector().getInstance(ClimberMechanism.class);
        this.firstLoop = true;
    }

    @Override
    public void update()
    {
        super.update();

        this.firstLoop = false;
    }

    @Override
    public boolean hasCompleted()
    {
        if (this.completeWithTime)
        {
            return super.hasCompleted();
        }

        if (super.hasCompleted())
        {
            return true;
        }

        double positionError = Math.abs(this.climber.getCamDesiredPosition() - this.climber.getCamPosition());
        if (!this.firstLoop && positionError < TuningConstants.CLIMBER_CAM_POSITION_ERROR_THRESHOLD)
        {
            return true;
        }

        return false;
    }
}
