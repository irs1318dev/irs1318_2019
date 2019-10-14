package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.ClimberMechanism;

public class ClimberCamPositionTask extends CompositeOperationTask
{
    private static final DigitalOperation[] possibleOperations =
    {
        DigitalOperation.ClimberCamStoredPosition,
        DigitalOperation.ClimberCamLowClimbPosition,
        DigitalOperation.ClimberCamHighClimbPosition,
        DigitalOperation.ClimberCamOutOfWayPosition,
    };

    private boolean completeWithTime;

    private ClimberMechanism climber;
    private boolean firstLoop;

    public ClimberCamPositionTask(double duration, DigitalOperation toPerform)
    {
        super(duration, toPerform, ClimberCamPositionTask.possibleOperations);
        this.completeWithTime = true;
    }

    public ClimberCamPositionTask(DigitalOperation toPerform)
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
