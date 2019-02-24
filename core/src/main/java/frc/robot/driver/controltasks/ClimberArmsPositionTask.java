package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.Operation;
import frc.robot.mechanisms.ClimberMechanism;

public class ClimberArmsPositionTask extends CompositeOperationTask
{
    private static final Operation[] possibleOperations =
        {
            Operation.ClimberArmsRetractedPosition,
            Operation.ClimberArmsLowClimbPosition,
            Operation.ClimberArmsHighClimbPosition  
        };

    private final boolean completeWithTime;

    private ClimberMechanism climber;
    private boolean firstLoop;

    public ClimberArmsPositionTask(double duration, Operation toPerform)
    {
        super(duration, toPerform, ClimberArmsPositionTask.possibleOperations);
        this.completeWithTime = true;
    }

    public ClimberArmsPositionTask(Operation toPerform)
    {
        super(TuningConstants.CLIMBER_ARMS_MOVEMENT_TIME_THRESHOLD, toPerform, ClimberArmsPositionTask.possibleOperations);
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

        double positionError = Math.abs(this.climber.getArmsDesiredPosition() - this.climber.getArmsPosition());
        if (!this.firstLoop && positionError < TuningConstants.CLIMBER_ARMS_POSITION_ERROR_THRESHOLD)
        {
            return true;
        }

        return false;
    }
}
