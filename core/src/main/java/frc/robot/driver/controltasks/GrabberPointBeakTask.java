package frc.robot.driver.controltasks;

import frc.robot.driver.Operation;
import frc.robot.TuningConstants;

public class GrabberPointBeakTask extends TimedTask
{
    private final boolean neverEnds;

    public GrabberPointBeakTask()
    {
        this(false);
    }

    public GrabberPointBeakTask(boolean neverEnds)
    {
        this(TuningConstants.GRABBER_POINT_BEAK_DURATION, neverEnds);
    }

    public GrabberPointBeakTask(double duration)
    {
        this(duration, false);
    }

    private GrabberPointBeakTask(double duration, boolean neverEnds)
    {
        super(duration);

        this.neverEnds = neverEnds;
    }

    @Override
    public void begin()
    {
        super.begin();
        this.setDigitalOperationState(Operation.GrabberPointBeak, true);
    }

    @Override
    public void update() 
    {
        this.setDigitalOperationState(Operation.GrabberPointBeak, true);
    }

    @Override
    public void end()
    {
        this.setDigitalOperationState(Operation.GrabberPointBeak,false);
    }

    @Override
    public boolean hasCompleted()
    {
        if (this.neverEnds)
        {
            return false;
        }

        return super.hasCompleted();
    }
}
