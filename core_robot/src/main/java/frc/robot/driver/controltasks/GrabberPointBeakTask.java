package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;
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
        this.setDigitalOperationState(DigitalOperation.GrabberPointBeak, true);
    }

    @Override
    public void update() 
    {
        this.setDigitalOperationState(DigitalOperation.GrabberPointBeak, true);
    }

    @Override
    public void end()
    {
        this.setDigitalOperationState(DigitalOperation.GrabberPointBeak,false);
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
