package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;
import frc.robot.TuningConstants;

public class GrabberKickPanelTask extends TimedTask
{
    private final boolean neverEnds;

    public GrabberKickPanelTask()
    {
        this(false);
    }

    public GrabberKickPanelTask(double duration)
    {
        this(duration, false);
    }

    public GrabberKickPanelTask(boolean neverEnds)
    {
        this(TuningConstants.GRABBER_KICK_PANEL_DURATION, neverEnds);
    }

    private GrabberKickPanelTask(double duration, boolean neverEnds)
    {
        super(duration);

        this.neverEnds = neverEnds;
    }

    @Override
    public void begin()
    {
        super.begin();
        this.setDigitalOperationState(DigitalOperation.GrabberKickPanel, true);
    }

    @Override
    public void update() 
    {
        this.setDigitalOperationState(DigitalOperation.GrabberKickPanel, true);
    }

    @Override
    public void end()
    {
        this.setDigitalOperationState(DigitalOperation.GrabberKickPanel,false);
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
