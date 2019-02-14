package frc.robot.driver.controltasks;

import frc.robot.driver.Operation;
import frc.robot.TuningConstants;

public class GrabberKickPanelTask extends TimedTask
{
    public GrabberKickPanelTask()
    {
        super(TuningConstants.GRABBER_KICK_PANEL_DURATION);
    }

    @Override
    public void begin()
    {
        super.begin();
        this.setDigitalOperationState(Operation.GrabberKickPanel, true);
    }

    @Override
    public void update() 
    {
        this.setDigitalOperationState(Operation.GrabberKickPanel, true);
    }

    @Override
    public void end()
    {
        this.setDigitalOperationState(Operation.GrabberKickPanel,false);
    }
}
