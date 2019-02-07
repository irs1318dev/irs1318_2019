package frc.robot.driver.controltasks;

import frc.robot.driver.Operation;

public class GrabberKickPanelTask extends TimedTask
{
    public GrabberKickPanelTask(double duration)
    {
        super(duration);
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
