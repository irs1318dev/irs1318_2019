package frc.robot.driver.controltasks;

import frc.robot.driver.Operation;
import frc.robot.mechanisms.GrabberMechanism;

public class GrabberKickPanelTask extends TimedTask
{
    private GrabberMechanism grabber;

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
        
    }

    @Override
    public void end()
    {
        this.setDigitalOperationState(Operation.GrabberKickPanel,false);
    }

    @Override
    public void stop()
    {
        this.setDigitalOperationState(Operation.GrabberKickPanel,false);
    }

}
