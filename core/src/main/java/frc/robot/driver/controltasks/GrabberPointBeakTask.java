package frc.robot.driver.controltasks;

import frc.robot.driver.Operation;
import frc.robot.TuningConstants;

public class GrabberPointBeakTask extends TimedTask
{
    public GrabberPointBeakTask()
    {
        super(TuningConstants.GRABBER_POINT_BEAK_DURATION);
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
}
