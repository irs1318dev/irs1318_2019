package frc.robot.driver.controltasks;

import frc.robot.common.robotprovider.ITimer;
import frc.robot.driver.Operation;

public class GrabberKickPanelRepeatedlyTask extends ControlTaskBase
{
    private ITimer timer;

    private static final double cycleTime = 0.4;
    private static final double kickTime = 0.25;

    public GrabberKickPanelRepeatedlyTask()
    {
    }

    @Override
    public void begin()
    {
        this.timer = this.getInjector().getInstance(ITimer.class);
        this.timer.start();
    }

    @Override
    public void update()
    {
        if((timer.get() % GrabberKickPanelRepeatedlyTask.cycleTime) > (GrabberKickPanelRepeatedlyTask.kickTime))
        {
            this.setDigitalOperationState(Operation.GrabberKickPanel, true);
        }
        else{
            this.setDigitalOperationState(Operation.GrabberKickPanel, false);
        }
    }

    @Override
    public boolean hasCompleted()
    {
        return false;
    }

    @Override
    public void end()
    {
        this.setDigitalOperationState(Operation.GrabberKickPanel, false);
    }
}
