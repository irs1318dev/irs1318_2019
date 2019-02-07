package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.Operation;
import frc.robot.mechanisms.GrabberMechanism;

public class GrabberSetWristPositionTask extends CompositeOperationTask 
{
    private static final Operation[] possibleOperations =
        {
            Operation.GrabberWristStartPosition,
            Operation.GrabberWristHatchPosition,
            Operation.GrabberWristCargoPosition,
            Operation.GrabberWristFloorPosition,
        };

    private GrabberMechanism grabber;

    public GrabberSetWristPositionTask(double duration, Operation toPerfrom)
    {
        super(duration, toPerfrom, GrabberSetWristPositionTask.possibleOperations);
    }

    @Override
    public void begin()
    {
        super.begin();
        this.grabber = this.getInjector().getInstance(GrabberMechanism.class);
    }
}
