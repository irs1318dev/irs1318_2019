package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.DigitalOperation;

public class GrabberSetWristPositionTask extends CompositeOperationTask 
{
    private static final DigitalOperation[] possibleOperations =
        {
            DigitalOperation.GrabberWristStartPosition,
            DigitalOperation.GrabberWristHatchPosition,
            DigitalOperation.GrabberWristCargoPosition,
            DigitalOperation.GrabberWristFloorPosition,
        };


    public GrabberSetWristPositionTask(double duration, DigitalOperation toPerfrom)
    {
        super(duration, toPerfrom, GrabberSetWristPositionTask.possibleOperations);
    }

    public GrabberSetWristPositionTask(DigitalOperation toPerform) 
    {
        this(TuningConstants.GRABBER_SET_WRIST_TIME_DURATION, toPerform);
    }
}
