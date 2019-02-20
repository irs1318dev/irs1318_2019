package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.Operation;

public class GrabberSetWristPositionTask extends CompositeOperationTask 
{
    private static final Operation[] possibleOperations =
        {
            Operation.GrabberWristStartPosition,
            Operation.GrabberWristHatchPosition,
            Operation.GrabberWristCargoPosition,
            Operation.GrabberWristFloorPosition,
        };


    public GrabberSetWristPositionTask(double duration, Operation toPerfrom)
    {
        super(duration, toPerfrom, GrabberSetWristPositionTask.possibleOperations);
    }

    public GrabberSetWristPositionTask(Operation toPerform) 
    {
        this(TuningConstants.GRABBER_SET_WRIST_TIME_DURATION, toPerform);
    }
}
