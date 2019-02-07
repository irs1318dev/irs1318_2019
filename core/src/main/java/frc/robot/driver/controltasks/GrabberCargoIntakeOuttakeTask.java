package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.Operation;
import frc.robot.mechanisms.GrabberMechanism;

public class GrabberCargoIntakeOuttakeTask extends CompositeOperationTask 
{
    private static final Operation[] possibleOperations =
        {
            Operation.GrabberIntakeCargo,
            Operation.GrabberOuttakeCargo
        };

    private GrabberMechanism grabber;
    // True when intaking
    private boolean completeWithThroughBeam;

    public GrabberCargoIntakeOuttakeTask (Operation toPerform, boolean completeWithThroughBeam)
    {
        super(TuningConstants.GRABBER_CARGO_INTAKE_OUTTAKE_OVERRIDE_TIME, toPerform, GrabberCargoIntakeOuttakeTask.possibleOperations);
        this.completeWithThroughBeam = completeWithThroughBeam;
    }

    public GrabberCargoIntakeOuttakeTask (double duration, Operation toPerform, boolean completeWithThroughBeam)
    {
        super(duration, toPerform, GrabberCargoIntakeOuttakeTask.possibleOperations);
        this.completeWithThroughBeam = completeWithThroughBeam;
    }

    @Override
    public void begin()
    {
        super.begin();
        this.grabber = this.getInjector().getInstance(GrabberMechanism.class);
    }

    @Override 
    public boolean hasCompleted()
    {
        if (this.completeWithThroughBeam)
        {
            return this.grabber.hasCargo();
        }

        return super.hasCompleted();
    } 
}
