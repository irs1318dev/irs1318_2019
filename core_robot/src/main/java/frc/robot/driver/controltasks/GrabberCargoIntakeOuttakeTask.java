package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.GrabberMechanism;

public class GrabberCargoIntakeOuttakeTask extends CompositeOperationTask 
{
    private static final DigitalOperation[] possibleOperations =
        {
            DigitalOperation.GrabberIntakeCargo,
            DigitalOperation.GrabberOuttakeCargo
        };

    private GrabberMechanism grabber;
    private boolean completeWithSensor;
    private boolean useTime;

    public GrabberCargoIntakeOuttakeTask(DigitalOperation toPerform, boolean completeWithSensor)
    {
        super(TuningConstants.GRABBER_CARGO_INTAKE_OUTTAKE_OVERRIDE_TIME, toPerform, GrabberCargoIntakeOuttakeTask.possibleOperations);
        this.completeWithSensor = completeWithSensor;
        this.useTime = false;
    }

    public GrabberCargoIntakeOuttakeTask(double duration, DigitalOperation toPerform, boolean completeWithSensor)
    {
        super(duration, toPerform, GrabberCargoIntakeOuttakeTask.possibleOperations);
        this.completeWithSensor = completeWithSensor;
        this.useTime = true;
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
        if (this.completeWithSensor)
        {
            return this.grabber.hasCargo();
        }

        if (!this.useTime)
        {
            return false;
        }

        return super.hasCompleted();
    } 
}
