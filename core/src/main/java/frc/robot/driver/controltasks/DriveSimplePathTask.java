package frc.robot.driver.controltasks;

import frc.robot.driver.Operation;
import frc.robot.mechanisms.DriveTrainMechanism;

public class DriveSimplePathTask extends TimedTask
{
    private double velocityL;
    private double velocityR;

    private DriveTrainMechanism driveTrain;

    public DriveSimplePathTask(double duration, double velocityL, double velocityR)
    {
        super(duration);

        this.velocityL = velocityL;
        this.velocityR = velocityR;
    }

    @Override
    public void begin()
    {
        super.begin();
        this.driveTrain = this.getInjector().getInstance(DriveTrainMechanism.class);
    }

    @Override
    public void update()
    {
        this.setDigitalOperationState(Operation.DriveTrainUseSimplePathMode, true);
        this.setAnalogOperationState(Operation.DriveTrainLeftVelocity, this.velocityL);
        this.setAnalogOperationState(Operation.DriveTrainRightVelocity, this.velocityR);
    }



}
