package frc.robot.driver.controltasks;

import frc.robot.driver.Operation;
import frc.robot.mechanisms.*;

public class DriveUntilSensorTask extends TimedTask
{
    private final double velocity;

    private ClimberMechanism climber;

    public DriveUntilSensorTask(double velocity, double duration)
    {
        super(duration);

        this.velocity = velocity;
    }

    @Override
    public void begin()
    {
        super.begin();

        this.climber = this.getInjector().getInstance(ClimberMechanism.class);
    }

    @Override
    public void update()
    {
        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);
        this.setAnalogOperationState(Operation.DriveTrainTurn, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainMoveForward, this.velocity);
    }

    @Override
    public void end()
    {
        super.end();

        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);
        this.setAnalogOperationState(Operation.DriveTrainTurn, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainMoveForward, 0.0);
    }

    @Override
    public boolean hasCompleted()
    {
        if (super.hasCompleted())
        {
            return true;
        }

        return this.climber.isClimbed();
    }
}
