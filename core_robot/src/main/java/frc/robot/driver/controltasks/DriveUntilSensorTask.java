package frc.robot.driver.controltasks;

import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
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
        this.setDigitalOperationState(DigitalOperation.DriveTrainUsePositionalMode, false);
        this.setAnalogOperationState(AnalogOperation.DriveTrainTurn, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, this.velocity);
    }

    @Override
    public void end()
    {
        super.end();

        this.setDigitalOperationState(DigitalOperation.DriveTrainUsePositionalMode, false);
        this.setAnalogOperationState(AnalogOperation.DriveTrainTurn, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.0);
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
