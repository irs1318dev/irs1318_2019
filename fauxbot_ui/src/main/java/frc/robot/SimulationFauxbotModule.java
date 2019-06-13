package frc.robot;

import frc.robot.simulation.*;

public class SimulationFauxbotModule extends FauxbotModule
{
    @Override
    protected void configure()
    {
        super.configure();

        this.bind(IRealWorldSimulator.class).to(RobotSimulator.class);
    }
}
