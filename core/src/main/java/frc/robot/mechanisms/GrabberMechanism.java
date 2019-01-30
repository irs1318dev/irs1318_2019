package frc.robot.mechanisms;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.robot.ElectronicsConstants;
import frc.robot.TuningConstants;
import frc.robot.driver.Operation;
import frc.robot.common.IMechanism;
import frc.robot.driver.common.Driver;
import frc.robot.common.robotprovider.*;

@Singleton
public class GrabberMechanism implements IMechanism {

    private static final String logName = "gr";
    private IDashboardLogger logger;

    private Driver driver;

    // actuators
    private final IDoubleSolenoid kicker; // for ejecting hatches
    private final ITalonSRX cargoMotor; // for cargo intake/outtake
    private final IDoubleSolenoid wristInner; // for controlling the piston closest to the elevator
    private final IDoubleSolenoid wristOuter; // for controlling the piston farthest from the elevator

    @Inject
    public GrabberMechanism(IRobotProvider provider, IDashboardLogger logger) {

        this.kicker = provider.getDoubleSolenoid(ElectronicsConstants.GRABBER_KICKER_FORWARD_CHANNEL, ElectronicsConstants.GRABBER_KICKER_REVERSE_CHANNEL);
        this.cargoMotor = provider.getTalonSRX(ElectronicsConstants.GRABBER_CARGO_MOTOR_CAN_ID);

        this.wristInner = provider.getDoubleSolenoid(ElectronicsConstants.GRABBER_WRIST_INNER_FORWARD_CHANNEL, ElectronicsConstants.GRABBER_WRIST_INNER_REVERSE_CHANNEL);
        this.wristOuter = provider.getDoubleSolenoid(ElectronicsConstants.GRABBER_WRIST_OUTER_FORWARD_CHANNEL, ElectronicsConstants.GRABBER_WRIST_OUTER_REVERSE_CHANNEL);

        this.logger = logger;
    }

    @Override
    public void readSensors()
    {
        // no sensors to read
    }

    @Override
    public void update()
    {
        if (this.driver.getDigital(Operation.GrabberKickPanel))
        {
            this.kicker.set(DoubleSolenoidValue.Forward);
            this.logger.logBoolean(GrabberMechanism.logName, "kicker", true);
        }
        else if (this.driver.getDigital(Operation.GrabberStowKicker))
        {
            this.kicker.set(DoubleSolenoidValue.Reverse);
            this.logger.logBoolean(GrabberMechanism.logName, "kicker", false);
        }

        double cargoMotorPower = 0.0;
        if (this.driver.getDigital(Operation.GrabberIntakeCargo))
        {
            cargoMotorPower = TuningConstants.CARGO_INTAKE_MOTOR_POWER;
        }
        else if (this.driver.getDigital(Operation.GrabberOuttakeCargo))
        {
            cargoMotorPower = TuningConstants.CARGO_OUTTAKE_MOTOR_POWER;
        }

        this.logger.logNumber(GrabberMechanism.logName, "cargo", cargoMotorPower);
        this.cargoMotor.set(cargoMotorPower);

        // wrist postions: Inner = piston closest to elevetor, Outer = piston farthest from elevator
        if (this.driver.getDigital(Operation.GrabberWristStartPosition))
        {
            this.wristInner.set(DoubleSolenoidValue.Reverse);
            this.wristOuter.set(DoubleSolenoidValue.Reverse);
            this.logger.logString(GrabberMechanism.logName, "wrist", "start");
        }
        else if (this.driver.getDigital(Operation.GrabberWristHatchPosition))
        {
            this.wristInner.set(DoubleSolenoidValue.Reverse);
            this.wristOuter.set(DoubleSolenoidValue.Forward);
            this.logger.logString(GrabberMechanism.logName, "wrist", "hatch");
        }
        else if (this.driver.getDigital(Operation.GrabberWristCargoPosition))
        {
            this.wristInner.set(DoubleSolenoidValue.Forward);
            this.wristOuter.set(DoubleSolenoidValue.Reverse);
            this.logger.logString(GrabberMechanism.logName, "wrist", "cargo");
        }
        else if (this.driver.getDigital(Operation.GrabberWristFloorPosition))
        {
            this.wristInner.set(DoubleSolenoidValue.Forward);
            this.wristOuter.set(DoubleSolenoidValue.Forward);
            this.logger.logString(GrabberMechanism.logName, "wrist", "floor");
        }
    }

    @Override
    public void setDriver(Driver driver)
    {
        this.driver = driver;
    }

    @Override
    public void stop()
    {
        this.kicker.set(DoubleSolenoidValue.Off);
        this.cargoMotor.set(0.0);

        this.wristInner.set(DoubleSolenoidValue.Off);
        this.wristOuter.set(DoubleSolenoidValue.Off);
    }
}