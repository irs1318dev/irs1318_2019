package frc.robot.mechanisms;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.robot.ElectronicsConstants;
import frc.robot.HardwareConstants;
import frc.robot.TuningConstants;
import frc.robot.driver.DigitalOperation;
import frc.robot.common.IMechanism;
import frc.robot.driver.common.Driver;
import frc.robot.common.robotprovider.*;

@Singleton
public class GrabberMechanism implements IMechanism
{
    private enum GrabberPosition
    {
        Floor, // "180"
        Cargo, // "45"
        Hatch, // "90"
        Retracted;
    }

    private static final String logName = "gr";
    private final IDashboardLogger logger;

    // actuators
    private final IDoubleSolenoid kicker; // for ejecting hatches
    private final IDoubleSolenoid beak; // for pulling panels out of brushes
    private final ITalonSRX cargoMotor; // for cargo intake/outtake
    private final IDoubleSolenoid wristInner; // for controlling the piston closest to the elevator
    private final IDoubleSolenoid wristOuter; // for controlling the piston farthest from the elevator

    private final IDigitalInput cargoLimitSwitch;

    private Driver driver;

    private boolean cargoLimitSwitchStatus;

    private GrabberPosition currentPosition;

    @Inject
    public GrabberMechanism(IRobotProvider provider, IDashboardLogger logger)
    {
        this.logger = logger;
        this.kicker = provider.getDoubleSolenoid(ElectronicsConstants.GRABBER_KICKER_FORWARD_PCM_CHANNEL, ElectronicsConstants.GRABBER_KICKER_REVERSE_PCM_CHANNEL);
        this.beak = provider.getDoubleSolenoid(ElectronicsConstants.GRABBER_BEAK_FORWARD_PCM_CHANNEL, ElectronicsConstants.GRABBER_BEAK_REVERSE_PCM_CHANNEL);
        this.cargoMotor = provider.getTalonSRX(ElectronicsConstants.GRABBER_CARGO_MOTOR_CAN_ID);
        this.cargoMotor.setInvertOutput(HardwareConstants.GRABBER_CARGO_MOTOR_INVERT_OUTPUT);

        this.wristInner = provider.getDoubleSolenoid(ElectronicsConstants.GRABBER_WRIST_INNER_FORWARD_PCM_CHANNEL, ElectronicsConstants.GRABBER_WRIST_INNER_REVERSE_PCM_CHANNEL);
        this.wristOuter = provider.getDoubleSolenoid(ElectronicsConstants.GRABBER_WRIST_OUTER_FORWARD_PCM_CHANNEL, ElectronicsConstants.GRABBER_WRIST_OUTER_REVERSE_PCM_CHANNEL);

        this.cargoLimitSwitch = provider.getDigitalInput(ElectronicsConstants.GRABBER_CARGO_LIMIT_SWITCH_DIGITAL_CHANNEL);

        this.currentPosition = GrabberPosition.Retracted;
    }

    public boolean hasCargo()
    {
        return this.cargoLimitSwitchStatus;
    }

    public boolean isCargoMode()
    {
        return this.currentPosition == GrabberPosition.Cargo;
    }

    public boolean isHatchMode()
    {
        return this.currentPosition == GrabberPosition.Hatch;
    }

    @Override
    public void readSensors()
    {
        this.cargoLimitSwitchStatus = !this.cargoLimitSwitch.get();
        this.logger.logBoolean(GrabberMechanism.logName, "hasCargo", this.cargoLimitSwitchStatus);
    }

    @Override
    public void update()
    {
        boolean kickPanel = this.driver.getDigital(DigitalOperation.GrabberKickPanel);
        this.kicker.set(kickPanel ? DoubleSolenoidValue.Forward : DoubleSolenoidValue.Reverse);
        this.logger.logBoolean(GrabberMechanism.logName, "kicker", kickPanel);

        boolean pointBeak = kickPanel || this.driver.getDigital(DigitalOperation.GrabberPointBeak);
        this.beak.set(pointBeak ? DoubleSolenoidValue.Reverse : DoubleSolenoidValue.Forward);
        this.logger.logBoolean(GrabberMechanism.logName, "beak", pointBeak);

        double cargoMotorPower = 0.0;
        if (this.driver.getDigital(DigitalOperation.GrabberIntakeCargo))
        {
            cargoMotorPower = TuningConstants.GRABBER_CARGO_INTAKE_MOTOR_POWER;
        }
        else if (this.driver.getDigital(DigitalOperation.GrabberOuttakeCargo))
        {
            cargoMotorPower = TuningConstants.GRABBER_CARGO_OUTTAKE_MOTOR_POWER;
        }

        this.logger.logNumber(GrabberMechanism.logName, "cargo", cargoMotorPower);
        this.cargoMotor.set(cargoMotorPower);

        // wrist postions: Inner = piston closest to elevetor, Outer = piston farthest from elevator
        if (this.driver.getDigital(DigitalOperation.GrabberWristStartPosition))
        {
            this.currentPosition = GrabberPosition.Retracted;

            this.wristInner.set(DoubleSolenoidValue.Reverse);
            this.wristOuter.set(DoubleSolenoidValue.Reverse);
        }
        else if (this.driver.getDigital(DigitalOperation.GrabberWristHatchPosition))
        {
            this.currentPosition = GrabberPosition.Hatch;

            this.wristInner.set(DoubleSolenoidValue.Reverse);
            this.wristOuter.set(DoubleSolenoidValue.Forward);
        }
        else if (this.driver.getDigital(DigitalOperation.GrabberWristCargoPosition))
        {
            this.currentPosition = GrabberPosition.Cargo;

            this.wristInner.set(DoubleSolenoidValue.Forward);
            this.wristOuter.set(DoubleSolenoidValue.Reverse);
        }
        else if (this.driver.getDigital(DigitalOperation.GrabberWristFloorPosition))
        {
            this.currentPosition = GrabberPosition.Floor;

            this.wristInner.set(DoubleSolenoidValue.Forward);
            this.wristOuter.set(DoubleSolenoidValue.Forward);
        }

        this.logger.logString(GrabberMechanism.logName, "wrist", this.currentPosition.toString());
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
        this.beak.set(DoubleSolenoidValue.Off);
        this.cargoMotor.set(0.0);

        this.wristInner.set(DoubleSolenoidValue.Off);
        this.wristOuter.set(DoubleSolenoidValue.Off);
    }
}