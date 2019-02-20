package frc.robot.mechanisms;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.robot.ElectronicsConstants;
import frc.robot.HardwareConstants;
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
    private final IDoubleSolenoid finger; // for pulling panels out of brushes
    private final ITalonSRX cargoMotor; // for cargo intake/outtake
    private final IDoubleSolenoid wristInner; // for controlling the piston closest to the elevator
    private final IDoubleSolenoid wristOuter; // for controlling the piston farthest from the elevator

    private final IDigitalInput cargoLimitSwitch1;
    private final IDigitalInput cargoLimitSwitch2;
    private final IDigitalInput hatchLimitSwtich;

    private boolean cargoLimitSwitch1Status;
    private boolean cargoLimitSwitch2Status;
    private boolean hatchLimitSwitchStatus;

    @Inject
    public GrabberMechanism(IRobotProvider provider, IDashboardLogger logger) {

        this.kicker = provider.getDoubleSolenoid(ElectronicsConstants.GRABBER_KICKER_FORWARD_PCM_CHANNEL, ElectronicsConstants.GRABBER_KICKER_REVERSE_PCM_CHANNEL);
        this.finger = provider.getDoubleSolenoid(ElectronicsConstants.GRABBER_FINGER_FORWARD_PCM_CHANNEL, ElectronicsConstants.GRABBER_FINGER_REVERSE_PCM_CHANNEL);
        this.cargoMotor = provider.getTalonSRX(ElectronicsConstants.GRABBER_CARGO_MOTOR_CAN_ID);
        this.cargoMotor.setInvertOutput(HardwareConstants.GRABBER_CARGO_MOTOR_INVERT_OUTPUT);

        this.wristInner = provider.getDoubleSolenoid(ElectronicsConstants.GRABBER_WRIST_INNER_FORWARD_PCM_CHANNEL, ElectronicsConstants.GRABBER_WRIST_INNER_REVERSE_PCM_CHANNEL);
        this.wristOuter = provider.getDoubleSolenoid(ElectronicsConstants.GRABBER_WRIST_OUTER_FORWARD_PCM_CHANNEL, ElectronicsConstants.GRABBER_WRIST_OUTER_REVERSE_PCM_CHANNEL);

        this.cargoLimitSwitch1 = provider.getDigitalInput(ElectronicsConstants.GRABBER_CARGO_LIMIT_SWITCH_1_DIGITAL_CHANNEL);
        this.cargoLimitSwitch2 = provider.getDigitalInput(ElectronicsConstants.GRABBER_CARGO_LIMIT_SWITCH_2_DIGITAL_CHANNEL);
        this.hatchLimitSwtich = provider.getDigitalInput(ElectronicsConstants.GRABBER_HATCH_LIMIT_SWITCH_DIGITAL_CHANNEL);

        this.logger = logger;
    }

    public boolean hasCargo()
    {
        return this.cargoLimitSwitch1Status && this.cargoLimitSwitch2Status;
    }

    public boolean hasHatch()
    {
        return this.hatchLimitSwitchStatus;
    }

    @Override
    public void readSensors()
    {
        this.cargoLimitSwitch1Status = this.cargoLimitSwitch1.get();
        this.cargoLimitSwitch2Status = this.cargoLimitSwitch2.get();
        this.hatchLimitSwitchStatus = this.hatchLimitSwtich.get();

        this.logger.logBoolean(GrabberMechanism.logName, "cargo1", this.cargoLimitSwitch1Status);
        this.logger.logBoolean(GrabberMechanism.logName, "cargo2", this.cargoLimitSwitch2Status);
        this.logger.logBoolean(GrabberMechanism.logName, "hatch", this.hatchLimitSwitchStatus);
    }

    @Override
    public void update()
    {
        boolean kickPanel = this.driver.getDigital(Operation.GrabberKickPanel);
        this.kicker.set(kickPanel ? DoubleSolenoidValue.Forward : DoubleSolenoidValue.Reverse);
        this.logger.logBoolean(GrabberMechanism.logName, "kicker", kickPanel);

        boolean pointFinger = this.driver.getDigital(Operation.GrabberPointFinger);
        this.finger.set(pointFinger ? DoubleSolenoidValue.Reverse : DoubleSolenoidValue.Forward);
        this.logger.logBoolean(GrabberMechanism.logName, "finger", pointFinger);

        double cargoMotorPower = 0.0;
        if (this.driver.getDigital(Operation.GrabberIntakeCargo))
        {
            cargoMotorPower = TuningConstants.GRABBER_CARGO_INTAKE_MOTOR_POWER;
        }
        else if (this.driver.getDigital(Operation.GrabberOuttakeCargo))
        {
            cargoMotorPower = TuningConstants.GRABBER_CARGO_OUTTAKE_MOTOR_POWER;
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
        this.finger.set(DoubleSolenoidValue.Off);
        this.cargoMotor.set(0.0);

        this.wristInner.set(DoubleSolenoidValue.Off);
        this.wristOuter.set(DoubleSolenoidValue.Off);
    }
}