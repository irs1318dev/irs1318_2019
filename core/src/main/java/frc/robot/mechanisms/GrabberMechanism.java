package frc.robot.mechanisms;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.robot.ElectronicsConstants;
import frc.robot.TuningConstants;
import frc.robot.driver.Operation;
import frc.robot.common.IMechanism;
import frc.robot.common.robotprovider.ITalonSRX;
import frc.robot.common.robotprovider.DoubleSolenoidValue;
import frc.robot.common.robotprovider.IDoubleSolenoid;
import frc.robot.driver.common.Driver;
import frc.robot.common.robotprovider.IRobotProvider;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.common.robotprovider.IDashboardLogger;

@Singleton
public class GrabberMechanism implements IMechanism {

    private static final String logName = "gr";
    private IDashboardLogger logger;

    private Driver driver;

    // actuators
    // for ejecting hatches
    private final IDoubleSolenoid kicker;
    // for cargo intake/outtake
    private final ITalonSRX cargoMotor;
    // for wrist mechanism
    private final IDoubleSolenoid wristInner;
    private final IDoubleSolenoid wristOuter;

    @Inject
    public GrabberMechanism(IRobotProvider provider, IDashboardLogger logger) {

        this.kicker = provider.getDoubleSolenoid(ElectronicsConstants.KICKER_FORWARD_CHANNEL,
                ElectronicsConstants.KICKER_REVERSE_CHANNEL);

        this.cargoMotor = provider.getTalonSRX(ElectronicsConstants.CARGO_MOTOR_CAN_ID);

        this.wristInner = provider.getDoubleSolenoid(ElectronicsConstants.WRIST_INNER_FORWARD_CHANNEL,
                ElectronicsConstants.WRIST_INNER_REVERSE_CHANNEL);

        this.wristOuter = provider.getDoubleSolenoid(ElectronicsConstants.WRIST_OUTER_FORWARD_CHANNEL,
                ElectronicsConstants.WRIST_OUTER_REVERSE_CHANNEL);

        this.logger = logger;

    }

    @Override
    public void readSensors() {
        // no sensors to read
    }

    @Override
    public void update() {

        // operations
        boolean kickerStowRequest = this.driver.getDigital(Operation.KickerStowedPosition);
        boolean kickerEjectHatchRequest = this.driver.getDigital(Operation.KickerEjectHatchPosition);

        boolean cargoIntakeRequest = this.driver.getDigital(Operation.CargoIntakeMode);
        boolean cargoOuttakeRequest = this.driver.getDigital(Operation.CargoOuttakeMode);

        // wrist postions: Inner = solenoid closest to elevetor, outer = solenoid farthest from elevator
        boolean wristInnerContract = this.driver.getDigital(Operation.WristInnerContract);
        boolean wristInnerExtend = this.driver.getDigital(Operation.WristInnerExtend);
        
        boolean wristOuterContract = this.driver.getDigital(Operation.WristOuterContract);
        boolean wristOuterExtend = this.driver.getDigital(Operation.WristOuterExtend);

        if (kickerEjectHatchRequest) {
            // extend solenoid
            this.kicker.set(DoubleSolenoidValue.Forward);

        } else if (kickerStowRequest) {
            // retract solenoid
            this.kicker.set(DoubleSolenoidValue.Reverse);

        }

        if (cargoIntakeRequest) {
            // set motor to spin to intake cargo
            this.cargoMotor.set(TuningConstants.CARGO_INTAKE_MOTOR_POWER);

        } else if (cargoOuttakeRequest) {
            // set motor to spin to eject cargo
            this.cargoMotor.set(TuningConstants.CARGO_OUTTAKE_MOTOR_POWER);

        }

        if(wristInnerContract) {
            // retract inner solenoid
            this.wristInner.set(DoubleSolenoidValue.Reverse);

        } else if(wristInnerExtend) {
            // extend inner solenoid
            this.wristInner.set(DoubleSolenoidValue.Forward);

        } else if(wristOuterContract) {
            // retract outer solenoid
            this.wristOuter.set(DoubleSolenoidValue.Reverse);

        } else if(wristOuterExtend) {
            // extend outer solenoid
            this.wristOuter.set(DoubleSolenoidValue.Forward);

        }

    }

    @Override
    public void setDriver(Driver driver) {
        this.driver = driver;
    }

    @Override
    public void stop() {
        this.kicker.set(DoubleSolenoidValue.Off);

        this.cargoMotor.set(0.0);

        this.wristInner.set(DoubleSolenoidValue.Off);
        this.wristOuter.set(DoubleSolenoidValue.Off);
    }

}