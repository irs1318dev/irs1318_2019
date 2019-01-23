package frc.robot.mechanisms;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.robot.ElectronicsConstants;
//import frc.robot.HardwareConstants;
import frc.robot.TuningConstants;
import frc.robot.driver.Operation;
//import frc.robot.common.Helpers;
import frc.robot.common.IMechanism;
import frc.robot.common.robotprovider.ITalonSRX;
//import frc.robot.common.robotprovider.TalonSRXControlMode;
//import frc.robot.common.robotprovider.TalonSRXFeedbackDevice;
//import frc.robot.common.robotprovider.ITimer;
import frc.robot.common.robotprovider.DoubleSolenoidValue;
import frc.robot.common.robotprovider.IDashboardLogger;
import frc.robot.common.robotprovider.IDoubleSolenoid;
import frc.robot.driver.common.Driver;
import frc.robot.common.robotprovider.IRobotProvider;
//import frc.robot.common.robotprovider.TalonSRXNeutralMode;
//import frc.robot.common.robotprovider.TalonSRXLimitSwitchStatus;

@Singleton
public class GrabberMechanism implements IMechanism{
    //logger
    //private final IDashboardLogger logger;
    //private static final String logName = "grabMech";

    private Driver driver;

    //actuators
    //for ejecting hatches
    private final IDoubleSolenoid kicker;
    //for cargo intake/outtake
    private final ITalonSRX cargoMotor;
    //for wrist mechanism
    private final IDoubleSolenoid wristOne;
    private final IDoubleSolenoid wristTwo;

    @Inject
    public GrabberMechanism(IRobotProvider provider, IDashboardLogger logger) {
        //this.logger = logger;

        this.kicker = provider.getDoubleSolenoid(ElectronicsConstants.KICKER_FORWARD_CHANNEL, ElectronicsConstants.KICKER_REVERSE_CHANNEL);
        
        this.cargoMotor = provider.getTalonSRX(ElectronicsConstants.CARGO_MOTOR_CAN_ID);

        this.wristOne = provider.getDoubleSolenoid(ElectronicsConstants.WRIST_ONE_FORWARD_CHANNEL, ElectronicsConstants.WRIST_ONE_REVERSE_CHANNEL);
        this.wristTwo = provider.getDoubleSolenoid(ElectronicsConstants.WRIST_TWO_FORWARD_CHANNEL, ElectronicsConstants.WRIST_TWO_REVERSE_CHANNEL);

    }

    @Override
    public void readSensors() {
        //no sensors to read
    }

    @Override
    public void update() {
        
        //operations
        boolean kickerStowRequest = this.driver.getDigital(Operation.KickerStowedPosition);
        boolean kickerEjectHatchRequest = this.driver.getDigital(Operation.KickerEjectHatchPosition);

        boolean cargoIntakeRequest = this.driver.getDigital(Operation.CargoIntakeMode);
        boolean cargoOuttakeRequest = this.driver.getDigital(Operation.CargoOuttakeMode);

        //do operations for each individual solenoid in the wrist need to be added?
        boolean wristContractRequest = this.driver.getDigital(Operation.WristContractedPosition);
        boolean wristExtendRequest = this.driver.getDigital(Operation.WristExtendedPosition);


        if(kickerEjectHatchRequest) {
            //extend solenoid
            this.kicker.set(DoubleSolenoidValue.Forward);
        } else if(kickerStowRequest) {
            //retract solenoid
            this.kicker.set(DoubleSolenoidValue.Reverse);
        }

        if(cargoIntakeRequest) {
            //set motor to spin to intake cargo
            this.cargoMotor.set(TuningConstants.CARGO_INTAKE_MOTOR_POWER);
        } else if(cargoOuttakeRequest) {
            //set motor to spin to eject cargo
            this.cargoMotor.set(TuningConstants.CARGO_OUTTAKE_MOTOR_POWER);
        }

        if(wristExtendRequest) {
            //set both solenoids to extend
            this.wristOne.set(DoubleSolenoidValue.Forward);
            this.wristTwo.set(DoubleSolenoidValue.Forward);
        } else if(wristContractRequest) {
            //set both solenoids to contract
            this.wristOne.set(DoubleSolenoidValue.Reverse);
            this.wristTwo.set(DoubleSolenoidValue.Reverse);
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

       this.wristOne.set(DoubleSolenoidValue.Off);
       this.wristTwo.set(DoubleSolenoidValue.Off);
    }

}