package frc.robot.mechanisms;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.robot.ElectronicsConstants;
import frc.robot.HardwareConstants;
import frc.robot.TuningConstants;
import frc.robot.driver.Operation;
import frc.robot.common.Helpers;
import frc.robot.common.IMechanism;
import frc.robot.common.robotprovider.ITalonSRX;
import frc.robot.common.robotprovider.TalonSRXControlMode;
import frc.robot.common.robotprovider.TalonSRXFeedbackDevice;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.common.robotprovider.DoubleSolenoidValue;
import frc.robot.common.robotprovider.IDashboardLogger;
import frc.robot.common.robotprovider.IDoubleSolenoid;
import frc.robot.driver.common.Driver;
import frc.robot.common.robotprovider.IRobotProvider;
import frc.robot.common.robotprovider.TalonSRXNeutralMode;
import frc.robot.common.robotprovider.TalonSRXLimitSwitchStatus;

/////////////////////////TODO//////////////////////////////////
//Add operations for:
//Ejecting hatch
//Cargo intake mode, cargo outtake mode
//wrist positions
//look at other code to see how to do this ^^
//set buttons
//add all constants (tuning, hardware, electronics)



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
    private final ITalonSRX cargoMotorMaster;
    //for wrist mechanism
    private final IDoubleSolenoid wristOne;
    private final IDoubleSolenoid wristTwo;

    @Inject
    public GrabberMechanism(IRobotProvider provider, IDashboardLogger logger) {
        //this.logger = logger;

        this.kicker = provider.getDoubleSolenoid(ElectronicsConstants.KICKER_FORWARD_CHANNEL, ElectronicsConstants.KICKER_REVERSE_CHANNEL);
        
        this.cargoMotorMaster = provider.getTalonSRX(ElectronicsConstants.GRABBER_MOTOR_MASTER_CAN_ID);

        this.wristOne = provider.getDoubleSolenoid(ElectronicsConstants.WRIST_ONE_FORWARD_CHANNEL, ElectronicsConstants.WRIST_ONE_REVERSE_CHANNEL);
        this.wristTwo = provider.getDoubleSolenoid(ElectronicsConstants.WRIST_TWO_FORWARD_CHANNEL, ElectronicsConstants.WRIST_TWO_REVERSE_CHANNEL);

    }

    @Override
    public void readSensors() {

    }

    @Override
    public void update() {
        
    }

    @Override
    public void setDriver(Driver driver) {
        this.driver = driver;
    }

    @Override
    public void stop() {
       this.kicker.set(DoubleSolenoidValue.Off);
       
       this.cargoMotorMaster.set(0.0);

       this.wristOne.set(DoubleSolenoidValue.Off);
       this.wristTwo.set(DoubleSolenoidValue.Off);
    }

}