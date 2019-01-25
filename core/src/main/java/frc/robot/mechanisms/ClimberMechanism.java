package frc.robot.mechanisms;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.robot.ElectronicsConstants;
import frc.robot.HardwareConstants;
import frc.robot.TuningConstants;
import frc.robot.driver.Operation;
import frc.robot.common.IMechanism;
import frc.robot.common.robotprovider.ITalonSRX;
import frc.robot.common.robotprovider.TalonSRXControlMode;
import frc.robot.common.robotprovider.TalonSRXFeedbackDevice;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.common.robotprovider.IDashboardLogger;
import frc.robot.driver.common.Driver;
import frc.robot.common.robotprovider.IRobotProvider;
import frc.robot.common.robotprovider.TalonSRXNeutralMode;



@Singleton
public class ClimberMechanism implements IMechanism
{
    private static final String logName = "climber";
    private static final int pidSlotId = 0;

    private final ITimer timer;

    private final IDashboardLogger logger;

    private final ITalonSRX climberArmsMotorMaster;
    private final ITalonSRX climberCamMotorMaster;

    private Driver driver;

    private double armsVelocity;
    private double armsError;
    private int armsPosition;

    private double camVelocity;
    private double camError;
    private int camPosition;

    private double desiredArmsPosition;
    private double desiredCamPosition;

    private double lastUpdateTime;

    @Inject 
    public ClimberMechanism(IDashboardLogger logger,
                             IRobotProvider provider,
                             ITimer timer)
    {
        this.logger = logger;
        this.timer = timer;

        this.climberArmsMotorMaster = provider.getTalonSRX(ElectronicsConstants.CLIMBER_ARMS_MOTOR_MASTER_ID);
        this.climberArmsMotorMaster.setNeutralMode(TalonSRXNeutralMode.Brake);
        this.climberArmsMotorMaster.setInvertOutput(HardwareConstants.CLIMBER_ARMS_MASTER_INVERT_OUTPUT);
        this.climberArmsMotorMaster.setInvertSensor(HardwareConstants.CLIMBER_ARMS_INVERT_SENSOR);
        this.climberArmsMotorMaster.setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        this.climberArmsMotorMaster.setPosition(
            (int)(TuningConstants.CLIMBER_ARMS_RETRACTED_POSITION / HardwareConstants.CLIMBER_ARMS_PULSE_DISTANCE));

        ITalonSRX climberArmsMotorFollower = provider.getTalonSRX(ElectronicsConstants.CLIMBER_ARMS_MOTOR_FOLLOWER_ID);
        climberArmsMotorFollower.setNeutralMode(TalonSRXNeutralMode.Brake);
        climberArmsMotorFollower.setInvertOutput(HardwareConstants.CLIMBER_ARMS_FOLLOWER_INVERT_OUTPUT);
        climberArmsMotorFollower.setControlMode(TalonSRXControlMode.Follower);
        climberArmsMotorFollower.set(ElectronicsConstants.CLIMBER_ARMS_MOTOR_MASTER_ID);

        this.climberArmsMotorMaster.setControlMode(TalonSRXControlMode.Position);
        this.climberArmsMotorMaster.setSelectedSlot(ClimberMechanism.pidSlotId);

        this.armsVelocity = 0.0;
        this.armsError = 0.0;
        this.armsPosition = 0;
        
        this.climberCamMotorMaster = provider.getTalonSRX(ElectronicsConstants.CLIMBER_CAM_MOTOR_MASTER_ID);
        this.climberCamMotorMaster.setNeutralMode(TalonSRXNeutralMode.Brake);
        this.climberCamMotorMaster.setInvertOutput(HardwareConstants.CLIMBER_CAM_MASTER_INVERT_OUTPUT);
        this.climberCamMotorMaster.setInvertSensor(HardwareConstants.CLIMBER_CAM_INVERT_SENSOR);
        this.climberCamMotorMaster.setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        this.climberCamMotorMaster.setPosition(
            (int)(TuningConstants.CLIMBER_CAM_STORED_POSITION / HardwareConstants.CLIMBER_CAM_PULSE_DISTANCE));

        ITalonSRX climberCamMotorFollower = provider.getTalonSRX(ElectronicsConstants.CLIMBER_CAM_MOTOR_FOLLOWER_ID);
        climberCamMotorFollower.setNeutralMode(TalonSRXNeutralMode.Brake);
        climberCamMotorFollower.setInvertOutput(HardwareConstants.CLIMBER_CAM_FOLLOWER_INVERT_OUTPUT);
        climberCamMotorFollower.setControlMode(TalonSRXControlMode.Follower);
        climberCamMotorFollower.set(ElectronicsConstants.CLIMBER_CAM_MOTOR_MASTER_ID);

        this.climberCamMotorMaster.setControlMode(TalonSRXControlMode.Position);
        this.climberCamMotorMaster.setSelectedSlot(ClimberMechanism.pidSlotId);

        this.camVelocity = 0.0;
        this.camError = 0.0;
        this.camPosition = 0;

    }

    public double getArmsVelocity() 
    {
        return this.armsVelocity;
    }
    public double getArmsError() 
    {
        return this.armsError;
    }
    public int getArmsPosition() {
        return this.armsPosition;
    }

    public double getCamVelocity() 
    {
        return this.camVelocity;
    }
    public double getCamError()
    {
        return this.camError;
    }
    public int getCamPosition() 
    {
        return this.camPosition;
    }

    @Override
    public void setDriver(Driver driver)
    {
        this.driver = driver;
    }

    @Override
    public void readSensors()
    {
        this.armsVelocity = this.climberArmsMotorMaster.getVelocity();
        this.armsError = this.climberArmsMotorMaster.getError();
        this.armsPosition = this.climberArmsMotorMaster.getPosition();

        this.camVelocity = this.climberCamMotorMaster.getVelocity();
        this.camError = this.climberCamMotorMaster.getError();
        this.camPosition = this.climberCamMotorMaster.getPosition();

        this.logger.logNumber(ClimberMechanism.logName, "armsVelocity", this.armsVelocity);
        this.logger.logNumber(ClimberMechanism.logName, "armsError", this.armsError);
        this.logger.logNumber(ClimberMechanism.logName, "armsPosition", this.armsPosition);

        this.logger.logNumber(ClimberMechanism.logName, "camVelocity", this.camVelocity);
        this.logger.logNumber(ClimberMechanism.logName, "camError", this.camError);
        this.logger.logNumber(ClimberMechanism.logName, "camPosition", this.camPosition);

    }

    @Override
    public void update()
    {
        double currentTime = this.timer.get();
        double deltaTime = currentTime - this.lastUpdateTime;

        if (this.driver.getDigital(Operation.ClimberArmsRetractedPosition))
        {
            this.desiredArmsPosition = TuningConstants.CLIMBER_ARMS_RETRACTED_POSITION;
        }
        else if (this.driver.getDigital(Operation.ClimberArmsLevel1Position)) {
            this.desiredArmsPosition = TuningConstants.CLIMBER_ARMS_LEVEL_1_POSITION;
        }
        else if (this.driver.getDigital(Operation.ClimberArmsLevel2Position)) {
            this.desiredArmsPosition = TuningConstants.CLIMBER_ARMS_LEVEL_2_POSITION;
        }
        
        if (this.driver.getDigital(Operation.ClimberCamStoredPosition)) {
            this.desiredCamPosition = TuningConstants.CLIMBER_CAM_STORED_POSITION;
        }
        else if (this.driver.getDigital(Operation.ClimberCamLevel1Position)) {
            this.desiredCamPosition = TuningConstants.CLIMBER_CAM_LEVEL_1_POSITION;
        }
        else if (this.driver.getDigital(Operation.ClimberCamLevel2Position)) {
            this.desiredCamPosition = TuningConstants.CLIMBER_CAM_LEVEL_2_POSITION;
        }

        if (this.driver.getDigital(Operation.ClimberArmsMoveForward))
        {
            double deltaPosition = deltaTime * TuningConstants.CLIMBER_ARMS_MOVE_VELOCITY;
            this.desiredArmsPosition += deltaPosition;
        }
        else if (this.driver.getDigital(Operation.ClimberArmsMoveBackward))
        {
            double deltaPosition = deltaTime * TuningConstants.CLIMBER_ARMS_MOVE_VELOCITY;
            this.desiredArmsPosition -= deltaPosition;
        }

        if (this.driver.getDigital(Operation.ClimberCamMoveForward))
        {
            double deltaPosition = deltaTime * TuningConstants.CLIMBER_CAM_MOVE_VELOCITY;
            this.desiredCamPosition += deltaPosition;
        }
        else if (this.driver.getDigital(Operation.ClimberCamMoveBackward))
        {
            double deltaPosition = deltaTime * TuningConstants.CLIMBER_CAM_MOVE_VELOCITY;
            this.desiredCamPosition -= deltaPosition;
        }

        this.logger.logNumber(ClimberMechanism.logName, "desiredArmsPosition", this.desiredArmsPosition);
        this.logger.logNumber(ClimberMechanism.logName, "desiredCamPosition", this.desiredCamPosition);
    
        this.climberArmsMotorMaster.setControlMode(TalonSRXControlMode.Position);
        this.climberArmsMotorMaster.set(this.desiredArmsPosition / HardwareConstants.CLIMBER_ARMS_PULSE_DISTANCE);
    
        this.climberCamMotorMaster.setControlMode(TalonSRXControlMode.Position);
        this.climberCamMotorMaster.set(this.desiredCamPosition / HardwareConstants.CLIMBER_CAM_PULSE_DISTANCE);
    
        this.lastUpdateTime = currentTime;
    }

    @Override
    public void stop()
    {
        this.climberArmsMotorMaster.stop();
        this.armsVelocity = 0.0;
        this.armsError = 0.0;
        this.armsPosition = 0;
        
        this.climberCamMotorMaster.stop();
        this.camVelocity = 0.0;
        this.camError = 0.0;
        this.camPosition = 0;
    }
}
