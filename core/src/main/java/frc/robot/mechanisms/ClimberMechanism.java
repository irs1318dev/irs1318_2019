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
import frc.robot.common.robotprovider.TalonSRXLimitSwitchStatus;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.common.robotprovider.IAnalogInput;
import frc.robot.common.robotprovider.IDashboardLogger;
import frc.robot.common.robotprovider.IDigitalInput;
import frc.robot.driver.common.Driver;
import frc.robot.common.robotprovider.IRobotProvider;
import frc.robot.common.robotprovider.TalonSRXNeutralMode;

@Singleton
public class ClimberMechanism implements IMechanism
{
    private static final String logName = "cl";
    private static final int pidSlotId = 0;

    private final ITimer timer;

    private final IDashboardLogger logger;

    private final ITalonSRX climberArmsMotorMaster;
    private final ITalonSRX climberCamMotorMaster;
    private final IDigitalInput climberCamLimitSwitch;
    private final IAnalogInput climberHeightSensor;

    private Driver driver;

    private double armsVelocity;
    private double armsError;
    private int armsPosition;
    private boolean armsForwardLimitSwitchStatus;
    private boolean armsReverseLimitSwitchStatus;

    private double camVelocity;
    private double camError;
    private int camPosition;
    private boolean camLimitSwitchStatus;

    private double climbedHeight;

    private double desiredArmsPosition;
    private double desiredCamPosition;
    private double lastUpdateTime;

    @Inject 
    public ClimberMechanism(IDashboardLogger logger, IRobotProvider provider, ITimer timer)
    {
        this.logger = logger;
        this.timer = timer;

        this.climberArmsMotorMaster = provider.getTalonSRX(ElectronicsConstants.CLIMBER_ARMS_MOTOR_MASTER_CAN_ID);
        this.climberArmsMotorMaster.setNeutralMode(TalonSRXNeutralMode.Brake);
        this.climberArmsMotorMaster.setInvertOutput(HardwareConstants.CLIMBER_ARMS_MASTER_INVERT_OUTPUT);
        this.climberArmsMotorMaster.setInvertSensor(HardwareConstants.CLIMBER_ARMS_INVERT_SENSOR);
        this.climberArmsMotorMaster.setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        this.climberArmsMotorMaster.setPIDF(
            TuningConstants.CLIMBER_ARMS_POSITION_PID_KP,
            TuningConstants.CLIMBER_ARMS_POSITION_PID_KI,
            TuningConstants.CLIMBER_ARMS_POSITION_PID_KD,
            TuningConstants.CLIMBER_ARMS_POSITION_PID_KF,
            ClimberMechanism.pidSlotId);
        this.climberArmsMotorMaster.setPosition(
            (int)(TuningConstants.CLIMBER_ARMS_RETRACTED_POSITION / HardwareConstants.CLIMBER_ARMS_PULSE_DISTANCE));
        this.climberArmsMotorMaster.setForwardLimitSwitch(
            TuningConstants.CLIMBER_ARMS_FORWARD_LIMIT_SWITCH_ENABLED,
            TuningConstants.CLIMBER_ARMS_FORWARD_LIMIT_SWITCH_NORMALLY_OPEN);
        this.climberArmsMotorMaster.setReverseLimitSwitch(
            TuningConstants.CLIMBER_ARMS_REVERSE_LIMIT_SWITCH_ENABLED,
            TuningConstants.CLIMBER_ARMS_REVERSE_LIMIT_SWITCH_NORMALLY_OPEN);

        this.climberArmsMotorMaster.setControlMode(TalonSRXControlMode.Position);
        this.climberArmsMotorMaster.setSelectedSlot(ClimberMechanism.pidSlotId);    

        ITalonSRX climberArmsMotorFollower = provider.getTalonSRX(ElectronicsConstants.CLIMBER_ARMS_MOTOR_FOLLOWER_CAN_ID);
        climberArmsMotorFollower.setNeutralMode(TalonSRXNeutralMode.Brake);
        climberArmsMotorFollower.setInvertOutput(HardwareConstants.CLIMBER_ARMS_FOLLOWER_INVERT_OUTPUT);
        climberArmsMotorFollower.setControlMode(TalonSRXControlMode.Follower);
        climberArmsMotorFollower.set(ElectronicsConstants.CLIMBER_ARMS_MOTOR_MASTER_CAN_ID);

        this.armsVelocity = 0.0;
        this.armsError = 0.0;
        this.armsPosition = 0;
        this.armsForwardLimitSwitchStatus = false;
        this.armsReverseLimitSwitchStatus = false;
        
        this.climberCamMotorMaster = provider.getTalonSRX(ElectronicsConstants.CLIMBER_CAM_MOTOR_MASTER_CAN_ID);
        this.climberCamMotorMaster.setNeutralMode(TalonSRXNeutralMode.Brake);
        this.climberCamMotorMaster.setInvertOutput(HardwareConstants.CLIMBER_CAM_MASTER_INVERT_OUTPUT);
        this.climberCamMotorMaster.setInvertSensor(HardwareConstants.CLIMBER_CAM_INVERT_SENSOR);
        this.climberCamMotorMaster.setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        this.climberCamMotorMaster.setPIDF(
            TuningConstants.CLIMBER_CAM_POSITION_PID_KP,
            TuningConstants.CLIMBER_CAM_POSITION_PID_KI,
            TuningConstants.CLIMBER_CAM_POSITION_PID_KD,
            TuningConstants.CLIMBER_CAM_POSITION_PID_KF,
            ClimberMechanism.pidSlotId);
        this.climberCamMotorMaster.setPosition(
            (int)(TuningConstants.CLIMBER_CAM_STORED_POSITION / HardwareConstants.CLIMBER_CAM_PULSE_DISTANCE));

        this.climberCamMotorMaster.setControlMode(TalonSRXControlMode.Position);
        this.climberCamMotorMaster.setSelectedSlot(ClimberMechanism.pidSlotId);

        ITalonSRX climberCamMotorFollower = provider.getTalonSRX(ElectronicsConstants.CLIMBER_CAM_MOTOR_FOLLOWER_CAN_ID);
        climberCamMotorFollower.setNeutralMode(TalonSRXNeutralMode.Brake);
        climberCamMotorFollower.setInvertOutput(HardwareConstants.CLIMBER_CAM_FOLLOWER_INVERT_OUTPUT);
        climberCamMotorFollower.setControlMode(TalonSRXControlMode.Follower);
        climberCamMotorFollower.set(ElectronicsConstants.CLIMBER_CAM_MOTOR_MASTER_CAN_ID);

        this.climberCamLimitSwitch = provider.getDigitalInput(ElectronicsConstants.CLIMBER_CAM_LIMIT_SWITCH_DIGITAL_CHANNEL);
        this.climberHeightSensor = provider.getAnalogInput(ElectronicsConstants.CLIMBER_HEIGHT_SENSOR_ANALOG_CHANNEL);

        this.climbedHeight = 0.0;
        this.camVelocity = 0.0;
        this.camError = 0.0;
        this.camPosition = 0;
        this.camLimitSwitchStatus = false;
    }

    public double getArmsVelocity() 
    {
        return this.armsVelocity;
    }

    public double getArmsError() 
    {
        return this.armsError;
    }

    public int getArmsPosition()
    {
        return this.armsPosition;
    }

    public boolean getArmsForwardLimitSwitchStatus()
    {
        return this.armsForwardLimitSwitchStatus;
    }

    public boolean getArmsReverseLimitSwitchStatus()
    {
        return this.armsReverseLimitSwitchStatus;
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

    public boolean getCamLimitSwitchStatus()
    {
        return this.camLimitSwitchStatus;
    }

    public boolean isClimbed()
    {
        return this.climbedHeight > TuningConstants.CLIMBER_CLIMB_COMPLETED_VOLTAGE;
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

        TalonSRXLimitSwitchStatus armsLimitSwitchStatus = this.climberArmsMotorMaster.getLimitSwitchStatus();
        this.armsForwardLimitSwitchStatus = armsLimitSwitchStatus.isForwardClosed;
        this.armsReverseLimitSwitchStatus = armsLimitSwitchStatus.isReverseClosed;

        this.camVelocity = this.climberCamMotorMaster.getVelocity();
        this.camError = this.climberCamMotorMaster.getError();
        this.camPosition = this.climberCamMotorMaster.getPosition();

        this.camLimitSwitchStatus = this.climberCamLimitSwitch.get();

        this.climbedHeight = this.climberHeightSensor.getVoltage();

        this.logger.logNumber(ClimberMechanism.logName, "armsVelocity", this.armsVelocity);
        this.logger.logNumber(ClimberMechanism.logName, "armsError", this.armsError);
        this.logger.logNumber(ClimberMechanism.logName, "armsPosition", this.armsPosition);
        this.logger.logBoolean(ClimberMechanism.logName, "armsForwardLimitSwitchStatus", this.armsForwardLimitSwitchStatus);
        this.logger.logBoolean(ClimberMechanism.logName, "armsReverseLimitSwitchStatus", this.armsReverseLimitSwitchStatus);

        this.logger.logNumber(ClimberMechanism.logName, "camVelocity", this.camVelocity);
        this.logger.logNumber(ClimberMechanism.logName, "camError", this.camError);
        this.logger.logNumber(ClimberMechanism.logName, "camPosition", this.camPosition);
        this.logger.logBoolean(ClimberMechanism.logName, "camLimitSwitchStatus", this.camLimitSwitchStatus);
        this.logger.logNumber(ClimberMechanism.logName, "climbedHeight", this.climbedHeight);
    }

    @Override
    public void update()
    {
        double currentTime = this.timer.get();
        double deltaTime = currentTime - this.lastUpdateTime;

        boolean forceArmsForward = this.driver.getDigital(Operation.ClimberArmsForceForward);
        boolean forceArmsBack = this.driver.getDigital(Operation.ClimberArmsForceBackward);
        if (forceArmsForward || forceArmsBack) 
        {
            this.desiredArmsPosition = this.armsPosition;
            if (this.armsReverseLimitSwitchStatus || this.armsPosition < 0.0)
            {
                this.desiredArmsPosition = 0.0;
                this.climberArmsMotorMaster.reset();
            }

            if (this.armsForwardLimitSwitchStatus || this.armsPosition > TuningConstants.CLIMBER_ARMS_POSITION_MAX)
            {
                this.desiredArmsPosition = TuningConstants.CLIMBER_ARMS_POSITION_MAX;
                this.climberArmsMotorMaster.reset();
            }

            this.climberArmsMotorMaster.setControlMode(TalonSRXControlMode.PercentOutput);

            if (forceArmsForward)
            {
                this.climberArmsMotorMaster.set(TuningConstants.CLIMBER_ARMS_DEBUG_FORWARD_POWER_LEVEL);
            }
            else if (forceArmsBack)
            {
                this.climberArmsMotorMaster.set(TuningConstants.CLIMBER_ARMS_DEBUG_BACKWARDS_POWER_LEVEL);
            }
        }
        else
        {
            if (this.driver.getDigital(Operation.ClimberArmsRetractedPosition))
            {
                this.desiredArmsPosition = TuningConstants.CLIMBER_ARMS_RETRACTED_POSITION;
            }
            else if (this.driver.getDigital(Operation.ClimberArmsLowClimbPosition))
            {
                this.desiredArmsPosition = TuningConstants.CLIMBER_ARMS_LOW_CLIMB_POSITION;
            }
            else if (this.driver.getDigital(Operation.ClimberArmsHighClimbPosition))
            {
                this.desiredArmsPosition = TuningConstants.CLIMBER_ARMS_HIGH_CLIMB_POSITION;
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

            this.logger.logNumber(ClimberMechanism.logName, "desiredArmsPosition", this.desiredArmsPosition);
            
            this.climberArmsMotorMaster.setControlMode(TalonSRXControlMode.Position);
            this.climberArmsMotorMaster.set(this.desiredArmsPosition / HardwareConstants.CLIMBER_ARMS_PULSE_DISTANCE);
        }

        boolean forceCamForward = this.driver.getDigital(Operation.ClimberCamForceForward);
        boolean forceCamBack = this.driver.getDigital(Operation.ClimberCamForceBackward);
        if (forceCamForward || forceCamBack) 
        {
            this.desiredCamPosition = this.camPosition;
            if (this.camLimitSwitchStatus)
            {
                this.desiredCamPosition = 0.0;
                this.climberCamMotorMaster.reset();
            }

            this.climberCamMotorMaster.setControlMode(TalonSRXControlMode.PercentOutput);

            if (forceCamForward)
            {
                this.climberCamMotorMaster.set(TuningConstants.CLIMBER_CAM_DEBUG_FORWARD_POWER_LEVEL);
            }
            else if (forceCamBack)
            {
                this.climberCamMotorMaster.set(TuningConstants.CLIMBER_CAM_DEBUG_BACKWARDS_POWER_LEVEL);
            }
        }
        else
        {   
            if (this.driver.getDigital(Operation.ClimberCamStoredPosition))
            {
                this.desiredCamPosition = TuningConstants.CLIMBER_CAM_STORED_POSITION;
            }
            else if (this.driver.getDigital(Operation.ClimberCamLowClimbPosition))
            {
                this.desiredCamPosition = TuningConstants.CLIMBER_CAM_LOW_CLIMB_POSITION;
            }
            else if (this.driver.getDigital(Operation.ClimberCamHighClimbPosition))
            {
                this.desiredCamPosition = TuningConstants.CLIMBER_CAM_HIGH_CLIMB_POSITION;
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

            this.logger.logNumber(ClimberMechanism.logName, "desiredCamPosition", this.desiredCamPosition);
                
            this.climberCamMotorMaster.setControlMode(TalonSRXControlMode.Position);
            this.climberCamMotorMaster.set(this.desiredCamPosition / HardwareConstants.CLIMBER_CAM_PULSE_DISTANCE);
        }

        this.lastUpdateTime = currentTime;
    }

    @Override
    public void stop()
    {
        this.climberArmsMotorMaster.stop();
        this.armsVelocity = 0.0;
        this.armsError = 0.0;
        this.armsPosition = 0;
        this.armsForwardLimitSwitchStatus = false;
        this.armsReverseLimitSwitchStatus = false;

        this.climberCamMotorMaster.stop();
        this.camVelocity = 0.0;
        this.camError = 0.0;
        this.camPosition = 0;
        this.camLimitSwitchStatus = false;
        this.climbedHeight = 0.0;

    }
}
