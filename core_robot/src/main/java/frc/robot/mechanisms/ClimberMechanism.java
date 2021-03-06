package frc.robot.mechanisms;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.robot.ElectronicsConstants;
import frc.robot.HardwareConstants;
import frc.robot.TuningConstants;
import frc.robot.driver.DigitalOperation;
import frc.robot.common.IMechanism;
import frc.robot.common.robotprovider.ITalonSRX;
import frc.robot.common.robotprovider.TalonSRXControlMode;
import frc.robot.common.robotprovider.TalonSRXFeedbackDevice;
import frc.robot.common.robotprovider.TalonSRXLimitSwitchStatus;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.common.robotprovider.IVictorSPX;
import frc.robot.common.robotprovider.IDashboardLogger;
import frc.robot.common.robotprovider.IDigitalInput;
import frc.robot.driver.common.Driver;
import frc.robot.common.robotprovider.IRobotProvider;
import frc.robot.common.robotprovider.MotorNeutralMode;

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
    private final IDigitalInput climberHeightSensor;

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
    private boolean climbedHeightStatus;

    // private double climbedHeight;

    private double desiredArmsPosition;
    private double desiredCamPosition;
    private double lastUpdateTime;

    @Inject 
    public ClimberMechanism(IDashboardLogger logger, IRobotProvider provider, ITimer timer)
    {
        this.logger = logger;
        this.timer = timer;

        this.climberArmsMotorMaster = provider.getTalonSRX(ElectronicsConstants.CLIMBER_ARMS_MOTOR_MASTER_CAN_ID);
        this.climberArmsMotorMaster.setNeutralMode(MotorNeutralMode.Brake);
        this.climberArmsMotorMaster.setInvertOutput(HardwareConstants.CLIMBER_ARMS_MASTER_INVERT_OUTPUT);
        this.climberArmsMotorMaster.setInvertSensor(HardwareConstants.CLIMBER_ARMS_INVERT_SENSOR);
        this.climberArmsMotorMaster.setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        this.climberArmsMotorMaster.setPosition(
            (int)(TuningConstants.CLIMBER_ARMS_RETRACTED_POSITION / HardwareConstants.CLIMBER_ARMS_PULSE_DISTANCE));
        this.climberArmsMotorMaster.setForwardLimitSwitch(
            TuningConstants.CLIMBER_ARMS_FORWARD_LIMIT_SWITCH_ENABLED,
            TuningConstants.CLIMBER_ARMS_FORWARD_LIMIT_SWITCH_NORMALLY_OPEN);
        this.climberArmsMotorMaster.setReverseLimitSwitch(
            TuningConstants.CLIMBER_ARMS_REVERSE_LIMIT_SWITCH_ENABLED,
            TuningConstants.CLIMBER_ARMS_REVERSE_LIMIT_SWITCH_NORMALLY_OPEN);
        this.climberArmsMotorMaster.configureAllowableClosedloopError(ClimberMechanism.pidSlotId, TuningConstants.CLIMBER_ARMS_ALLOWABLE_CLOSED_LOOP_ERROR);
        this.climberArmsMotorMaster.setSelectedSlot(ClimberMechanism.pidSlotId);
        if (TuningConstants.CLIMBER_ARMS_USE_MOTION_MAGIC)
        {
            this.climberArmsMotorMaster.setControlMode(TalonSRXControlMode.MotionMagicPosition);
            this.climberArmsMotorMaster.setMotionMagicPIDF(
                TuningConstants.CLIMBER_ARMS_MM_POSITION_PID_KP,
                TuningConstants.CLIMBER_ARMS_MM_POSITION_PID_KI,
                TuningConstants.CLIMBER_ARMS_MM_POSITION_PID_KD,
                TuningConstants.CLIMBER_ARMS_MM_POSITION_PID_KF,
                TuningConstants.CLIMBER_ARMS_MM_POSITION_PID_CRUISE_VELOC,
                TuningConstants.CLIMBER_ARMS_MM_POSITION_PID_ACCEL,
                ClimberMechanism.pidSlotId);
        }
        else
        {
            this.climberArmsMotorMaster.setControlMode(TalonSRXControlMode.Position);
            this.climberArmsMotorMaster.setPIDF(
                TuningConstants.CLIMBER_ARMS_POSITION_PID_KP,
                TuningConstants.CLIMBER_ARMS_POSITION_PID_KI,
                TuningConstants.CLIMBER_ARMS_POSITION_PID_KD,
                TuningConstants.CLIMBER_ARMS_POSITION_PID_KF,
                ClimberMechanism.pidSlotId);
        }

        ITalonSRX climberArmsMotorFollower = provider.getTalonSRX(ElectronicsConstants.CLIMBER_ARMS_MOTOR_FOLLOWER_CAN_ID);
        climberArmsMotorFollower.setNeutralMode(MotorNeutralMode.Brake);
        climberArmsMotorFollower.setInvertOutput(HardwareConstants.CLIMBER_ARMS_FOLLOWER_INVERT_OUTPUT);
        climberArmsMotorFollower.follow(this.climberArmsMotorMaster);

        this.armsVelocity = 0.0;
        this.armsError = 0.0;
        this.armsPosition = 0;
        this.armsForwardLimitSwitchStatus = false;
        this.armsReverseLimitSwitchStatus = false;
        
        this.climberCamMotorMaster = provider.getTalonSRX(ElectronicsConstants.CLIMBER_CAM_MOTOR_MASTER_CAN_ID);
        this.climberCamMotorMaster.setNeutralMode(MotorNeutralMode.Brake);
        this.climberCamMotorMaster.setInvertOutput(HardwareConstants.CLIMBER_CAM_MASTER_INVERT_OUTPUT);
        this.climberCamMotorMaster.setInvertSensor(HardwareConstants.CLIMBER_CAM_INVERT_SENSOR);
        this.climberCamMotorMaster.setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        this.climberCamMotorMaster.setPosition(
            (int)(TuningConstants.CLIMBER_CAM_STORED_POSITION / HardwareConstants.CLIMBER_CAM_PULSE_DISTANCE));

        this.climberCamMotorMaster.setSelectedSlot(ClimberMechanism.pidSlotId);
        if (TuningConstants.CLIMBER_CAM_USE_MOTION_MAGIC)
        {
            this.climberCamMotorMaster.setControlMode(TalonSRXControlMode.MotionMagicPosition);
            this.climberCamMotorMaster.setMotionMagicPIDF(
                TuningConstants.CLIMBER_CAM_MM_POSITION_PID_KP,
                TuningConstants.CLIMBER_CAM_MM_POSITION_PID_KI,
                TuningConstants.CLIMBER_CAM_MM_POSITION_PID_KD,
                TuningConstants.CLIMBER_CAM_MM_POSITION_PID_KF,
                TuningConstants.CLIMBER_CAM_MM_POSITION_PID_CRUISE_VELOC,
                TuningConstants.CLIMBER_CAM_MM_POSITION_PID_ACCEL,
                ClimberMechanism.pidSlotId);
        }
        else
        {
            this.climberCamMotorMaster.setControlMode(TalonSRXControlMode.Position);
            this.climberCamMotorMaster.setPIDF(
                TuningConstants.CLIMBER_CAM_POSITION_PID_KP,
                TuningConstants.CLIMBER_CAM_POSITION_PID_KI,
                TuningConstants.CLIMBER_CAM_POSITION_PID_KD,
                TuningConstants.CLIMBER_CAM_POSITION_PID_KF,
                ClimberMechanism.pidSlotId);    
        }

        IVictorSPX climberCamMotorFollower = provider.getVictorSPX(ElectronicsConstants.CLIMBER_CAM_MOTOR_FOLLOWER_CAN_ID);
        climberCamMotorFollower.setNeutralMode(MotorNeutralMode.Brake);
        climberCamMotorFollower.setInvertOutput(HardwareConstants.CLIMBER_CAM_FOLLOWER_INVERT_OUTPUT);
        climberCamMotorFollower.follow(this.climberCamMotorMaster);

        this.climberCamLimitSwitch = provider.getDigitalInput(ElectronicsConstants.CLIMBER_CAM_LIMIT_SWITCH_DIGITAL_CHANNEL);
        this.climberHeightSensor = provider.getDigitalInput(ElectronicsConstants.CLIMBER_HEIGHT_SENSOR_DIGITAL_CHANNEL);

        this.climbedHeightStatus = true;
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

    public double getArmsDesiredPosition()
    {
        return this.desiredArmsPosition;
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

    public double getCamDesiredPosition()
    {
        return this.desiredCamPosition;
    }

    public boolean getCamLimitSwitchStatus()
    {
        return this.camLimitSwitchStatus;
    }

    // new logic? - changes from false to true when climb completed, true when driving normally
    public boolean isClimbed()
    {
        return this.climbedHeightStatus;
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

        this.camLimitSwitchStatus = !this.climberCamLimitSwitch.get();

        this.climbedHeightStatus = !this.climberHeightSensor.get();

        this.logger.logNumber(ClimberMechanism.logName, "armsVelocity", this.armsVelocity);
        this.logger.logNumber(ClimberMechanism.logName, "armsError", this.armsError);
        this.logger.logNumber(ClimberMechanism.logName, "armsPosition", this.armsPosition);
        this.logger.logBoolean(ClimberMechanism.logName, "armsForwardLimitSwitchStatus", this.armsForwardLimitSwitchStatus);
        this.logger.logBoolean(ClimberMechanism.logName, "armsReverseLimitSwitchStatus", this.armsReverseLimitSwitchStatus);

        this.logger.logNumber(ClimberMechanism.logName, "camVelocity", this.camVelocity);
        this.logger.logNumber(ClimberMechanism.logName, "camError", this.camError);
        this.logger.logNumber(ClimberMechanism.logName, "camPosition", this.camPosition);
        this.logger.logBoolean(ClimberMechanism.logName, "camLimitSwitchStatus", this.camLimitSwitchStatus);
        this.logger.logBoolean(ClimberMechanism.logName, "climbedHeightStatus", this.climbedHeightStatus);
    }

    @Override
    public void update()
    {
        double currentTime = this.timer.get();
        double deltaTime = currentTime - this.lastUpdateTime;

        boolean forceArmsForward = this.driver.getDigital(DigitalOperation.ClimberArmsForceForward);
        boolean forceArmsBack = this.driver.getDigital(DigitalOperation.ClimberArmsForceBackward);
        boolean forceArmsZero = this.driver.getDigital(DigitalOperation.ClimberArmsForceZero);
        if (forceArmsForward || forceArmsBack || forceArmsZero || !TuningConstants.CLIMBER_ARMS_USE_PID) 
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
                this.climberArmsMotorMaster.setPosition(TuningConstants.CLIMBER_ARMS_POSITION_MAX);
            }

            this.logger.logNumber(ClimberMechanism.logName, "desiredArmsPosition", this.desiredArmsPosition);
            this.climberArmsMotorMaster.setControlMode(TalonSRXControlMode.PercentOutput);
            if (forceArmsForward && !this.armsForwardLimitSwitchStatus)
            {
                this.climberArmsMotorMaster.set(TuningConstants.CLIMBER_ARMS_DEBUG_FORWARD_POWER_LEVEL);
            }
            else if (forceArmsBack && !this.armsReverseLimitSwitchStatus)
            {
                this.climberArmsMotorMaster.set(TuningConstants.CLIMBER_ARMS_DEBUG_BACKWARDS_POWER_LEVEL);
            }
            else
            {
                this.climberArmsMotorMaster.set(0.0);
            }
        }
        else
        {
            if (this.driver.getDigital(DigitalOperation.ClimberArmsRetractedPosition))
            {
                this.desiredArmsPosition = TuningConstants.CLIMBER_ARMS_RETRACTED_POSITION;
            }
            else if (this.driver.getDigital(DigitalOperation.ClimberArmsPrepClimbPosition))
            {
                this.desiredArmsPosition = TuningConstants.CLIMBER_ARMS_PREP_CLIMB_POSITION;
            }
            else if (this.driver.getDigital(DigitalOperation.ClimberArmsHighClimbPosition))
            {
                this.desiredArmsPosition = TuningConstants.CLIMBER_ARMS_HIGH_CLIMB_POSITION;
            }

            if (this.driver.getDigital(DigitalOperation.ClimberArmsMoveForward))
            {
                double deltaPosition = deltaTime * TuningConstants.CLIMBER_ARMS_MOVE_VELOCITY;
                this.desiredArmsPosition += deltaPosition;
            }
            else if (this.driver.getDigital(DigitalOperation.ClimberArmsMoveBackward))
            {
                double deltaPosition = deltaTime * TuningConstants.CLIMBER_ARMS_MOVE_VELOCITY;
                this.desiredArmsPosition -= deltaPosition;
            }

            this.logger.logNumber(ClimberMechanism.logName, "desiredArmsPosition", this.desiredArmsPosition);
            if (TuningConstants.CLIMBER_ARMS_USE_MOTION_MAGIC)
            {
                this.climberArmsMotorMaster.setControlMode(TalonSRXControlMode.MotionMagicPosition);   
            }
            else
            {
                this.climberArmsMotorMaster.setControlMode(TalonSRXControlMode.Position);
            }

            this.climberArmsMotorMaster.set(this.desiredArmsPosition / HardwareConstants.CLIMBER_ARMS_PULSE_DISTANCE);
        }

        boolean forceCamForward = this.driver.getDigital(DigitalOperation.ClimberCamForceForward);
        boolean forceCamBack = this.driver.getDigital(DigitalOperation.ClimberCamForceBackward);
        if (forceCamForward || forceCamBack || !TuningConstants.CLIMBER_CAM_USE_PID) 
        {
            this.desiredCamPosition = this.camPosition;
            if (this.camLimitSwitchStatus)
            {
                this.desiredCamPosition = 0.0;
                this.climberCamMotorMaster.reset();
            }

            this.logger.logNumber(ClimberMechanism.logName, "desiredCamPosition", this.desiredCamPosition);
            this.climberCamMotorMaster.setControlMode(TalonSRXControlMode.PercentOutput);
            if (forceCamForward)
            {
                this.climberCamMotorMaster.set(TuningConstants.CLIMBER_CAM_DEBUG_FORWARD_POWER_LEVEL);
            }
            else if (forceCamBack)
            {
                this.climberCamMotorMaster.set(TuningConstants.CLIMBER_CAM_DEBUG_BACKWARDS_POWER_LEVEL);
            }
            else
            {
                this.climberCamMotorMaster.set(0.0);
            }
        }
        else
        {
            // note - we want the desired position to increase in the positive direction when going to a setpoint...
            if (this.driver.getDigital(DigitalOperation.ClimberCamStoredPosition))
            {
                this.desiredCamPosition = ClimberMechanism.updateCamRotation(this.camPosition, TuningConstants.CLIMBER_CAM_STORED_POSITION);
            }
            else if (this.driver.getDigital(DigitalOperation.ClimberCamLowClimbPosition))
            {
                this.desiredCamPosition = ClimberMechanism.updateCamRotation(this.camPosition, TuningConstants.CLIMBER_CAM_LOW_CLIMB_POSITION);
            }
            else if (this.driver.getDigital(DigitalOperation.ClimberCamHighClimbPosition))
            {
                this.desiredCamPosition = ClimberMechanism.updateCamRotation(this.camPosition, TuningConstants.CLIMBER_CAM_HIGH_CLIMB_POSITION);
            }
            else if (this.driver.getDigital(DigitalOperation.ClimberCamOutOfWayPosition))
            {
                this.desiredCamPosition = ClimberMechanism.updateCamRotation(this.camPosition, TuningConstants.CLIMBER_CAM_OUT_OF_WAY_POSITION);
            }

            if (this.driver.getDigital(DigitalOperation.ClimberCamMoveForward))
            {
                double deltaPosition = deltaTime * TuningConstants.CLIMBER_CAM_MOVE_VELOCITY;
                this.desiredCamPosition += deltaPosition;
            }
            else if (this.driver.getDigital(DigitalOperation.ClimberCamMoveBackward))
            {
                double deltaPosition = deltaTime * TuningConstants.CLIMBER_CAM_MOVE_VELOCITY;
                this.desiredCamPosition -= deltaPosition;
            }

            this.logger.logNumber(ClimberMechanism.logName, "desiredCamPosition", this.desiredCamPosition);
            if (TuningConstants.CLIMBER_CAM_USE_MOTION_MAGIC)
            {
                this.climberCamMotorMaster.setControlMode(TalonSRXControlMode.MotionMagicPosition);   
            }
            else
            {
                this.climberCamMotorMaster.setControlMode(TalonSRXControlMode.Position);
            }

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
        this.climbedHeightStatus = true;

    }

    private static double updateCamRotation(double currentPosition, double newPositionOffset)
    {
        double currentRotations = currentPosition / TuningConstants.CLIMBER_CAM_FULL_ROTATION;
        double prevFullRotation = Math.floor(currentRotations) * TuningConstants.CLIMBER_CAM_FULL_ROTATION;

        double absDiff = Math.abs((prevFullRotation + newPositionOffset) - currentPosition);
        if ((prevFullRotation + newPositionOffset) >= currentPosition || absDiff < 200.0)
        {
            return prevFullRotation + newPositionOffset;
        }

        double nextFullRotation = prevFullRotation + TuningConstants.CLIMBER_CAM_FULL_ROTATION;
        return nextFullRotation + newPositionOffset;
    }
}
