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
import frc.robot.common.robotprovider.IDashboardLogger;
import frc.robot.driver.common.Driver;
import frc.robot.common.robotprovider.IRobotProvider;
import frc.robot.common.robotprovider.TalonSRXNeutralMode;
import frc.robot.common.robotprovider.TalonSRXLimitSwitchStatus;



@Singleton
public class ElevatorMechanism implements IMechanism 
{

    private static final String logName = "el";
    private static final int pidSlotId = 0;

    private final ITimer timer;

    private final IDashboardLogger logger;

    private final ITalonSRX elevatorMotor;

    private final TalonSRXControlMode pidControlMode;

    private Driver driver;

    private double elevatorVelocity;
    private double elevatorError;
    private int elevatorPosition;
    private double elevatorHeight;
    private boolean elevatorForwardLimitSwitchStatus;
    private boolean elevatorReverseLimitSwitchStatus;

    private double desiredHeight;

    private double lastUpdateTime;

    @Inject
    public ElevatorMechanism(IDashboardLogger logger,
                             IRobotProvider provider,
                             ITimer timer) 
    {
        this.logger = logger;
        this.timer = timer;

        this.pidControlMode = TuningConstants.ELEVATOR_USE_MOTION_MAGIC
            ? TalonSRXControlMode.MotionMagicPosition
            : TalonSRXControlMode.Position;

        this.elevatorMotor = provider.getTalonSRX(ElectronicsConstants.ELEVATOR_MOTOR_CAN_ID);
        this.elevatorMotor.setNeutralMode(TalonSRXNeutralMode.Brake);
        this.elevatorMotor.setInvertOutput(TuningConstants.ELEVATOR_INVERT_OUTPUT);
        this.elevatorMotor.setInvertSensor(TuningConstants.ELEVATOR_INVERT_SENSOR);
        this.elevatorMotor.setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        this.elevatorMotor.setPosition(
            (int)(TuningConstants.ELEVATOR_BOTTOM_POSITION / HardwareConstants.ELEVATOR_PULSE_DISTANCE));
        this.elevatorMotor.setForwardLimitSwitch(
            TuningConstants.ELEVATOR_FORWARD_LIMIT_SWITCH_ENABLED,
            TuningConstants.ELEVATOR_FORWARD_LIMIT_SWITCH_NORMALLY_OPEN);
        this.elevatorMotor.setReverseLimitSwitch(
            TuningConstants.ELEVATOR_REVERSE_LIMIT_SWITCH_ENABLED,
            TuningConstants.ELEVATOR_REVERSE_LIMIT_SWITCH_NORMALLY_OPEN);

        if (TuningConstants.ELEVATOR_USE_MOTION_MAGIC)
        {
            this.elevatorMotor.setMotionMagicPIDF(
                TuningConstants.ELEVATOR_MM_POSITION_PID_KP,
                TuningConstants.ELEVATOR_MM_POSITION_PID_KI,
                TuningConstants.ELEVATOR_MM_POSITION_PID_KD,
                TuningConstants.ELEVATOR_MM_POSITION_PID_KF,
                TuningConstants.ELEVATOR_MM_POSITION_PID_CRUISE_VELOC,
                TuningConstants.ELEVATOR_MM_POSITION_PID_ACCEL,
                ElevatorMechanism.pidSlotId);
        }
        else
        {
            this.elevatorMotor.setPIDF(
                TuningConstants.ELEVATOR_POSITION_PID_KP,
                TuningConstants.ELEVATOR_POSITION_PID_KI,
                TuningConstants.ELEVATOR_POSITION_PID_KD,
                TuningConstants.ELEVATOR_POSITION_PID_KF,
                ElevatorMechanism.pidSlotId);
        }

        this.elevatorMotor.setControlMode(this.pidControlMode);
        this.elevatorMotor.setSelectedSlot(ElevatorMechanism.pidSlotId);

        this.elevatorVelocity = 0.0;
        this.elevatorError = 0.0;
        this.elevatorPosition = 0;
        this.elevatorHeight = 0.0;
        this.elevatorForwardLimitSwitchStatus = false;
        this.elevatorReverseLimitSwitchStatus = false;

        this.desiredHeight = 0.0;
    }

    public double getVelocity()
    {
        return this.elevatorVelocity;
    }

    public double getError()
    {
        return this.elevatorError;
    }

    public double getPosition()
    {
        return this.elevatorPosition;
    }

    public boolean getForwardLimitSwitchStatus() 
    {
        return this.elevatorForwardLimitSwitchStatus;
    }

    public boolean getReverseLimitSwitchStatus()
    {
        return this.elevatorReverseLimitSwitchStatus;
    }

    @Override
    public void setDriver(Driver driver)
    {
        this.driver = driver;
    }

    @Override
    public void readSensors() 
    {
        this.elevatorVelocity = this.elevatorMotor.getVelocity();
        this.elevatorError = this.elevatorMotor.getError();
        this.elevatorPosition = this.elevatorMotor.getPosition();
        this.elevatorHeight = this.elevatorPosition * HardwareConstants.ELEVATOR_PULSE_DISTANCE;

        TalonSRXLimitSwitchStatus limitSwitchStatus = this.elevatorMotor.getLimitSwitchStatus();
        this.elevatorForwardLimitSwitchStatus = limitSwitchStatus.isForwardClosed;
        this.elevatorReverseLimitSwitchStatus = limitSwitchStatus.isReverseClosed;
    
        this.logger.logNumber(ElevatorMechanism.logName, "elevatorVelocity", this.elevatorVelocity);
        this.logger.logNumber(ElevatorMechanism.logName, "elevatorError", this.elevatorError);
        this.logger.logNumber(ElevatorMechanism.logName, "elevatorPosition", this.elevatorPosition);
        this.logger.logNumber(ElevatorMechanism.logName, "elevatorHeight", this.elevatorHeight);
        this.logger.logBoolean(ElevatorMechanism.logName, "elevatorReverseLimitSwitch", this.elevatorReverseLimitSwitchStatus);
        this.logger.logBoolean(ElevatorMechanism.logName, "elevatorForwardLimitSwitch", this.elevatorForwardLimitSwitchStatus);
    }

    @Override
    public void update()
    {
        double currentTime = this.timer.get();
        double deltaTime = currentTime - this.lastUpdateTime;

        boolean forceUp = this.driver.getDigital(Operation.ElevatorForceUp);
        boolean forceDown = this.driver.getDigital(Operation.ElevatorForceDown);
        if (forceUp || forceDown)
        {
            this.desiredHeight = this.elevatorHeight;
            if (this.elevatorReverseLimitSwitchStatus || this.elevatorPosition < 0.0)
            {
                this.desiredHeight = 0.0;
                this.elevatorMotor.reset();
            }

            this.elevatorMotor.setControlMode(TalonSRXControlMode.PercentOutput);
            if (forceUp)
            {
                this.elevatorMotor.set(
                    this.elevatorForwardLimitSwitchStatus ? 0.0 : TuningConstants.ELEVATOR_DEBUG_UP_POWER_LEVEL);
            }
            else if (forceDown)
            {
                this.elevatorMotor.set(
                    this.elevatorReverseLimitSwitchStatus ? 0.0 : -TuningConstants.ELEVATOR_DEBUG_DOWN_POWER_LEVEL);
            }
        }
        else
        {
            double newDesiredHeight = this.desiredHeight;
            if (this.driver.getDigital(Operation.ElevatorBottomPosition))
            {
                newDesiredHeight = TuningConstants.ELEVATOR_BOTTOM_POSITION;
            }
            else if (this.driver.getDigital(Operation.ElevatorHatch2Position))
            {
                newDesiredHeight = TuningConstants.ELEVATOR_HATCH_2_POSITION;
            }
            else if (this.driver.getDigital(Operation.ElevatorHatch3Position))
            {
                newDesiredHeight = TuningConstants.ELEVATOR_HATCH_3_POSITION;
            }
            else if (this.driver.getDigital(Operation.ElevatorCargo1Position))
            {
                newDesiredHeight = TuningConstants.ELEVATOR_CARGO_1_POSITION;
            }
            else if (this.driver.getDigital(Operation.ElevatorCargo2Position))
            {
                newDesiredHeight = TuningConstants.ELEVATOR_CARGO_2_POSITION;
            }
            else if (this.driver.getDigital(Operation.ElevatorCargo3Position))
            {
                newDesiredHeight = TuningConstants.ELEVATOR_CARGO_3_POSITION;
            }
            else if (this.driver.getDigital(Operation.ElevatorCargoLoadPosition))
            {
                newDesiredHeight = TuningConstants.ELEVATOR_CARGO_LOAD_POSITION;
            }
            
            if (this.driver.getDigital(Operation.ElevatorMoveUp))
            {
                double deltaPosition = deltaTime * TuningConstants.ELEVATOR_MOVE_VELOCITY;
                double remainingHeight = HardwareConstants.ELEVATOR_MAX_HEIGHT - newDesiredHeight;
                if (deltaPosition > remainingHeight)
                {
                    newDesiredHeight = HardwareConstants.ELEVATOR_MAX_HEIGHT;
                    newDesiredHeight += (deltaPosition - remainingHeight);
                }
                else
                {
                    newDesiredHeight += deltaPosition;
                }
            }
            else if (this.driver.getDigital(Operation.ElevatorMoveDown))
            {
                double deltaPosition = deltaTime * TuningConstants.ELEVATOR_MOVE_VELOCITY;
                double remainingHeight = newDesiredHeight;
                if (deltaPosition > remainingHeight)
                {
                    newDesiredHeight = 0.0;
                    newDesiredHeight -= (deltaPosition - remainingHeight);
                }
                else
                {
                    newDesiredHeight -= deltaPosition;
                }
            }

            this.desiredHeight = Helpers.EnforceRange(
                                    newDesiredHeight,
                                    TuningConstants.ELEVATOR_BOTTOM_POSITION,
                                    HardwareConstants.ELEVATOR_MAX_HEIGHT);

            this.logger.logNumber(ElevatorMechanism.logName, "desiredHeight", this.desiredHeight);

            this.elevatorMotor.setControlMode(this.pidControlMode);
            this.elevatorMotor.set(this.desiredHeight / HardwareConstants.ELEVATOR_PULSE_DISTANCE);
        }

        this.lastUpdateTime = currentTime;
    }

    @Override
    public void stop()
    {
        this.elevatorMotor.stop();
        this.elevatorVelocity = 0.0;
        this.elevatorError = 0.0;
        this.elevatorPosition = 0;
        this.elevatorHeight = 0.0;
        this.elevatorForwardLimitSwitchStatus = false;
        this.elevatorReverseLimitSwitchStatus = false;
    }
}