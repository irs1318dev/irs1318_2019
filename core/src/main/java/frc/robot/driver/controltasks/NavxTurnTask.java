package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.common.Helpers;
import frc.robot.common.PIDHandler;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.driver.Operation;
import frc.robot.driver.common.IControlTask;
import frc.robot.mechanisms.DriveTrainMechanism;
import frc.robot.mechanisms.PositionManager;

/**
 * Task that turns the robot a certain amount clockwise or counterclockwise in-place based on vision center
 */
public class NavxTurnTask extends ControlTaskBase implements IControlTask
{
    private final boolean useTime;
    private final double desiredAngle;
    private final double minRange;
    private final double maxRange;
    private final double waitTime;

    private double desiredTurnVelocity;
    private PIDHandler turnPidHandler;
    private Double completeTime;
    protected PositionManager pManager;
    protected DriveTrainMechanism dt;
    private boolean relativeMode;
    private double startingAngle;

    /**
    * Initializes a new NavxTurnTask using time to make sure we completed turn
    * @param desiredAngle the desired angle
    */
    public NavxTurnTask(double desiredAngle)
    {
        this(true, desiredAngle);
    }

    /**
    * Initializes a new NavxTurnTask
    * @param useTime whether to make sure we completed turn for a second or not
    * @param desiredAngle the desired angle
    */
    public NavxTurnTask(boolean useTime, double desiredAngle)
    {
        this(
            useTime,
            desiredAngle,
            TuningConstants.NAVX_TURN_MIN_ACCEPTABLE_ANGLE_VALUE,
            TuningConstants.NAVX_TURN_MAX_ACCEPTABLE_ANGLE_VALUE,
            TuningConstants.NAVX_TURN_COMPLETE_TIME, false);
    }

    public NavxTurnTask(boolean useTime, double desiredAngle, boolean relativeMode) {
        this(useTime,
        desiredAngle,
        TuningConstants.NAVX_TURN_MIN_ACCEPTABLE_ANGLE_VALUE,
        TuningConstants.NAVX_TURN_MAX_ACCEPTABLE_ANGLE_VALUE,
        TuningConstants.NAVX_TURN_COMPLETE_TIME, relativeMode);
    }

    /**
     * Initializes a new NavxTurnTask using a variable wait time after turn has reached the goal
     * @param desiredAngle the desired angle
     * @param waitTime the desired wait time
     */
    public NavxTurnTask(double desiredAngle, double waitTime)
    {
        this(true,
            desiredAngle,
            TuningConstants.NAVX_TURN_MIN_ACCEPTABLE_ANGLE_VALUE,
            TuningConstants.NAVX_TURN_MAX_ACCEPTABLE_ANGLE_VALUE,
            waitTime, false);

    }

    /**
    * Initializes a new NavxTurnTask
    * @param useTime whether to make sure we completed turn for a second or not
    * @param desiredAngle the desired angle
    * @param minRange the minimum of the measured angle range that we accept from the navx
    * @param maxRange the maximum of the measured angle range that we accept from the navx
    */
    public NavxTurnTask(boolean useTime, double desiredAngle, double minRange, double maxRange, double waitTime, boolean relativeMode)
    {
        this.useTime = useTime;
        this.desiredAngle = desiredAngle;
        this.minRange = minRange;
        this.maxRange = maxRange;
        this.waitTime = waitTime;
        this.relativeMode = relativeMode;
        this.startingAngle = 0;

        this.turnPidHandler = null;
        this.completeTime = null;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.pManager = this.getInjector().getInstance(PositionManager.class);
        this.dt = this.getInjector().getInstance(DriveTrainMechanism.class);
        this.turnPidHandler = this.createTurnHandler();
        if (this.relativeMode)
        {
            this.startingAngle = this.pManager.getNavxAngle();

            // if the navx isn't working, let's fall back to using odometry
            if (!this.pManager.getNavxIsConnected())
            {
                this.startingAngle = this.pManager.getOdometryAngle();
            }
        }
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);

        double currentMeasuredAngle = this.pManager.getNavxAngle();

        // if we are not within the expected range, let's fall back to using odometry
        if (!this.pManager.getNavxIsConnected()
            || !Helpers.WithinRange(currentMeasuredAngle, this.minRange, this.maxRange))
        {
            currentMeasuredAngle = this.pManager.getOdometryAngle();
        }

        this.desiredTurnVelocity = this.turnPidHandler.calculatePosition(this.desiredAngle + this.startingAngle, currentMeasuredAngle);
        
        this.setAnalogOperationState(
            Operation.DriveTrainTurn,
            this.desiredTurnVelocity);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);
        this.setAnalogOperationState(Operation.DriveTrainTurn, 0.0);
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        double currentMeasuredAngle = this.pManager.getNavxAngle();
        double currentTurnVelocity = this.dt.getLeftVelocity();

        // if we are not within the expected range, let's fall back to using odometry
        if (!this.pManager.getNavxIsConnected()
            || !Helpers.WithinRange(currentMeasuredAngle, this.minRange, this.maxRange))
        {
            currentMeasuredAngle = this.pManager.getOdometryAngle();
        }

        double centerAngleDifference = Math.abs(currentMeasuredAngle - (this.startingAngle + this.desiredAngle));
        if (centerAngleDifference > TuningConstants.MAX_NAVX_TURN_RANGE_DEGREES)
        {
            return false;
        }

        if (!this.useTime)
        {
            return true;
        }
        else
        {
            // If desired and current turn velocity are near 0, complete this task. Otherwise, use timer.
            if (Helpers.WithinDelta(currentTurnVelocity, 0.0, TuningConstants.NAVX_TURN_COMPLETE_CURRENT_VELOCITY_DELTA)
                && Helpers.WithinDelta(this.desiredTurnVelocity, 0.0, TuningConstants.NAVX_TURN_COMPLETE_DESIRED_VELOCITY_DELTA))
            {
                return true;
            }

            ITimer timer = this.getInjector().getInstance(ITimer.class);
            if (this.completeTime == null)
            {
                this.completeTime = timer.get();
                return false;
            }
            else if (timer.get() - this.completeTime < this.waitTime)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
    }

    protected PIDHandler createTurnHandler()
    {
        return new PIDHandler(
            TuningConstants.NAVX_TURN_PID_KP,
            TuningConstants.NAVX_TURN_PID_KI,
            TuningConstants.NAVX_TURN_PID_KD,
            TuningConstants.NAVX_TURN_PID_KF,
            TuningConstants.NAVX_TURN_PID_KS,
            TuningConstants.NAVX_TURN_PID_MIN,
            TuningConstants.NAVX_TURN_PID_MAX,
            this.getInjector().getInstance(ITimer.class));
    }
}
