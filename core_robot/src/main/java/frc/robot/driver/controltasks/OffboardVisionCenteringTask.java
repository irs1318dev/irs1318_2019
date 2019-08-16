package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.common.PIDHandler;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.driver.Operation;
import frc.robot.driver.common.IControlTask;
import frc.robot.mechanisms.OffboardVisionManager;
import frc.robot.mechanisms.VisionManager;

/**
 * Task that turns the robot a certain amount clockwise or counterclockwise in-place based on vision center
 */
public class OffboardVisionCenteringTask extends ControlTaskBase implements IControlTask
{
    private static final int NO_CENTER_THRESHOLD = 40;

    private final boolean useTime;

    private PIDHandler turnPidHandler;
    private Double centeredTime;
    protected OffboardVisionManager offboardVisionManager;

    private int noCenterCount;

    /**
    * Initializes a new OffboardVisionCenteringTask
    */
    public OffboardVisionCenteringTask()
    {
        this(true);
    }

    /**
    * Initializes a new OffboardVisionCenteringTask
    * @param useTime whether to make sure we are centered for a second or not
    */
    public OffboardVisionCenteringTask(boolean useTime)
    {
        this.useTime = useTime;

        this.turnPidHandler = null;
        this.centeredTime = null;

        this.noCenterCount = 0;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.offboardVisionManager = this.getInjector().getInstance(OffboardVisionManager.class);
        this.turnPidHandler = this.createTurnHandler();
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);

        double currentMeasuredAngle = this.offboardVisionManager.getBallAngle();        
        this.setAnalogOperationState(
            Operation.DriveTrainTurn,
            -this.turnPidHandler.calculatePosition(-10.0, currentMeasuredAngle));
        
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);
        this.setAnalogOperationState(Operation.DriveTrainTurn, 0.0);

        this.setDigitalOperationState(Operation.VisionDisable, true);
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        double currentMeasuredAngle = this.offboardVisionManager.getBallAngle();

        double centerAngleDifference = Math.abs(currentMeasuredAngle);
        if (centerAngleDifference > TuningConstants.MAX_VISION_CENTERING_RANGE_DEGREES)
        {
            return false;
        }

        if (!this.useTime)
        {
            return true;
        }
        else
        {
            ITimer timer = this.getInjector().getInstance(ITimer.class);
            if (this.centeredTime == null)
            {
                this.centeredTime = timer.get();
                return false;
            }
            else if (timer.get() - this.centeredTime < 0.75)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
    }

    /**
     * Checks whether this task should be stopped, or whether it should continue being processed.
     * @return true if we should cancel this task (and stop performing any subsequent tasks), otherwise false (to keep processing this task)
     */
    @Override
    public boolean shouldCancel()
    {
        if (!this.offboardVisionManager.getBallSeen())
        {
            this.noCenterCount++;
        }
        else
        {
            this.noCenterCount = 0;
        }

        return this.noCenterCount >= OffboardVisionCenteringTask.NO_CENTER_THRESHOLD;
    }

    protected PIDHandler createTurnHandler()
    {
        return new PIDHandler(
            TuningConstants.VISION_STATIONARY_CENTERING_PID_KP,
            TuningConstants.VISION_STATIONARY_CENTERING_PID_KI,
            TuningConstants.VISION_STATIONARY_CENTERING_PID_KD,
            TuningConstants.VISION_STATIONARY_CENTERING_PID_KF,
            TuningConstants.VISION_STATIONARY_CENTERING_PID_KS,
            TuningConstants.VISION_STATIONARY_CENTERING_PID_MIN,
            TuningConstants.VISION_STATIONARY_CENTERING_PID_MAX,
            this.getInjector().getInstance(ITimer.class));
    }
}
