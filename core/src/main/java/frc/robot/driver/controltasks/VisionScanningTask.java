package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.common.PIDHandler;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.driver.Operation;
import frc.robot.driver.common.IControlTask;
import frc.robot.mechanisms.OffboardVisionManager;
import frc.robot.mechanisms.PositionManager;
import frc.robot.mechanisms.VisionManager;

/**
 * Task that turns the robot a certain amount clockwise or counterclockwise in-place based on vision center
 */
public class VisionScanningTask extends ControlTaskBase implements IControlTask
{
    private static final int NO_CENTER_THRESHOLD = 40;

    private PIDHandler turnPidHandler;
    private Double centeredTime;
    protected OffboardVisionManager offboardVisionManager;
    private PositionManager positionManager;

    private int noCenterCount;
    private double startingAngle;
    private boolean shouldLookLeft;

    protected ITimer timer;
    protected Double startTime;

    /**
    * Initializes a new VisionCenteringTask
    */
    public VisionScanningTask()
    {
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
        this.turnPidHandler = this.createTurnHandler();

        this.positionManager = this.getInjector().getInstance(PositionManager.class);
        this.offboardVisionManager = this.getInjector().getInstance(OffboardVisionManager.class);
        this.startingAngle = this.positionManager.getNavxAngle();
        
        this.timer = this.getInjector().getInstance(ITimer.class);

        this.startTime = this.timer.get();

    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        this.setDigitalOperationState(Operation.VisionEnableOffboardProcessing, true);

        double currentMeasuredAngle = this.positionManager.getNavxAngle();

        double currentDesiredAngle;
        if(shouldLookLeft) {
            currentDesiredAngle = this.startingAngle - 30;
        }
        else {
            currentDesiredAngle = this.startingAngle + 30;
        }

        this.setAnalogOperationState(
            Operation.DriveTrainTurn,
            -this.turnPidHandler.calculatePosition(currentDesiredAngle, currentMeasuredAngle));

        if (Math.abs(currentMeasuredAngle - currentDesiredAngle) < 1.0)
            shouldLookLeft = !shouldLookLeft;

    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        this.setAnalogOperationState(Operation.DriveTrainTurn, 0.0);

        this.setDigitalOperationState(Operation.VisionEnableOffboardProcessing, false);
        this.setDigitalOperationState(Operation.VisionDisable, true);
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        Double currentMeasuredAngle = this.positionManager.getNavxAngle();
        Double currentDesiredAngle = this.positionManager.getNavxAngle();
        if (currentMeasuredAngle == null || currentDesiredAngle == null)
        {
            return false;
        }

        if (this.offboardVisionManager.getBallSeen()) {
            return true;
        }

        return this.timer.get() >= this.startTime + 30.0;


        /*
        double centerAngleDifference = Math.abs(currentMeasuredAngle - currentDesiredAngle);
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
        }*/
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
