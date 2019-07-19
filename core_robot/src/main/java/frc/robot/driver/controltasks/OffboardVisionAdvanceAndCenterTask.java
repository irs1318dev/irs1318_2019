package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.common.PIDHandler;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.driver.Operation;
import frc.robot.driver.common.IControlTask;

public class OffboardVisionAdvanceAndCenterTask extends OffboardVisionCenteringTask implements IControlTask
{
    private PIDHandler forwardPIDHandler;

    /**
    * Initializes a new VisionForwardAndCenterTask
    */
    public OffboardVisionAdvanceAndCenterTask()
    {
        super(false);

        this.forwardPIDHandler = null;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        super.begin();
        this.forwardPIDHandler = new PIDHandler(
            TuningConstants.VISION_ADVANCING_PID_KP,
            TuningConstants.VISION_ADVANCING_PID_KI,
            TuningConstants.VISION_ADVANCING_PID_KD,
            TuningConstants.VISION_ADVANCING_PID_KF,
            TuningConstants.VISION_ADVANCING_PID_KS,
            TuningConstants.VISION_ADVANCING_PID_MIN,
            TuningConstants.VISION_ADVANCING_PID_MAX,
            this.getInjector().getInstance(ITimer.class));
    }

    @Override
    public void update()
    {
        super.update();
        double currentDistance = this.offboardVisionManager.getBallDistance();
       
        this.setAnalogOperationState(Operation.DriveTrainMoveForward, this.forwardPIDHandler.calculatePosition(0.0, -currentDistance));
    }

    @Override
    public void end()
    {
        super.end();
        this.setAnalogOperationState(Operation.DriveTrainMoveForward, 0.0);
    }

    @Override
    public boolean hasCompleted()
    {
        double currentDistance = this.offboardVisionManager.getBallDistance();
        return currentDistance <= TuningConstants.MAX_VISION_ACCEPTABLE_FORWARD_DISTANCE;
    }

    @Override
    protected PIDHandler createTurnHandler()
    {
        return new PIDHandler(
            TuningConstants.VISION_MOVING_CENTERING_PID_KP,
            TuningConstants.VISION_MOVING_CENTERING_PID_KI,
            TuningConstants.VISION_MOVING_CENTERING_PID_KD,
            TuningConstants.VISION_MOVING_CENTERING_PID_KF,
            TuningConstants.VISION_MOVING_CENTERING_PID_KS,
            TuningConstants.VISION_MOVING_CENTERING_PID_MIN,
            TuningConstants.VISION_MOVING_CENTERING_PID_MAX,
            this.getInjector().getInstance(ITimer.class));
    }
}
