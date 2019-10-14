package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.ElevatorMechanism;

public class ElevatorPositionTask extends CompositeOperationTask {

    private static final DigitalOperation[] possibleOperations = 
        {
            DigitalOperation.ElevatorBottomPosition,
            DigitalOperation.ElevatorHatch2Position,
            DigitalOperation.ElevatorHatch3Position,
            DigitalOperation.ElevatorCargo1Position,
            DigitalOperation.ElevatorCargo2Position,
            DigitalOperation.ElevatorCargo3Position,
            DigitalOperation.ElevatorCargoLoadPosition,
            DigitalOperation.ElevatorCamReturnPosition,
        };

    private final boolean completeWithTime;

    private ElevatorMechanism elevator;
    private boolean firstLoop;

    public ElevatorPositionTask(DigitalOperation toPerform)
    {
        super(TuningConstants.ELEVATOR_CLIMBING_MOVEMENT_TIME_THRESHOLD, toPerform, ElevatorPositionTask.possibleOperations);
        this.completeWithTime = false;
    }

    public ElevatorPositionTask(double duration, DigitalOperation toPerform)
    {
        super(duration, toPerform, ElevatorPositionTask.possibleOperations);
        this.completeWithTime = true;
    }

    @Override
    public void begin()
    {
        super.begin();

        this.elevator = this.getInjector().getInstance(ElevatorMechanism.class);
        this.firstLoop = true;
    }

    @Override
    public void update()
    {
        super.update();

        this.firstLoop = false;
    }

    @Override
    public boolean hasCompleted()
    {
        if (this.completeWithTime)
        {
            return super.hasCompleted();
        }

        if (super.hasCompleted())
        {
            return true;
        }

        double heightError = Math.abs(this.elevator.getHeight() - this.elevator.getDesiredHeight());
        if (!this.firstLoop && heightError < TuningConstants.ELEVATOR_CLIMBING_HEIGHT_ERROR_THRESHOLD)
        {
            return true;
        }

        return false;
    }    
}
