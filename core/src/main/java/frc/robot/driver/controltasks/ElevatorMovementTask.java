package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.Operation;
import frc.robot.mechanisms.ElevatorMechanism;

public class ElevatorMovementTask extends CompositeOperationTask {

    private static final Operation[] possibleOperations = 
        {
            Operation.ElevatorBottomPosition,
            Operation.ElevatorHatch2Position,
            Operation.ElevatorHatch3Position,
            Operation.ElevatorCargo1Position,
            Operation.ElevatorCargo2Position,
            Operation.ElevatorCargo3Position,
            Operation.ElevatorCargoLoadPosition,
        };

    private ElevatorMechanism elevator;
    private boolean completeWithTime;
    
    public ElevatorMovementTask(Operation toPerform)
    {
        super(TuningConstants.ELEVATOR_CLIMBING_MOVEMENT_TIME_THRESHOLD, toPerform, ElevatorMovementTask.possibleOperations);
        this.completeWithTime = false;
    }

    public ElevatorMovementTask(double duration, Operation toPerform)
    {
        super(duration, toPerform, ElevatorMovementTask.possibleOperations);
        this.completeWithTime = true;
    }

    @Override
    public void begin()
    {
        super.begin();
        this.elevator = this.getInjector().getInstance(ElevatorMechanism.class);
    }

    @Override
    public boolean hasCompleted()
    {
        if (this.completeWithTime)
        {
            return super.hasCompleted();
        }

        if(super.hasCompleted())
        {
            return true;
        }

        double heightError = Math.abs(elevator.getHeight()-elevator.getDesiredHeight());
        if (heightError < TuningConstants.ELEVATOR_CLIMBING_HEIGHT_ERROR_THRESHOLD)
        {
            return true;
        }

        return false;
    }



    
}
