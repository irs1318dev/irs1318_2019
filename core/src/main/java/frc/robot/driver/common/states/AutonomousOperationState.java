package frc.robot.driver.common.states;

import java.util.Map;
import java.util.Set;

import frc.robot.common.robotprovider.IJoystick;
import frc.robot.driver.Operation;
import frc.robot.driver.Shift;
import frc.robot.driver.common.IControlTask;
import frc.robot.driver.common.descriptions.AnalogOperationDescription;
import frc.robot.driver.common.descriptions.MacroOperationDescription;

import com.google.inject.Injector;

/**
 * The state of the current autonomous operation.
 *
 */
public class AutonomousOperationState extends OperationState implements IMacroOperationState
{
    private final Map<Operation, OperationState> operationStateMap;

    private IControlTask task;

    private boolean hasBegun;
    private boolean hasEnded;

    public AutonomousOperationState(
        IControlTask task,
        Map<Operation, OperationState> operationStateMap)
    {
        super(null);

        this.operationStateMap = operationStateMap;
        this.task = task;

        this.hasBegun = false;
        this.hasEnded = false;
    }

    /**
     * Sets whether the current operation is being interrupted by a macro
     * @param enable value of true indicates that we are interrupted
     */
    @Override
    public void setIsInterrupted(boolean enable)
    {
    }

    /**
     * Gets whether the current operation is being interrupted by a macro
     * @return value of true indicates that we are interrupted
     */
    @Override
    public boolean getIsInterrupted()
    {
        return false;
    }

    /**
     * Checks whether the operation state should change based on the driver and co-driver joysticks and component sensors. 
     * @param driver joystick to update from
     * @param coDriver joystick to update from
     * @param activeShifts to update from
     * @return true if there was any active user input that triggered a state change
     */
    @Override
    public boolean checkInput(IJoystick driver, IJoystick coDriver, Set<Shift> activeShifts)
    {
        return false;
    }

    public Operation[] getAffectedOperations()
    {
        Set<Operation> keys = this.operationStateMap.keySet();
        Operation[] keyArray = new Operation[keys.size()];
        return (Operation[])keys.toArray(keyArray);
    }

    public boolean getIsActive()
    {
        return this.hasBegun && !this.hasEnded;
    }

    public void run()
    {
        if (!this.hasEnded)
        {
            if (!this.hasBegun)
            {
                // if we haven't begun, begin
                this.task.begin();
                this.hasBegun = true;
            }

            if (this.task.hasCompleted())
            {
                // if we shouldn't continue, end the task
                this.task.end();
                this.hasEnded = true;
            }
            else if (this.task.shouldCancel())
            {
                this.task.stop();
                this.hasEnded = true;
            }
            else
            {
                // run the current task and apply the result to the state
                this.task.update();
            }
        }
    }

    public void cancel()
    {
        this.task.stop();
        this.task = null;
        this.hasEnded = true;
    }
}
