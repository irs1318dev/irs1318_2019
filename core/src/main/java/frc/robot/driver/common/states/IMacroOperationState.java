package frc.robot.driver.common.states;

import java.util.Map;
import java.util.Set;

import frc.robot.TuningConstants;
import frc.robot.common.robotprovider.IJoystick;
import frc.robot.driver.Operation;
import frc.robot.driver.Shift;
import frc.robot.driver.common.IControlTask;
import frc.robot.driver.common.UserInputDeviceButton;
import frc.robot.driver.common.buttons.ClickButton;
import frc.robot.driver.common.buttons.IButton;
import frc.robot.driver.common.buttons.SimpleButton;
import frc.robot.driver.common.buttons.ToggleButton;
import frc.robot.driver.common.descriptions.MacroOperationDescription;

import com.google.inject.Injector;

/**
 * The state of the current macro operation.
 *
 */
public interface IMacroOperationState extends IOperationState
{
    public Operation[] getAffectedOperations();

    public boolean getIsActive();

    public void run();

    public void cancel();
}
