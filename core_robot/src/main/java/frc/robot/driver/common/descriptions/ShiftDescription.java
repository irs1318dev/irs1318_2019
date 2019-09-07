package frc.robot.driver.common.descriptions;

import frc.robot.TuningConstants;
import frc.robot.common.robotprovider.IJoystick;
import frc.robot.driver.common.UserInputDeviceButton;

public class ShiftDescription
{
    private final UserInputDevice userInputDevice;
    private final UserInputDeviceButton userInputDeviceButton;
    private final int userInputDevicePovValue;

    public ShiftDescription(UserInputDevice userInputDevice, UserInputDeviceButton userInputDeviceButton)
    {
        this.userInputDevice = userInputDevice;
        this.userInputDeviceButton = userInputDeviceButton;
        this.userInputDevicePovValue = -1;
    }

    public ShiftDescription(UserInputDevice userInputDevice, int povValue)
    {
        this.userInputDevice = userInputDevice;
        this.userInputDeviceButton = UserInputDeviceButton.POV;
        this.userInputDevicePovValue = povValue;
    }

    public UserInputDevice getUserInputDevice()
    {
        return this.userInputDevice;
    }

    public UserInputDeviceButton getUserInputDeviceButton()
    {
        return this.userInputDeviceButton;
    }

    public int getUserInputDevicePovValue()
    {
        return this.userInputDevicePovValue;
    }

    public boolean checkInput(IJoystick driver, IJoystick coDriver)
    {
        IJoystick relevantJoystick;
        UserInputDeviceButton relevantButton;
        switch (this.getUserInputDevice())
        {
            case None:
                return false;

            case Driver:
                relevantJoystick = driver;
                break;

            case CoDriver:
                relevantJoystick = coDriver;
                break;

            case Sensor:
                relevantJoystick = null;

            default:
                if (TuningConstants.THROW_EXCEPTIONS)
                {
                    throw new RuntimeException("unexpected user input device " + this.getUserInputDevice().toString());
                }

                return false;
        }

        if (relevantJoystick != null)
        {
            // find the appropriate button and grab the value from the relevant joystick
            relevantButton = this.getUserInputDeviceButton();

            if (relevantButton == UserInputDeviceButton.POV)
            {
                return relevantJoystick.getPOV() == this.getUserInputDevicePovValue();
            }
            else if (relevantButton != UserInputDeviceButton.NONE)
            {
                return relevantJoystick.getRawButton(relevantButton.Value);
            }
            else
            {
                return false;
            }
        }
        else
        {
            // grab the appropriate sensor output.
            // e.g.: if (description.getSensor() == DigitalSensor.None) ...
            return false;
        }
    }
}
