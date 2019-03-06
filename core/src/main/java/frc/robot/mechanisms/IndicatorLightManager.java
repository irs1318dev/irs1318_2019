package frc.robot.mechanisms;

import frc.robot.*;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.Operation;
import frc.robot.driver.common.Driver;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Indicator Light manager
 * 
 * This class manages indicator lights on the robot.
 * 
 */
@Singleton
public class IndicatorLightManager implements IMechanism
{
    private static final double FlashingFrequency = 0.2;
    private static final double FlashingComparisonFrequency = IndicatorLightManager.FlashingFrequency / 2.0;

    private final GrabberMechanism grabberMechanism;
    private final VisionManager visionManager;
    private final ITimer timer;

    private final IRelay hatchIndicator;
    private final IRelay cargoIndicator;

    private Driver driver;

    private LightMode hatchMode;
    private LightMode cargoMode;

    /**
     * Initializes a new IndicatorLightManager
     * @param provider for obtaining electronics objects
     * @param timer to use
     * @param grabberMechanism for grabber reference
     * @param visionManager for vision reference
     */
    @Inject
    public IndicatorLightManager(
        IRobotProvider provider,
        ITimer timer,
        GrabberMechanism grabberMechanism,
        VisionManager visionManager)
    {
        this.grabberMechanism = grabberMechanism;
        this.visionManager = visionManager;
        this.timer = timer;

        this.hatchIndicator = provider.getRelay(ElectronicsConstants.INDICATOR_HATCH_RELAY_CHANNEL);
        this.cargoIndicator = provider.getRelay(ElectronicsConstants.INDICATOR_CARGO_RELAY_CHANNEL);

        this.hatchMode = LightMode.Off;
        this.cargoMode = LightMode.Off;
    }

    /**
     * set the driver that the mechanism should use
     * @param driver to use
     */
    @Override
    public void setDriver(Driver driver)
    {
        this.driver = driver;
    }

    @Override
    public void readSensors()
    {
    }

    /**
     * calculate the various outputs to use based on the inputs and apply them to the outputs for the relevant mechanism
     */
    @Override
    public void update()
    {
        if (this.grabberMechanism.isHatchMode() &&
            this.visionManager != null &&
            (this.driver.getDigital(Operation.VisionEnableCargoShip) || this.driver.getDigital(Operation.VisionEnableRocket)) &&
            this.visionManager.getCenter() != null)
        {
            if (this.visionManager.getMeasuredDistance() > TuningConstants.INDICATOR_LIGHT_VISION_CONSIDERATION_DISTANCE_RANGE)
            {
                this.hatchMode = LightMode.Off;
            }
            else if (this.visionManager.getMeasuredDistance() <= TuningConstants.INDICATOR_LIGHT_VISION_ACCEPTABLE_DISTANCE_RANGE &&
                Math.abs(this.visionManager.getMeasuredAngle() - this.visionManager.getDesiredAngle()) < TuningConstants.INDICATOR_LIGHT_VISION_ACCEPTABLE_ANGLE_RANGE)
            {
                this.hatchMode = LightMode.Flashing;
            }
            else
            {
                this.hatchMode = LightMode.On;
            }
        }
        else
        {
            this.hatchMode = LightMode.Off;
        }

        /*if (this.grabberMechanism.isCargoMode())
        {
            if (this.visionManager != null &&
                (this.driver.getDigital(Operation.VisionEnableCargoShip) || this.driver.getDigital(Operation.VisionEnableRocket)) &&
                this.visionManager.getCenter() == null)
            {
                this.hatchMode = LightMode.Flashing;
            }
            else
            {
                this.hatchMode = LightMode.On;
            }
        }
        else
        {
            this.cargoMode = LightMode.Off;
        }*/

        this.controlLight(this.hatchIndicator, this.hatchMode);
        this.controlLight(this.cargoIndicator, this.cargoMode);
    }

    /**
     * stop the relevant component
     */
    @Override
    public void stop()
    {
        this.hatchIndicator.set(RelayValue.Off);
        this.cargoIndicator.set(RelayValue.Off);
    }

    private void controlLight(IRelay indicatorLight, LightMode mode)
    {
        if (mode == LightMode.On)
        {
            indicatorLight.set(RelayValue.Forward);
        }
        else if (mode == LightMode.Off)
        {
            indicatorLight.set(RelayValue.Off);
        }
        else
        {
            double currentTime = this.timer.get();
            if (currentTime % IndicatorLightManager.FlashingFrequency >= IndicatorLightManager.FlashingComparisonFrequency)
            {
                indicatorLight.set(RelayValue.Forward);
            }
            else
            {
                indicatorLight.set(RelayValue.Off);
            }
        }
    }

    private enum LightMode
    {
        Off,
        Flashing,
        On,
    }
}
