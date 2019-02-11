package frc.robot.mechanisms;

import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.common.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Offboard Vision manager.
 * 
 * @author Will
 *
 */
@Singleton
public class OffboardVisionManager implements IMechanism
{
    private final INetworkTableProvider networkTable;

    private double ballCenterX;
    private double ballCenterY;
    private double ballDistance;
    private String ballDirection;

    /**
     * Initializes a new OffboardVisionManager
     * @param provider for obtaining electronics objects
     */
    @Inject
    public OffboardVisionManager(IRobotProvider provider)
    {
        this.networkTable = provider.getNetworkTableProvider();

        this.ballCenterX = 0.0;
        this.ballCenterY = 0.0;
        this.ballDistance = 0.0;
        this.ballDirection = "";
    }

    public double getBallCenterX()
    {
        return this.ballCenterX;
    }

    public double getBallCenterY()
    {
        return this.ballCenterY;
    }

    public double getBallDistance()
    {
        return this.ballDistance;
    }

    public String getBallDirection()
    {
        return this.ballDirection;
    }

    /**
     * read all of the sensors for the mechanism that we will use in macros/autonomous mode and record their values
     */
    @Override
    public void readSensors()
    {
        this.ballCenterX = this.networkTable.getSmartDashboardNumber("rpi.BallcenterX");
        this.ballCenterY = this.networkTable.getSmartDashboardNumber("rpi.BallcenterY");
        this.ballDistance = this.networkTable.getSmartDashboardNumber("rpi.BallDistance");
        this.ballDirection = this.networkTable.getSmartDashboardString("rpi.BallDirection");
    }

    @Override
    public void update()
    {
    }

    @Override
    public void stop()
    {
    }

    @Override
    public void setDriver(Driver driver)
    {
    }
}
