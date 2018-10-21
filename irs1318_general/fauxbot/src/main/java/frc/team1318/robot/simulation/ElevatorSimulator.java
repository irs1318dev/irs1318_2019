package frc.team1318.robot.simulation;

import java.io.FileInputStream;
import java.util.HashMap;
import java.util.Map;

import frc.team1318.robot.IRealWorldSimulator;
import frc.team1318.robot.common.robotprovider.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;

@Singleton
public class ElevatorSimulator implements IRealWorldSimulator
{
    private static final int EncoderAChannel = 0;
    private static final int EncoderBChannel = 1;
    private static final int MotorChannel = 0;

    private FileInputStream elevatorPersonInputStream;
    private Image ElevatorPerson;

    @SuppressWarnings("serial")
    private final Map<Integer, String> sensorNameMap = new HashMap<Integer, String>()
    {
        {
            this.put(ElevatorSimulator.EncoderAChannel, "Elevator encoder");
            this.put(ElevatorSimulator.EncoderBChannel, "Elevator encoder");
        }
    };

    @SuppressWarnings("serial")
    private final Map<Integer, String> motorNameMap = new HashMap<Integer, String>()
    {
        {
            this.put(ElevatorSimulator.MotorChannel, "Elevator motor");
        }
    };

    private static final double ElevatorMinHeight = 0.0;
    private static final double ElevatorMaxHeight = 250.0;

    private static final double[] FloorHeightPercentages = new double[] { 0.0, 0.2, 0.4, 0.6, 0.8 };
    private static final double PercentageAllowance = 0.01;
    private static final double ElevatorCarWidth = 15.0;
    private static final double ElevatorCarHeight = 35.0;

    private double currentElevatorHeight;

    @Inject
    public ElevatorSimulator()
    {
        try
        {
            elevatorPersonInputStream = new FileInputStream(this.getClass().getResource("/images/stickFigure.png").getPath());
        }
        catch (Exception e)
        {
            System.out.println("ERROR: INVALID IMAGE");             
        }
        
        ElevatorPerson = new Image(elevatorPersonInputStream);
        this.currentElevatorHeight = 0.0;
    }

    public String getSensorName(int channel)
    {
        if (this.sensorNameMap.containsKey(channel))
        {
            return this.sensorNameMap.get(channel);
        }

        return "Sensor " + channel;
    }

    public double getEncoderMin(int channel)
    {
        return ElevatorSimulator.ElevatorMinHeight;
    }

    public double getEncoderMax(int channel)
    {
        return ElevatorSimulator.ElevatorMaxHeight;
    }

    public String getActuatorName(int channel)
    {
        if (this.motorNameMap.containsKey(channel))
        {
            return this.motorNameMap.get(channel);
        }

        return "Motor " + channel;
    }

    public void update()
    {
        FauxbotActuatorBase actuator = FauxbotActuatorManager.get(ElevatorSimulator.MotorChannel);
        if (actuator != null && actuator instanceof FauxbotMotorBase)
        {
            FauxbotMotorBase motor = (FauxbotMotorBase)actuator;
            double motorPower = motor.get();
            this.currentElevatorHeight += motorPower;
        }

        if (this.currentElevatorHeight > ElevatorSimulator.ElevatorMaxHeight)
        {
            this.currentElevatorHeight = ElevatorSimulator.ElevatorMaxHeight;
        }
        else if (this.currentElevatorHeight < ElevatorSimulator.ElevatorMinHeight)
        {
            this.currentElevatorHeight = ElevatorSimulator.ElevatorMinHeight;
        }

        FauxbotSensorBase sensor = FauxbotSensorManager.get(ElevatorSimulator.EncoderAChannel);
        if (sensor != null && sensor instanceof FauxbotEncoder)
        {
            FauxbotEncoder encoder = (FauxbotEncoder)sensor;
            encoder.set(this.currentElevatorHeight);
        }
    }

    /**
     * Draw a frame of animation based on the current state of the simulation.
     * Remember that (0, 0) is at the top left!
     */
    @Override
    public void draw(Canvas canvas)
    {
        double elevatorHeightRatio = this.currentElevatorHeight / (ElevatorSimulator.ElevatorMaxHeight - ElevatorSimulator.ElevatorMinHeight);

        double canvasHeight = canvas.getHeight();
        double canvasWidth = canvas.getWidth();
        GraphicsContext gc = canvas.getGraphicsContext2D();
        gc.clearRect(0.0, 0.0, canvasWidth, canvasHeight);

        // cycle through the floors:
        Color elevatorCarColor = Color.RED;
        for (double ratio : ElevatorSimulator.FloorHeightPercentages)
        {
            // if we are within a small allowance from the floor, change elevator color to Green
            if (Math.abs(elevatorHeightRatio - ratio) < ElevatorSimulator.PercentageAllowance)
            {
                elevatorCarColor = Color.GREEN;
            }

            // draw the floor:
            gc.setStroke(Color.BLACK);
            gc.setLineWidth(1.0);
            gc.strokeLine(ElevatorSimulator.ElevatorCarWidth, (1 - ratio) * canvasHeight, canvasWidth, (1 - ratio) * canvasHeight); 
        }

        // draw the elevator car:
        gc.setStroke(elevatorCarColor);
        gc.setLineWidth(1.0);
        gc.strokeRect(
            0.0,
            (1.0 - elevatorHeightRatio) * canvasHeight - ElevatorSimulator.ElevatorCarHeight,
            ElevatorSimulator.ElevatorCarWidth,
            ElevatorSimulator.ElevatorCarHeight);

        // draw the elevator rider:
        gc.drawImage(ElevatorPerson, 0.0, (1.0 - elevatorHeightRatio) * canvasHeight - ElevatorSimulator.ElevatorCarHeight, 
                        ElevatorSimulator.ElevatorCarWidth,  ElevatorSimulator.ElevatorCarHeight);
    }
}
