package frc.robot.vision;

import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class VisionCalculationsTest {
    public static final double EPS = 0.1;

    public static final List<PixelsToInches> SAMPLE_PIXELS_TO_INCHES = new ArrayList() {{
        add(new PixelsToInches(60, 0));
        add(new PixelsToInches(40, 24.0));
        add(new PixelsToInches(29, 42.0));
        add(new PixelsToInches(10, 100));
    }};

    //@Test
    public void testInterpolate_happyCase() {

        VisionCalculations calc = new VisionCalculations();

        // 35 pixels puts the camera 32.2 inches from the target.
        int interval = calc.findInterval(35.0, SAMPLE_PIXELS_TO_INCHES);
        assertEquals(1, interval);
        double inches = calc.interpolateInchesFromPixels(35, interval, SAMPLE_PIXELS_TO_INCHES);
        assertEquals(32.2, inches, EPS);

    }


}
