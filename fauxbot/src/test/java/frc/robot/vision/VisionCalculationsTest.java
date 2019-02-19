package frc.robot.vision;

import frc.robot.GamePiece;
import frc.robot.common.robotprovider.IRotatedRect;
import frc.robot.common.robotprovider.RotatedRectWrapper;
import frc.robot.vision.common.VisionProcessingState;
import frc.robot.vision.common.VisionResult;
import org.junit.jupiter.api.Test;
import org.opencv.core.RotatedRect;

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
    /*
C:\Users\james\IdeaProjects2018\irs1318_2019\fauxbot\src\test\resources\frc.robot.vision.pipelines\Capture.PNG
R_0 : [227.61538696289062, 67.0769271850586, 11.374734878540039, 28.240720748901367, -11.309932708740234]
R_1 : [169.058837890625, 63.76471710205078, 27.649059295654297, 10.186494827270508, -75.96376037597656]
S_0_R_0 : [227.61538696289062, 67.0769271850586, 11.374734878540039, 28.240720748901367, -11.309932708740234]
S_0_R_1 : [169.058837890625, 63.76471710205078, 27.649059295654297, 10.186494827270508, -75.96376037597656]
     */
    public static final List<IRotatedRect> CAPTURE_RECT_LIST =
            new ArrayList() {{
                add(new RotatedRectWrapper(
                        new RotatedRect(
                                new double[]{227.61538696289062, 67.0769271850586, 11.374734878540039, 28.240720748901367, -11.309932708740234})));
                add(new RotatedRectWrapper(
                        new RotatedRect(
                                new double[]{169.058837890625, 63.76471710205078, 27.649059295654297, 10.186494827270508, -75.96376037597656})));
            }};
    @Test
    public void testInterpolate_happyCase() {

        VisionCalculations calc = new VisionCalculations();

        // 35 pixels puts the camera 32.2 inches from the target.
        int interval = calc.findInterval(35.0, SAMPLE_PIXELS_TO_INCHES);
        assertEquals(1, interval);
        double inches = calc.interpolateInchesFromPixels(35, interval, SAMPLE_PIXELS_TO_INCHES);
        assertEquals(32.2, inches, EPS);

    }

    @Test
    public void testDetermineVisionResult_happyCase_highTarget()
    {
        VisionCalculations calc = new VisionCalculations();
        assertEquals(VisionResult.HIGH_TARGET,
                calc.determineVisionResult(GamePiece.Cargo,
                        VisionProcessingState.ActiveRocket));

        assertEquals(VisionResult.LOW_TARGET,
                calc.determineVisionResult(GamePiece.HatchPanel,
                        VisionProcessingState.ActiveRocket));

        assertEquals(VisionResult.LOW_TARGET,
                calc.determineVisionResult(GamePiece.HatchPanel,
                        VisionProcessingState.ActiveCargoShip));
    }

    @Test
    public void testComputeAvgPixels_happyCase()
    {
//        Expected :0.0
//        Actual   :10.780614852905273

        VisionCalculations calc = new VisionCalculations();
        assertEquals(10.8 ,calc.computeAvgPixel(CAPTURE_RECT_LIST), EPS);
    }


}
