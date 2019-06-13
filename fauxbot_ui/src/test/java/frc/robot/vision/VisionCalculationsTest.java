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
    public static final List<IRotatedRect> CAPTURE_0DEG_RECT_PAIR_LIST =
            new ArrayList() {{
                add(new RotatedRectWrapper(
                        new RotatedRect(
                                new double[]{169.058837890625, 63.76471710205078, 27.649059295654297, 10.186494827270508, -75.96376037597656})));
                add(new RotatedRectWrapper(
                        new RotatedRect(
                                new double[]{227.61538696289062, 67.0769271850586, 11.374734878540039, 28.240720748901367, -11.309932708740234})));
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

    //@Test
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

    //@Test
    public void testComputeAvgPixels_happyCase()
    {
//        Expected :0.0
//        Actual  : 27.944890022277832

        VisionCalculations calc = new VisionCalculations();
        assertEquals(27.94 ,calc.computeAvgPixel(CAPTURE_0DEG_RECT_PAIR_LIST), EPS);
    }

    //@Test
    public void testAzimuth_0degress()
    {
//        Expected :0.0
//        Actual   :10.780614852905273

        // o= observedPixelsBetween Centers, d=expectedDistanceBetweenCenters
        //o=58.556549, d=58.430225
        // double avgPixel = 27.944890022277832;
        VisionCalculations calc = new VisionCalculations();
        assertEquals(0.0 ,calc.azimuth(
                calc.computeAvgPixel(CAPTURE_0DEG_RECT_PAIR_LIST),
                CAPTURE_0DEG_RECT_PAIR_LIST), EPS);
    }

    /*
C:\Users\DriverStationB\git\\irs1318_2019\fauxbot\src\test\resources\frc.robot.vision.pipelines\Capture2.PNG
R_0 : [165.84146118164062, 67.07316589355469, 29.683290481567383, 9.358966827392578, -77.90524291992188]
R_1 : [213.24998474121094, 62.74999237060547, 8.7681245803833, 26.728639602661133, -8.13010311126709]
S_0_R_0 : [213.24998474121094, 62.74999237060547, 8.7681245803833, 26.728639602661133, -8.13010311126709]
S_0_R_1 : [165.84146118164062, 67.07316589355469, 29.683290481567383, 9.358966827392578, -77.90524291992188]
o=47.408524, d=58.976109
     */
    public static final List<IRotatedRect> CAPTURE2_60DEG_RECT_PAIR_LIST =
            new ArrayList() {{
                add(new RotatedRectWrapper(
                        new RotatedRect(
                                new double[]{165.84146118164062, 67.07316589355469, 29.683290481567383, 9.358966827392578, -77.90524291992188})));
                add(new RotatedRectWrapper(
                        new RotatedRect(
                                new double[]{213.24998474121094, 62.74999237060547, 8.7681245803833, 26.728639602661133, -8.13010311126709})));
            }};

    //@Test
    public void testAzimuth_happyCase()
    {
        //o=47.408524, d=58.976109
        //a=0.637040, adeg=36.499716

        // o= observedPixelsBetween Centers, d=expectedDistanceBetweenCenters
        //o=58.556549, d=58.430225
        // double avgPixel = 27.944890022277832;
        VisionCalculations calc = new VisionCalculations();
        double azimuth = calc.azimuth(
                calc.computeAvgPixel(CAPTURE2_60DEG_RECT_PAIR_LIST),
                CAPTURE2_60DEG_RECT_PAIR_LIST);
        assertEquals(0.637 ,azimuth, EPS);
    }

/*
C:\Users\DriverStationB\git\\irs1318_2019\fauxbot\src\test\resources\frc.robot.vision.pipelines\Capture6.PNG
R_0 : [256.54998779296875, 106.85000610351562, 45.53679656982422, 14.862703323364258, -71.56505584716797]
R_1 : [195.69793701171875, 38.526268005371094, 13.081071853637695, 48.64252853393555, -4.969740390777588]
R_2 : [136.5460968017578, 30.923683166503906, 39.63337707519531, 9.60930061340332, -85.42607879638672]

*/
public static final List<IRotatedRect> CAPTURE6_RECT_PAIR_LIST =
        new ArrayList() {{
            add(new RotatedRectWrapper(
                    new RotatedRect(
                            new double[]{136.5460968017578, 30.923683166503906, 39.63337707519531, 9.60930061340332, -85.42607879638672})));
            add(new RotatedRectWrapper(
                    new RotatedRect(
                            new double[]{195.69793701171875, 38.526268005371094, 13.081071853637695, 48.64252853393555, -4.969740390777588})));
        }};

    //@Test
    public void testAzimuthCapture6_happyCase()
    {

        //o=59.151840, d=92.288447
        //a=0.875067, adeg=-50.137661

        VisionCalculations calc = new VisionCalculations();
        double azimuth = calc.azimuth(
                calc.computeAvgPixel(CAPTURE6_RECT_PAIR_LIST),
                CAPTURE6_RECT_PAIR_LIST);
        assertEquals(-0.875 ,azimuth, EPS);
    }
    public static final List<IRotatedRect> UNSORTED_RECTANGLES =
            new ArrayList() {{
                add(new RotatedRectWrapper(
                        new RotatedRect(
                                new double[]{310.0, 30.923683166503906, 39.63337707519531, 9.60930061340332, -85.42607879638672})));
                add(new RotatedRectWrapper(
                        new RotatedRect(
                                new double[]{195.0, 38.526268005371094, 13.081071853637695, 48.64252853393555, -4.969740390777588})));
                add(new RotatedRectWrapper(
                        new RotatedRect(
                                new double[]{220.0, 30.923683166503906, 39.63337707519531, 9.60930061340332, -85.42607879638672})));
                add(new RotatedRectWrapper(
                        new RotatedRect(
                                new double[]{136.0, 30.923683166503906, 39.63337707519531, 9.60930061340332, -85.42607879638672})));
            }};

    //@Test
    public void testSortByCenterX() {
        VisionCalculations calc = new VisionCalculations();

        List<IRotatedRect> sortedList = calc.sortByCenterX(UNSORTED_RECTANGLES);
        assertEquals(4, sortedList.size());
        assertEquals(136.0, sortedList.get(0).getCenter().getX(), EPS);
        assertEquals(195.0, sortedList.get(1).getCenter().getX(), EPS);
        assertEquals(220.0, sortedList.get(2).getCenter().getX(), EPS);
        assertEquals(310.0, sortedList.get(3).getCenter().getX(), EPS);
    }


}
