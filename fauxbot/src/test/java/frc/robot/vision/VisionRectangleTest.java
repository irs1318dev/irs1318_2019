package frc.robot.vision;

import frc.robot.common.robotprovider.IRotatedRect;
import frc.robot.common.robotprovider.RotatedRectWrapper;
import frc.robot.vision.common.VisionResult;
import org.junit.jupiter.api.Test;
import org.opencv.core.RotatedRect;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class VisionRectangleTest {
    /*
C:\Users\james\IdeaProjects2018\irs1318_2019\fauxbot\src\test\resources\frc.robot.vision.pipelines\Capture4.PNG
R_0 : [150.5, 68.0, 1.0, 6.0, -0.0]
R_1 : [148.76470947265625, 56.558834075927734, 1.212678074836731, 11.156637191772461, -14.03624439239502]
R_2 : [180.94119262695312, 18.73529815673828, 31.529630661010742, 10.429031372070312, -75.96376037597656]
R_3 : [239.90577697753906, 16.463115692138672, 11.135895729064941, 32.41179656982422, -5.194429397583008]
S_0_R_0 : [150.5, 68.0, 1.0, 6.0, -0.0]
S_0_R_1 : [148.76470947265625, 56.558834075927734, 1.212678074836731, 11.156637191772461, -14.03624439239502]
S_0_R_2 : [180.94119262695312, 18.73529815673828, 31.529630661010742, 10.429031372070312, -75.96376037597656]
S_0_R_3 : [239.90577697753906, 16.463115692138672, 11.135895729064941, 32.41179656982422, -5.194429397583008]
S_1_R_0 : [150.5, 68.0, 1.0, 6.0, -0.0]
S_1_R_1 : [148.76470947265625, 56.558834075927734, 1.212678074836731, 11.156637191772461, -14.03624439239502]
S_1_R_2 : [180.94119262695312, 18.73529815673828, 31.529630661010742, 10.429031372070312, -75.96376037597656]
S_1_R_3 : [239.90577697753906, 16.463115692138672, 11.135895729064941, 32.41179656982422, -5.194429397583008]
    */
    /**
     * Capture4 has two strong targets (upper) and two weak targets.
     * The two strong targets show up in the contours.
     * The left weak target does not show up.
     * The right weak target is duplicated by the contours, and therefore the rectangles
     */
    public static final List<IRotatedRect> CAPTURE4_RECT_LIST =
            new ArrayList() {{
                add(new RotatedRectWrapper(
                        new RotatedRect(
                                new double[]{150.5, 68.0, 1.0, 6.0, -0.0})));
                add(new RotatedRectWrapper(
                        new RotatedRect(
                                new double[]{148.76470947265625, 56.558834075927734, 1.212678074836731, 11.156637191772461, -14.03624439239502})));
                add(new RotatedRectWrapper(
                        new RotatedRect(
                                new double[]{180.94119262695312, 18.73529815673828, 31.529630661010742, 10.429031372070312, -75.96376037597656})));
                add(new RotatedRectWrapper(
                        new RotatedRect(
                                new double[]{239.90577697753906, 16.463115692138672, 11.135895729064941, 32.41179656982422, -5.194429397583008})));
            }};

    //@Test
    public void testGroupRect_capture4() {
        VisionCalculations calc = new VisionCalculations();

        // capture 4 has two rows of targets, but one is invalid, so it is rejected.
        List<Set<IRotatedRect>> grouped = calc.groupRotatedRect(CAPTURE4_RECT_LIST);
        assertEquals(1, grouped.size());
        Set<IRotatedRect> upperRow = grouped.get(0);
        assertEquals(2,
                upperRow.stream()
                        .filter(rect ->
                                Math.abs(rect.getCenter().getY() - 20) < 10)
                        .collect(Collectors.toList()).size()
        );
    }


    //@Test
    public void testPickRow_capture4() {
        VisionCalculations calc = new VisionCalculations();
        List<Set<IRotatedRect>> grouped = calc.groupRotatedRect(CAPTURE4_RECT_LIST);
        assertEquals(1, grouped.size());

        Set<IRotatedRect> pickRowRects = calc.pickRow(grouped, VisionResult.HIGH_TARGET);
        assertEquals(2, pickRowRects.size());

        List<IRotatedRect> pairedRects = calc.pickPairedRect(grouped.get(0));
        assertEquals(2, pairedRects.size());

    }
    /*
    C:\Users\james\IdeaProjects2018\irs1318_2019\fauxbot\src\test\resources\frc.robot.vision.pipelines\Capture5.PNG
    R_0 : [219.82196044921875, 46.75457763671875, 62.32841873168945, 22.802085876464844, -78.2317123413086]
    S_0_R_0 : [219.82196044921875, 46.75457763671875, 62.32841873168945, 22.802085876464844, -78.2317123413086]

     */
    public static final List<IRotatedRect> CAPTURE5_RECT_LIST =
            new ArrayList() {{
                add(new RotatedRectWrapper(
                        new RotatedRect(
                                new double[]{219.82196044921875, 46.75457763671875, 62.32841873168945, 22.802085876464844, -78.2317123413086})));
            }};

    //@Test
    public void testPickRow_capture5() {
        VisionCalculations calc = new VisionCalculations();
        List<Set<IRotatedRect>> grouped = calc.groupRotatedRect(CAPTURE5_RECT_LIST);
        assertEquals(1, grouped.size());

        List<IRotatedRect> pairedRects = calc.pickPairedRect(grouped.get(0));
        assertEquals(0, pairedRects.size());

    }

    /*

C:\Users\james\IdeaProjects2018\irs1318_2019\fauxbot\src\test\resources\frc.robot.vision.pipelines\Capture3.PNG
R_0 : [154.91175842285156, 78.6470718383789, 10.6715669631958, 31.28709602355957, -14.03624439239502]
R_1 : [101.09406280517578, 69.05941009521484, 28.159555435180664, 9.950372695922852, -84.2894058227539]
R_2 : [202.20559692382812, 25.14854621887207, 37.081886291503906, 9.630990982055664, -78.11134338378906]
R_3 : [250.0, 19.0, 8.0, 34.0, -0.0]
S_0_R_0 : [154.91175842285156, 78.6470718383789, 10.6715669631958, 31.28709602355957, -14.03624439239502]
S_0_R_1 : [101.09406280517578, 69.05941009521484, 28.159555435180664, 9.950372695922852, -84.2894058227539]
S_1_R_0 : [202.20559692382812, 25.14854621887207, 37.081886291503906, 9.630990982055664, -78.11134338378906]
S_1_R_1 : [250.0, 19.0, 8.0, 34.0, -0.0]

*/
    public static final List<IRotatedRect> CAPTURE3_RECT_LIST =
            new ArrayList() {{
                add(new RotatedRectWrapper(
                        new RotatedRect(
                                new double[]{154.91175842285156, 78.6470718383789, 10.6715669631958, 31.28709602355957, -14.03624439239502})));
                add(new RotatedRectWrapper(
                        new RotatedRect(
                                new double[]{101.09406280517578, 69.05941009521484, 28.159555435180664, 9.950372695922852, -84.2894058227539})));
                add(new RotatedRectWrapper(
                        new RotatedRect(
                                new double[]{202.20559692382812, 25.14854621887207, 37.081886291503906, 9.630990982055664, -78.11134338378906})));
                add(new RotatedRectWrapper(
                        new RotatedRect(
                                new double[]{250.0, 19.0, 8.0, 34.0, -0.0})));
            }};
    //@Test
    public void testPickRow_capture3() {
        VisionCalculations calc = new VisionCalculations();
        List<Set<IRotatedRect>> grouped = calc.groupRotatedRect(CAPTURE3_RECT_LIST);
        assertEquals(2, grouped.size());

        assertTrue(calc.isRight(CAPTURE3_RECT_LIST.get(0)));

        Set<IRotatedRect> pickRowRectsLow = calc.pickRow(grouped, VisionResult.LOW_TARGET);
        assertEquals(2, pickRowRectsLow.size());

        Set<IRotatedRect> pickRowRectsHigh = calc.pickRow(grouped, VisionResult.HIGH_TARGET);
        assertEquals(2, pickRowRectsHigh.size());

        List<IRotatedRect> pairedRects = calc.pickPairedRect(grouped.get(0));
        assertEquals(2, pairedRects.size());

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
                                new double[]{256.54998779296875, 106.85000610351562, 45.53679656982422, 14.862703323364258, -71.56505584716797})));
                add(new RotatedRectWrapper(
                        new RotatedRect(
                                new double[]{195.69793701171875, 38.526268005371094, 13.081071853637695, 48.64252853393555, -4.969740390777588})));
                add(new RotatedRectWrapper(
                        new RotatedRect(
                                new double[]{136.5460968017578, 30.923683166503906, 39.63337707519531, 9.60930061340332, -85.42607879638672})));
            }};
    //@Test
    public void testPickRow_capture6() {
        VisionCalculations calc = new VisionCalculations();
        List<Set<IRotatedRect>> grouped = calc.groupRotatedRect(CAPTURE6_RECT_PAIR_LIST);
        assertEquals(2, grouped.size());

        Set<IRotatedRect> pickRowRectsLow = calc.pickRow(grouped, VisionResult.LOW_TARGET);
        assertEquals(1, pickRowRectsLow.size());

        List<IRotatedRect> pairedLowRects = calc.pickPairedRect(pickRowRectsLow);
        assertEquals(0, pairedLowRects.size());

        Set<IRotatedRect> pickRowRectsHigh = calc.pickRow(grouped, VisionResult.HIGH_TARGET);
        assertEquals(2, pickRowRectsHigh.size());

        List<IRotatedRect> pairedHighRects = calc.pickPairedRect(pickRowRectsHigh);
        assertEquals(2, pairedHighRects.size());


    }

}
