package frc.robot.vision;

import frc.robot.common.robotprovider.IRotatedRect;
import frc.robot.common.robotprovider.RotatedRectWrapper;
import org.junit.jupiter.api.Test;
import org.opencv.core.RotatedRect;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.assertEquals;

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

    @Test
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


    @Test
    public void testPickRow_capture4() {
        VisionCalculations calc = new VisionCalculations();
        List<Set<IRotatedRect>> grouped = calc.groupRotatedRect(CAPTURE4_RECT_LIST);
        assertEquals(1, grouped.size());

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

    @Test
    public void testPickRow_capture5() {
        VisionCalculations calc = new VisionCalculations();
        List<Set<IRotatedRect>> grouped = calc.groupRotatedRect(CAPTURE5_RECT_LIST);
        assertEquals(1, grouped.size());

        List<IRotatedRect> pairedRects = calc.pickPairedRect(grouped.get(0));
        assertEquals(0, pairedRects.size());

    }


}
