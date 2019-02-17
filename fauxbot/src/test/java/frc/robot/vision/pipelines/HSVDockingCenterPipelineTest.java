package frc.robot.vision.pipelines;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.Test;
import org.opencv.core.*;
import org.opencv.imgcodecs.*;

import frc.robot.common.robotprovider.*;
import frc.robot.vision.common.VisionProcessingState;

public class HSVDockingCenterPipelineTest
{
    //@Test
    public void testLoadImage()
    {
        nu.pattern.OpenCV.loadShared();

        Mat mat = Imgcodecs.imread("fauxbot/src/test/resources/frc.robot.vision.pipelines/Capture.PNG");
        MatWrapper wrapper = new MatWrapper(mat);
        HSVDockingCenterPipeline pipeline = new HSVDockingCenterPipeline(new FauxbotTimer(), new FauxbotProvider(), true);
        pipeline.setActivation(VisionProcessingState.ActiveCargoShip);
        pipeline.process(wrapper);
        assertNotNull(pipeline.getCenter());
    }
}