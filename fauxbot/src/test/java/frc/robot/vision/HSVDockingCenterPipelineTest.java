package frc.robot.vision;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.Test;
import org.opencv.core.*;
import org.opencv.imgcodecs.*;
import frc.robot.common.robotprovider.*;
import frc.robot.vision.pipelines.HSVDockingCenterPipeline;

public class HSVDockingCenterPipelineTest
{
    @Test
    public void testLoadImage()
    {
        nu.pattern.OpenCV.loadShared();

        Mat mat = Imgcodecs.imread("fauxbot/src/main/resources/images/Capture.PNG");
        MatWrapper wrapper = new MatWrapper(mat);
        HSVDockingCenterPipeline pipeline = new HSVDockingCenterPipeline(new FauxbotTimer(), new FauxbotProvider(), true);
        pipeline.setActivation(true);
        pipeline.process(wrapper);
        assertNotNull(pipeline.getCenter());
    }
}