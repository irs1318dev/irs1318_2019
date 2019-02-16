package frc.robot.vision.pipelines;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.Test;
import org.opencv.core.*;
import org.opencv.imgcodecs.*;

import frc.robot.common.robotprovider.*;

public class HSVDockingCenterPipelineTest
{
    @Test
    public void testLoadImage()
    {
        nu.pattern.OpenCV.loadShared();

        Mat mat = Imgcodecs.imread("fauxbot/src/test/resources/frc.robot.vision.pipelines/Capture.PNG");
        MatWrapper wrapper = new MatWrapper(mat);
        HSVDockingCenterPipeline pipeline = new HSVDockingCenterPipeline(new FauxbotTimer(), new FauxbotProvider(), true);
        pipeline.setActivation(true);
        pipeline.process(wrapper);
        assertNotNull(pipeline.getCenter());
    }

    @Test
    public void sampleTest()
    {

    }


}