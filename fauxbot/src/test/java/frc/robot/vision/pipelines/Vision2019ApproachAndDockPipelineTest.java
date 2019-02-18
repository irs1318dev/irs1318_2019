package frc.robot.vision.pipelines;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.Test;
import org.opencv.core.*;
import org.opencv.imgcodecs.*;

import frc.robot.common.robotprovider.*;

public class Vision2019ApproachAndDockPipelineTest
{
    //@Test
    public void testLoadImage()
    {
        nu.pattern.OpenCV.loadShared();

        String testImage = "C:\\Users\\samik\\git\\irs1318_2019\\fauxbot\\src\\test\\resources\\frc.robot.vision.pipelines\\Capture.PNG";
        String resourceImage = "fauxbot/src/test/resources/frc.robot.vision.pipelines/Capture.PNG";
        Mat mat = Imgcodecs.imread(testImage);
        MatWrapper wrapper = new MatWrapper(mat);
        Vision2019ApproachAndDockPipeline pipeline = new Vision2019ApproachAndDockPipeline(new FauxbotTimer(), new FauxbotProvider(), true);
        pipeline.setActivation(true);
        pipeline.process(wrapper);
        assertNotNull(pipeline.getCenter());
    }

}