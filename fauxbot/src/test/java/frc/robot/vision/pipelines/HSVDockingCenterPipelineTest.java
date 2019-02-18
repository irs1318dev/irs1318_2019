package frc.robot.vision.pipelines;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.Test;
import org.opencv.core.*;
import org.opencv.imgcodecs.*;

import frc.robot.common.robotprovider.*;
import frc.robot.vision.common.VisionProcessingState;

import java.util.ArrayList;
import java.util.List;

public class HSVDockingCenterPipelineTest
{

    // TODO
    String basePath = "C:\\Users\\james\\IdeaProjects2018\\irs1318_2019\\";
    String repoPath = "fauxbot\\src\\test\\resources\\frc.robot.vision.pipelines\\";

    @Test
    public void testLoadImage()
    {
        nu.pattern.OpenCV.loadShared();
        List<String> images = new ArrayList<>();
        images.add("Capture.PNG");
        images.add("Capture2.PNG");
        images.add("Capture3.PNG");
        images.add("Capture4.PNG");
        images.add("Capture5.PNG");
        images.add("Capture6.PNG");
        images.add("Capture7.PNG");

        for (String imageStr : images) {
            String imagePath = repoPath + imageStr;
            testImagePath(imagePath);
        }
    }

    private void testImagePath(String imagePath) {
        System.out.println(imagePath);
        Mat mat = Imgcodecs.imread(imagePath);
        MatWrapper wrapper = new MatWrapper(mat);
        HSVDockingCenterPipeline pipeline = new HSVDockingCenterPipeline(new FauxbotTimer(), new FauxbotProvider(), true);
        pipeline.setMode(VisionProcessingState.ActiveCargoShip);
        pipeline.process(wrapper);
        assertNotNull(pipeline.getCenter());

    }
}