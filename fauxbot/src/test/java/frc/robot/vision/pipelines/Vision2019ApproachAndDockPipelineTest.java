package frc.robot.vision.pipelines;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import frc.robot.GamePiece;
import frc.robot.vision.common.VisionProcessingState;
import org.junit.jupiter.api.Test;
import org.opencv.core.*;
import org.opencv.imgcodecs.*;

import frc.robot.common.robotprovider.*;

import java.util.ArrayList;
import java.util.List;

public class Vision2019ApproachAndDockPipelineTest
{

    String basePath = TestConstants.BASE_PATH;
    String repoPath = "fauxbot\\src\\test\\resources\\frc.robot.vision.pipelines\\";

    @Test
    public void testLoadImage()
    {
        nu.pattern.OpenCV.loadShared();

        nu.pattern.OpenCV.loadShared();
        List<String> images = new ArrayList<>();
        images.add("Capture.PNG");
        images.add("Capture2.PNG");
        images.add("Capture3.PNG");
        images.add("Capture4.PNG");
        images.add("Capture5.PNG");
        images.add("Capture6.PNG");
        images.add("Capture7.PNG");

        System.out.println("\nLow targets only");
        for (String imageStr : images) {
            String imagePath = basePath + repoPath + imageStr;
            testImagePath(imagePath);
        }
        System.out.println("\nHigh targets only");
        for (String imageStr : images) {
            String imagePath = basePath + repoPath + imageStr;
            testImagePath(imagePath);
        }
    }

    private void testImagePath(String imagePath) {
        System.out.println(imagePath);
        Mat mat = Imgcodecs.imread(imagePath);
        MatWrapper wrapper = new MatWrapper(mat);
        Vision2019ApproachAndDockPipeline pipeline =
                new Vision2019ApproachAndDockPipeline(new FauxbotTimer(), new FauxbotProvider(), true);
        pipeline.setActivation(true);

        pipeline.process(wrapper);
//        assertNotNull(pipeline.getCenter());

    }

    private void testImagePathCargoRocket(String imagePath) {
        System.out.println(imagePath);
        Mat mat = Imgcodecs.imread(imagePath);
        MatWrapper wrapper = new MatWrapper(mat);
        Vision2019ApproachAndDockPipeline pipeline =
                new Vision2019ApproachAndDockPipeline(new FauxbotTimer(), new FauxbotProvider(), true);

        pipeline.setActivation(true);
        pipeline.setGamePiece(GamePiece.Cargo);
        pipeline.setMode(VisionProcessingState.ActiveRocket);
        pipeline.process(wrapper);
//        assertNotNull(pipeline.getCenter());

    }

}