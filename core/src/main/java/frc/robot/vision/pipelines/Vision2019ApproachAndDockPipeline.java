package frc.robot.vision.pipelines;

import frc.robot.GamePiece;
import frc.robot.common.robotprovider.*;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.common.ContourHelper;
import frc.robot.vision.common.HSVFilter;
import frc.robot.vision.common.ImageUndistorter;
import frc.robot.vision.common.VisionProcessingState;
import frc.robot.vision.VisionCalculations;

import java.util.*;

public class Vision2019ApproachAndDockPipeline implements ICentroidVisionPipeline
{
    private final VisionCalculations calc;
    private final ITimer timer;
    private final IOpenCVProvider openCVProvider;
    private final boolean shouldUndistort;
    private final ImageUndistorter undistorter;
    private final HSVFilter hsvFilter;

    private final IVideoStream frameInput;
    private final IVideoStream hsvOutput;

    // measured values
    private IPoint largestCenter;
    //private IPoint secondLargestCenter;

    // need to be calculated
    private Double measuredAngleX;
    private Double desiredAngleX;
    private Double distanceFromRobot;

    //other calculated values
    private Double initialTurnAngle;
    private Double finalApproachTurn;
    private Double travelDistance;

    // FPS Measurement
    private long analyzedFrameCount;
    private double lastMeasuredTime;
    private double lastFpsMeasurement;

    // active status
    private volatile boolean isActive;
    private volatile GamePiece gamePiece;
    private volatile boolean streamEnabled;
    private volatile VisionProcessingState processingState;

    /**
     * Initializes a new instance of the HSVCenterPipeline class.
     * @param timer to use for any timing purposes
     * @param shouldUndistort whether to undistort the image or not
     */
    public Vision2019ApproachAndDockPipeline(
        ITimer timer,
        IRobotProvider provider,
        boolean shouldUndistort)
    {
        this.shouldUndistort = shouldUndistort;

        this.calc = provider.getVisionCalculations();
        this.openCVProvider = provider.getOpenCVProvider();
        this.undistorter = new ImageUndistorter(this.openCVProvider);
        IScalar lowFilter = this.openCVProvider.newScalar(VisionConstants.LIFECAM_HSV_FILTER_LOW_V0, VisionConstants.LIFECAM_HSV_FILTER_LOW_V1, VisionConstants.LIFECAM_HSV_FILTER_LOW_V2);
        IScalar highFilter = this.openCVProvider.newScalar(VisionConstants.LIFECAM_HSV_FILTER_HIGH_V0, VisionConstants.LIFECAM_HSV_FILTER_HIGH_V1, VisionConstants.LIFECAM_HSV_FILTER_HIGH_V2);
        this.hsvFilter = new HSVFilter(this.openCVProvider, lowFilter, highFilter);

        this.largestCenter = null;
        //this.secondLargestCenter = null;

        this.measuredAngleX = null;
        this.desiredAngleX = null;
        this.distanceFromRobot = null; 

        this.travelDistance = null;
        this.finalApproachTurn = null;
        this.initialTurnAngle = null;

        this.analyzedFrameCount = 0;
        this.timer = timer;
        this.lastMeasuredTime = this.timer.get();

        this.isActive = false;
        this.streamEnabled = true;
        this.gamePiece = GamePiece.None;
        this.processingState = VisionProcessingState.Disabled;

        this.frameInput = provider.getMJPEGStream("center.input", VisionConstants.LIFECAM_CAMERA_RESOLUTION_X, VisionConstants.LIFECAM_CAMERA_RESOLUTION_Y);

        if (VisionConstants.DEBUG &&
            VisionConstants.DEBUG_OUTPUT_FRAMES)
        {
            this.hsvOutput =  provider.getMJPEGStream("center.hsv", VisionConstants.LIFECAM_CAMERA_RESOLUTION_X, VisionConstants.LIFECAM_CAMERA_RESOLUTION_Y);
        }
        else
        {
            this.hsvOutput = null;
        }
    }

    /**
     * Process a single image frame
     */
    @Override
    public void process(IMat image)
    {
        if (VisionConstants.DEBUG)
        {
            if (VisionConstants.DEBUG_SAVE_FRAMES &&
                    this.analyzedFrameCount % VisionConstants.DEBUG_FRAME_OUTPUT_GAP == 0)
            {
                this.openCVProvider.imwrite(
                        String.format("%simage%d-1.jpg", VisionConstants.DEBUG_OUTPUT_FOLDER, this.analyzedFrameCount),
                        image);
            }
        }

        if (this.streamEnabled ||
                (VisionConstants.DEBUG && VisionConstants.DEBUG_OUTPUT_FRAMES))
        {
            this.frameInput.putFrame(image);
        }

        if (!this.isActive)
        {
            return;
        }

        this.analyzedFrameCount++;
        if (VisionConstants.DEBUG &&
                VisionConstants.DEBUG_PRINT_OUTPUT &&
                this.analyzedFrameCount % VisionConstants.DEBUG_FPS_AVERAGING_INTERVAL == 0)
        {
            double now = this.timer.get();
            double elapsedTime = now - this.lastMeasuredTime;

            this.lastFpsMeasurement = ((double)VisionConstants.DEBUG_FPS_AVERAGING_INTERVAL) / elapsedTime;
            this.lastMeasuredTime = this.timer.get();
        }

        // first, undistort the image.
        IMat undistortedImage;
        if (this.shouldUndistort)
        {
            image = this.undistorter.undistortFrame(image);
        }

        // save the undistorted image for possible output later...
        if (this.shouldUndistort)
        {
            undistortedImage = image.clone();
        }
        else
        {
            undistortedImage = image;
        }

        // second, filter HSV
        image = this.hsvFilter.filterHSV(undistortedImage);
        if (VisionConstants.DEBUG)
        {
            if (VisionConstants.DEBUG_SAVE_FRAMES &&
                    this.analyzedFrameCount % VisionConstants.DEBUG_FRAME_OUTPUT_GAP == 0)
            {
                this.openCVProvider.imwrite(
                        String.format("%simage%d-2.hsvfiltered.jpg", VisionConstants.DEBUG_OUTPUT_FOLDER, this.analyzedFrameCount),
                        image);
            }

            if (VisionConstants.DEBUG_OUTPUT_FRAMES)
            {
                this.hsvOutput.putFrame(image);
            }
        }

        List<IRotatedRect> rectangles =  findRectangles(image);
        for (int i = 0; i< rectangles.size(); i++) {
            IRotatedRect rectangle = rectangles.get(i);
            System.out.println("R_"+ i +" : "
                    + Arrays.toString(rectangle.getRawValues()));
        }
        List<Set<IRotatedRect>> groupedRects = calc.groupRotatedRect(rectangles);
        int setNum = 0;
        for (Set<IRotatedRect> group : groupedRects) {
            for (int i = 0; i< rectangles.size(); i++) {
                IRotatedRect rectangle = rectangles.get(i);
                System.out.println("S_" + setNum + "_R_"+ i +" : "
                        + Arrays.toString(rectangle.getRawValues()));
            }
            setNum++;
        }
        Set<IRotatedRect> row = calc.pickRow(groupedRects, gamePiece, processingState);
        List<IRotatedRect> pair = calc.pickPairedRect(row);
        double avgPixel = calc.computeAvgPixel(pair);
        int interval = calc.findInterval(avgPixel, VisionConstants.PIXELS_TO_INCHES);
        double pixelsPerInch = calc.interpolateInchesFromPixels(avgPixel, interval, VisionConstants.PIXELS_TO_INCHES);
        double azimuth = calc.azimuth(avgPixel, pair);
        this.initialTurnAngle = calc.calculateIntitialApproachTurnAngle(azimuth, pixelsPerInch, pair, pixelsPerInch);
        this.travelDistance = calc.calculateTravelDistance(azimuth, pixelsPerInch);
        this.finalApproachTurn = calc.finalApproachTurn(azimuth, pixelsPerInch);


    }

    public void prepareImage(IMat image) {
        if (VisionConstants.DEBUG)
        {
            if (VisionConstants.DEBUG_SAVE_FRAMES &&
                    this.analyzedFrameCount % VisionConstants.DEBUG_FRAME_OUTPUT_GAP == 0)
            {
                this.openCVProvider.imwrite(
                        String.format("%simage%d-1.jpg", VisionConstants.DEBUG_OUTPUT_FOLDER, this.analyzedFrameCount),
                        image);
            }
        }

        if (this.streamEnabled ||
                (VisionConstants.DEBUG && VisionConstants.DEBUG_OUTPUT_FRAMES))
        {
            this.frameInput.putFrame(image);
        }

        if (!this.isActive)
        {
            return;
        }

        this.analyzedFrameCount++;
        if (VisionConstants.DEBUG &&
                VisionConstants.DEBUG_PRINT_OUTPUT &&
                this.analyzedFrameCount % VisionConstants.DEBUG_FPS_AVERAGING_INTERVAL == 0)
        {
            double now = this.timer.get();
            double elapsedTime = now - this.lastMeasuredTime;

            this.lastFpsMeasurement = ((double)VisionConstants.DEBUG_FPS_AVERAGING_INTERVAL) / elapsedTime;
            this.lastMeasuredTime = this.timer.get();
        }

        // first, undistort the image.
        IMat undistortedImage;
        if (this.shouldUndistort)
        {
            image = this.undistorter.undistortFrame(image);
        }

        // save the undistorted image for possible output later...
        if (this.shouldUndistort)
        {
            undistortedImage = image.clone();
        }
        else
        {
            undistortedImage = image;
        }

        // second, filter HSV
        image = this.hsvFilter.filterHSV(undistortedImage);
        if (VisionConstants.DEBUG)
        {
            if (VisionConstants.DEBUG_SAVE_FRAMES &&
                    this.analyzedFrameCount % VisionConstants.DEBUG_FRAME_OUTPUT_GAP == 0)
            {
                this.openCVProvider.imwrite(
                        String.format("%simage%d-2.hsvfiltered.jpg", VisionConstants.DEBUG_OUTPUT_FOLDER, this.analyzedFrameCount),
                        image);
            }

            if (VisionConstants.DEBUG_OUTPUT_FRAMES)
            {
                this.hsvOutput.putFrame(image);
            }
        }
//        undistortedImage.release();
    }

    public List<IRotatedRect> findRectangles(IMat image){
        List<IMatOfPoint> contours = ContourHelper.getAllContours(openCVProvider, image, VisionConstants.CONTOUR_MIN_AREA);

        List<IRotatedRect> rotatedRect = new ArrayList<>();

        for(IMatOfPoint contour: contours){
            rotatedRect.add(openCVProvider.minAreaRect(openCVProvider.convertToMatOfPoints2f(contour)));
            contour.release();
        }
        return rotatedRect;
    }

    public double getInitialTurnAngle()
    {
        return this.initialTurnAngle;
    }

    public double getFinalApproachTurn()
    {
        return this.finalApproachTurn;
    }

    public double getTravelDistance()
    {
        return this.travelDistance;
    }

    public void setActivation(boolean isActive)
    {
        this.isActive = isActive;
    }

    public void setStreamMode(boolean isEnabled)
    {
        this.streamEnabled = isEnabled;
    }

    public boolean isActive()
    {
        return this.isActive;
    }

    public IPoint getCenter()
    {
        return this.largestCenter;
    }

    public Double getDesiredAngleX()
    {
        return this.desiredAngleX;
    }

    public Double getMeasuredAngleX()
    {
        return this.measuredAngleX;
    }

    public Double getRobotDistance()
    {
        return this.distanceFromRobot;
    }

    public double getFps()
    {
        return this.lastFpsMeasurement;
    }
    public void setGamePiece(GamePiece gamePiece){
        this.gamePiece = gamePiece;
    }
    public void setMode(VisionProcessingState processingState){
        this.processingState = processingState;
    }
}