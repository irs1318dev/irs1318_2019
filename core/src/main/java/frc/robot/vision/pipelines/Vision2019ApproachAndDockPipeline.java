package frc.robot.vision.pipelines;

import frc.robot.GamePiece;
import frc.robot.common.robotprovider.*;
import frc.robot.vision.PixelsToInches;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.common.ContourHelper;
import frc.robot.vision.common.HSVFilter;
import frc.robot.vision.common.ImageUndistorter;
import frc.robot.vision.common.VisionProcessingState;

import java.util.*;

public class Vision2019ApproachAndDockPipeline implements ICentroidVisionPipeline
{
    private final ITimer timer;
    private final IOpenCVProvider openCVProvider;
    private final boolean shouldUndistort;
    private final ImageUndistorter undistorter;
    private final HSVFilter hsvFilter;

    private final IVideoStream frameInput;
    private final IVideoStream hsvOutput;

    // measured values
    private IPoint largestCenter;
    private IPoint secondLargestCenter;

    // need to be calculated
    private Double measuredAngleX;
    private Double desiredAngleX;
    private Double distanceFromRobot;

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

        this.openCVProvider = provider.getOpenCVProvider();
        this.undistorter = new ImageUndistorter(this.openCVProvider);
        IScalar lowFilter = this.openCVProvider.newScalar(VisionConstants.LIFECAM_HSV_FILTER_LOW_V0, VisionConstants.LIFECAM_HSV_FILTER_LOW_V1, VisionConstants.LIFECAM_HSV_FILTER_LOW_V2);
        IScalar highFilter = this.openCVProvider.newScalar(VisionConstants.LIFECAM_HSV_FILTER_HIGH_V0, VisionConstants.LIFECAM_HSV_FILTER_HIGH_V1, VisionConstants.LIFECAM_HSV_FILTER_HIGH_V2);
        this.hsvFilter = new HSVFilter(this.openCVProvider, lowFilter, highFilter);

        this.largestCenter = null;
        this.secondLargestCenter = null;

        this.measuredAngleX = null;
        this.desiredAngleX = null;
        this.distanceFromRobot = null; 

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
        List<Set<IRotatedRect>> groupedRects = groupRotatedRect(rectangles);
        int setNum = 0;
        for (Set<IRotatedRect> group : groupedRects) {
            for (int i = 0; i< rectangles.size(); i++) {
                IRotatedRect rectangle = rectangles.get(i);
                System.out.println("S_" + setNum + "_R_"+ i +" : "
                        + Arrays.toString(rectangle.getRawValues()));
            }
            setNum++;
        }
        Set<IRotatedRect> row = pickRow(groupedRects);
        List<IRotatedRect> pair = pickPairedRect(row);
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

    public List<Set<IRotatedRect>> groupRotatedRect(List<IRotatedRect> rotatedRect)
    {
        List<Set<IRotatedRect>> rows = new ArrayList<>();
        for(IRotatedRect rect : rotatedRect)
        {
            boolean added = false;
            for(Set<IRotatedRect> row: rows)
            {
                IRotatedRect compare = row.stream().findFirst().get();
                if(Math.abs(rect.getCenter().getY() - compare.getCenter().getY())< 30)
                {
                    row.add(rect);
                    added = true;
                    break;
                }
            }
            if(added){
                continue;
            }
            Set<IRotatedRect> row = new HashSet<>();
            row.add(rect);
            rows.add(row);
        }
        return rows;
    }

    public Set<IRotatedRect> pickRow(List<Set<IRotatedRect>> rows)
    {
        if(rows.size() == 1){
            return rows.get(0);
        }
        if(this.gamePiece.equals(GamePiece.Cargo) && this.processingState.equals(VisionProcessingState.ActiveRocket))
        {
            double lowestY = 1000;
            Set<IRotatedRect> lowestRow = null;
            for(Set<IRotatedRect> row : rows)
            {
                IRotatedRect rect = row.stream().findFirst().get();
                if(rect.getCenter().getY() < lowestY){
                    lowestY = rect.getCenter().getY();
                    lowestRow = row;
                }

            }
           return lowestRow; 
        }
        else
        {
            double highestY = 0;
            Set<IRotatedRect> highestRow = null;
            for(Set<IRotatedRect> row : rows)
            {
                IRotatedRect rect = row.stream().findFirst().get();
                if(rect.getCenter().getY() > highestY){
                    highestY = rect.getCenter().getY();
                    highestRow = row;
                }

            }
            return highestRow;
        }
    }

    boolean isLeft(IRotatedRect rect) {
        if(rect.getAngle() < -65 && rect.getAngle() > -85) {
            return true;
        }
        return false;
    }

    boolean isRight(IRotatedRect rect) {
        if(rect.getAngle() < 0  && rect.getAngle() > -15) {
            return true;
        }
        return false;
    }
    List<IRotatedRect> largestRect(List<IRotatedRect> rect)
    {
        double area = 0;
        List<IRotatedRect> pair = new ArrayList<>();
        IRotatedRect left = null;
        IRotatedRect right = null;
        for(int i = 0; i < rect.size(); i++)
        {
            if(isLeft(rect.get(i))){
                if(area < (rect.get(i).size().getHeight() * rect.get(i).size().getWidth()))
                {
                    area = (rect.get(i).size().getHeight() * rect.get(i).size().getWidth());
                    left = rect.get(i);
                    right = rect.get(i+1);
                }
            }
        }
        pair.add(left);
        pair.add(right);
        return pair;
    }

    public List<IRotatedRect> pickPairedRect(Set<IRotatedRect> rects)
    {
        List<IRotatedRect> rectList = new ArrayList<>();
        List<IRotatedRect> pairedRect = new ArrayList<>();

        rectList.addAll(rects);
        Collections.sort(rectList, new Comparator<IRotatedRect>() {

        @Override
        public int compare(IRotatedRect arg0, IRotatedRect arg1) {
            return Double.compare(arg0.getCenter().getX(), arg1.getCenter().getX());
        }
        });
        for(int i = 0; i < rectList.size() - 1; i++)
        {
            if(isLeft(rectList.get(i)))
            {
                pairedRect.add(rectList.get(i));
                pairedRect.add(rectList.get(i+1));
            }
        }
        return largestRect(pairedRect);
    }

    public int findInterval(double avgPixelValue, List<PixelsToInches> interpolateList)
    {
        for(int i = 0; i < interpolateList.size(); i++)
        {
            PixelsToInches pixelsToInches = VisionConstants.PIXELS_TO_INCHES.get(i);
            if(avgPixelValue > pixelsToInches.getPixels())
            {
                return i - 1;
            }
        }
        return 0;
    }

    public double computeAvgPixel(List<IRotatedRect> rects)
    {
        IRotatedRect left = rects.get(0);
        IRotatedRect right = rects.get(1);

        return ((left.size().getWidth() + right.size().getHeight())/ 2.0);
    }

    public double interpolateInchesFromPixels(double avgPixel, int interval, List<PixelsToInches> interpolateList)
    {
        PixelsToInches low = interpolateList.get(interval);
        PixelsToInches high = interpolateList.get(interval + 1);
        double slope = (low.getInches()- high.getInches())/(low.getPixels() - high.getPixels());
        double inches = slope * (avgPixel - low.getPixels()) + low.getInches();
        return inches;
    }

    
    public double azimuth(double avgPixels, List<IRotatedRect> list)
    {
        IRotatedRect left = list.get(0);
        IRotatedRect right = list.get(1);
        //TODO: check for divide by zero
        double inchesPerPixels = VisionConstants.DOCKING_RETROREFLECTIVE_TAPE_HEIGHT_STRAIGHT / avgPixels;
        double observedPixels = (right.getCenter().getX() - left.getCenter().getX());
        double distanceBetweenCenters = VisionConstants.DOCKING_DISTANCE_BETWEEN_TAPE_TARGETS / inchesPerPixels;
        double azimuth = Math.acos(observedPixels/ distanceBetweenCenters);
        return azimuth;
    }

    public double calculateIntitialApproachTurnAngle(double azimuth, double pixelsPerInch, List<IRotatedRect> list, double distance)
    {
        IRotatedRect left = list.get(0);
        IRotatedRect right = list.get(1);
        //In radians
        double targetCenter = VisionConstants.LIFECAM_CAMERA_RESOLUTION_X 
                            + pixelsPerInch * VisionConstants.DOCKING_CAMERA_HORIZONTAL_MOUNTING_OFFSET
                            * Math.cos(azimuth);

        double observedTargetCenter = left.getCenter().getX() 
                                    + (right.getCenter().getX() -left.getCenter().getX()) / 2.0; 
        double initialTurnAngle = (Math.atan(targetCenter - observedTargetCenter))/ distance;
        return initialTurnAngle;
    }

    public double calculateTravelDistance(double azimuth, double distance)
    {
        double travelDistance = Math.sqrt(
            Math.pow(
                (distance*Math.sin(azimuth) 
                - VisionConstants.DOCKING_CAMERA_HORIZONTAL_MOUNTING_OFFSET), 2) 
            + Math.pow((distance*Math.cos(azimuth)), 2));
            return travelDistance;
    }

    public double finalApproachTurn(double azimuth, double distance)
    {
        //24 is fill-in for docking offset
        double approachAngle = Math.atan2(distance*Math.sin(azimuth)- 24, distance*Math.cos(azimuth));
        //convert to radians!!!!!
        return Math.PI/2 - approachAngle;
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