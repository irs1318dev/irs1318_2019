package frc.robot.vision.pipelines;

import frc.robot.common.robotprovider.*;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.common.ContourHelper;
import frc.robot.vision.common.HSVFilter;
import frc.robot.vision.common.ImageUndistorter;

public class HSVDockingCenterPipeline implements ICentroidVisionPipeline
{
    private final ITimer timer;
    private final IOpenCVProvider openCVProvider;
    private final boolean shouldUndistort;
    private final ImageUndistorter undistorter;
    private final HSVFilter hsvFilter;

    private final IVideoStream frameInput;
    private final IVideoStream hsvOutput;

    // measured values
    //contour 
    private IPoint largestCenter;
    private IPoint secondLargestCenter;

    // private IPoint centerGoalPoint;

    //need to be calculated
    private Double measuredAngleX;
    private Double desiredAngleX;
    private Double distanceFromRobot;

    // FPS Measurement
    private long analyzedFrameCount;
    private double lastMeasuredTime;
    private double lastFpsMeasurement;

    // active status
    private volatile boolean isActive;

    /**
     * Initializes a new instance of the HSVCenterPipeline class.
     * @param timer to use for any timing purposes
     * @param shouldUndistort whether to undistort the image or not
     */
    public HSVDockingCenterPipeline(
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

        this.isActive = true;

        if (VisionConstants.SHOW_INPUT_FRAMES ||
            (VisionConstants.DEBUG && VisionConstants.DEBUG_OUTPUT_FRAMES))
        {
            this.frameInput = provider.getMJPEGStream("center.input", VisionConstants.LIFECAM_CAMERA_RESOLUTION_X, VisionConstants.LIFECAM_CAMERA_RESOLUTION_Y);
        }
        else
        {
            this.frameInput = null;
        }

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
     * @param frame image to analyze
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

        if (VisionConstants.SHOW_INPUT_FRAMES ||
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
        image = this.hsvFilter.filterHSV(image);
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

        // third, find the largest contour.
        IMatOfPoint[] largestContours = ContourHelper.findTwoLargestContours(
            this.openCVProvider, 
            image, 
            VisionConstants.CONTOUR_MIN_AREA,
            VisionConstants.DOCKING_RETROREFLECTIVE_TAPE_HxW_RATIO,
            VisionConstants.DOCKING_RETROREFLECTIVE_TAPE_RATIO_RANGE,
            VisionConstants.DOCKING_CONTOUR_ALLOWABLE_RATIO );
        
        IMatOfPoint largestContour = largestContours[0];
        IMatOfPoint secondLargestContour = largestContours[1];

        if (largestContour == null)
        {
            if (VisionConstants.DEBUG &&
                VisionConstants.DEBUG_PRINT_OUTPUT &&
                VisionConstants.DEBUG_PRINT_ANALYZER_DATA)
            {
                System.out.println("could not find any contour");
            }
        }

        // fourth, find the center of mass for the largest two contours
        IPoint largestCenterOfMass = null;
        IPoint secondLargestCenterOfMass = null;
        IRect largestBoundingRect = null;
        IRect secondLargestBoundingRect = null;
        
        if (largestContour != null)
        {
            largestCenterOfMass = ContourHelper.findCenterOfMass(this.openCVProvider, largestContour);
            largestBoundingRect = openCVProvider.boundingRect(largestContour);
            largestContour.release();
        }

        if (secondLargestContour != null)
        {
            secondLargestCenterOfMass = ContourHelper.findCenterOfMass(this.openCVProvider, secondLargestContour);
            secondLargestBoundingRect = openCVProvider.boundingRect(secondLargestContour);
            secondLargestContour.release();
        }

        if (VisionConstants.DEBUG)
        {
            if (VisionConstants.DEBUG_PRINT_OUTPUT &&
                VisionConstants.DEBUG_PRINT_ANALYZER_DATA)
            {
                if (largestCenterOfMass == null)
                {
                    System.out.println("couldn't find the center of mass!");
                }
                else
                {
                    System.out.println(String.format("Center of mass: %f, %f", largestCenterOfMass.getX(), largestCenterOfMass.getY()));
                }
                if (secondLargestCenterOfMass == null)
                {
                    System.out.println("couldn't find the center of mass!");
                }
                else
                {
                    System.out.println(String.format("Center of mass: %f, %f", secondLargestCenterOfMass.getX(), secondLargestCenterOfMass.getY()));
                }
            }
        }

        // finally, record the centers of mass
        this.largestCenter = largestCenterOfMass;
        this.secondLargestCenter = secondLargestCenterOfMass;

        undistortedImage.release();

        //Docking Calculations
        if(this.largestCenter == null && this.secondLargestCenter == null)
        {
            this.desiredAngleX = null;
            this.measuredAngleX = null;

            this.distanceFromRobot = null;

            return;
        }

        IPoint dockingMarkerCenter;
        IPoint otherMarkerCenter;
        IRect boundingRect;
    
        //Finding which side is robot on by finding the center value
        //left
        if (this.largestCenter != null && this.secondLargestCenter != null && this.largestCenter.getX() > this.secondLargestCenter.getX())
        {
           dockingMarkerCenter = this.secondLargestCenter;
           boundingRect = secondLargestBoundingRect; 
           otherMarkerCenter = this.largestCenter;
        }
        //right
        else
        {
            dockingMarkerCenter = this.largestCenter;
            boundingRect = largestBoundingRect;
            otherMarkerCenter = this.secondLargestCenter;
        }

        double dockingMarkerHeight = boundingRect.getHeight();
        if (dockingMarkerHeight == 0)
        {
            return;
        }

        double xOffsetMeasured = dockingMarkerCenter.getX() - VisionConstants.LIFECAM_CAMERA_CENTER_WIDTH;
        this.measuredAngleX = Math.atan(xOffsetMeasured / VisionConstants.LIFECAM_CAMERA_FOCAL_LENGTH_X) * VisionConstants.RADIANS_TO_ANGLE;

        double distanceFromCam = ((VisionConstants.DOCKING_RETROREFLECTIVE_TAPE_HEIGHT) / (Math.tan(VisionConstants.LIFECAM_CAMERA_FIELD_OF_VIEW_Y_RADIANS)))
        * ((double)VisionConstants.LIFECAM_CAMERA_RESOLUTION_Y / dockingMarkerHeight);


        //Need to figure out???? 
    this.distanceFromRobot = distanceFromCam * Math.cos(this.measuredAngleX * VisionConstants.ANGLE_TO_RADIANS) + VisionConstants.DOCKING_CAMERA_MOUNTING_DISTANCE;
    this.desiredAngleX = Math.asin(VisionConstants.DOCKING_CAMERA_HORIZONTAL_MOUNTING_OFFSET / distanceFromCam) * VisionConstants.RADIANS_TO_ANGLE;

    // double[] centerGoal= new double[1];
    // centerGoal[0]= ((dockingMarkerCenter.getX()-otherMarkerCenter.getX())/2) + dockingMarkerCenter.getX();
    // centerGoal[1] = dockingMarkerCenter.getY();
    // centerGoalPoint.set(centerGoal);
    }

    public void setActivation(boolean isActive)
    {
        this.isActive = isActive;
    }

    public boolean isActive()
    {
        return this.isActive;
    }

    public IPoint getCenter()
    {
        return this.largestCenter;
    }

    // public IPoint getCenterGoal()
    // {
    //     return this.centerGoalPoint;
    // }

    public Double getDesiredAngleX()
    {
        return 0.0;
    }

    public Double getMeasuredAngleX()
    {
        return this.measuredAngleX;
    }

    public Double getRobotDistance()
    {
        return null;
    }

    public double getFps()
    {
        return this.lastFpsMeasurement;
    }
}