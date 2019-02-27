package frc.robot.vision.pipelines;

import frc.robot.*;
import frc.robot.common.robotprovider.*;
import frc.robot.vision.*;
import frc.robot.vision.common.*;

import java.util.*;

public class Vision2019ApproachAndDockPipeline implements ICentroidVisionPipeline {
    private final VisionCalculations calc;
    private final ITimer timer;
    private final IOpenCVProvider openCVProvider;
    private final boolean shouldUndistort;
    private final ImageUndistorter undistorter;
    private final HSVFilter hsvFilter;

    private final IVideoStream frameInput;
    private final IVideoStream hsvOutput;

    // measured values
    private IPoint dockingMarkerCenter;

    // need to be calculated
    private VisionResult visionResult;

    // docking values
    private Double measuredAngleX;
    private Double desiredAngleX;
    private Double distanceFromRobot;

    // other calculated values
    private Double initialTurnAngle;
    private Double finalApproachTurn;
    private Double travelDistance;

    // glide calcs
    private Double glideDistance;
    private Double sweepAngle;
    private Double glideRadius;

    // FPS Measurement
    private long analyzedFrameCount;
    private double lastMeasuredTime;
    private double lastFpsMeasurement;

    // active status
    private volatile GamePiece gamePiece;
    private volatile boolean streamEnabled;
    private volatile VisionProcessingState processingState;

    /**
     * Initializes a new instance of the HSVCenterPipeline class.
     *
     * @param timer           to use for any timing purposes
     * @param shouldUndistort whether to undistort the image or not
     */
    public Vision2019ApproachAndDockPipeline(
            ITimer timer,
            IRobotProvider provider,
            boolean shouldUndistort) {
        this.shouldUndistort = shouldUndistort;

        this.calc = provider.getVisionCalculations();
        this.openCVProvider = provider.getOpenCVProvider();
        this.undistorter = new ImageUndistorter(this.openCVProvider);
        IScalar lowFilter = this.openCVProvider.newScalar(VisionConstants.LIFECAM_HSV_FILTER_LOW_V0, VisionConstants.LIFECAM_HSV_FILTER_LOW_V1, VisionConstants.LIFECAM_HSV_FILTER_LOW_V2);
        IScalar highFilter = this.openCVProvider.newScalar(VisionConstants.LIFECAM_HSV_FILTER_HIGH_V0, VisionConstants.LIFECAM_HSV_FILTER_HIGH_V1, VisionConstants.LIFECAM_HSV_FILTER_HIGH_V2);
        this.hsvFilter = new HSVFilter(this.openCVProvider, lowFilter, highFilter);

        this.dockingMarkerCenter = null;

        this.visionResult = VisionResult.NONE;

        this.measuredAngleX = null;
        this.desiredAngleX = null;
        this.distanceFromRobot = null;

        this.travelDistance = null;
        this.finalApproachTurn = null;
        this.initialTurnAngle = null;

        this.glideDistance = null;
        this.glideRadius = null;
        this.sweepAngle = null;

        this.analyzedFrameCount = 0;
        this.timer = timer;
        this.lastMeasuredTime = this.timer.get();

        this.processingState = VisionProcessingState.Disabled;
        this.streamEnabled = true;
        this.gamePiece = GamePiece.None;

        this.frameInput = provider.getMJPEGStream("center.input", VisionConstants.LIFECAM_CAMERA_RESOLUTION_X, VisionConstants.LIFECAM_CAMERA_RESOLUTION_Y);

        if (VisionConstants.DEBUG &&
                VisionConstants.DEBUG_OUTPUT_FRAMES) {
            this.hsvOutput = provider.getMJPEGStream("center.hsv", VisionConstants.LIFECAM_CAMERA_RESOLUTION_X, VisionConstants.LIFECAM_CAMERA_RESOLUTION_Y);
        } else {
            this.hsvOutput = null;
        }
    }

    /**
     * Process a single image frame
     */
    @Override
    public void process(IMat image) {
        if (VisionConstants.DEBUG) {
            if (VisionConstants.DEBUG_SAVE_FRAMES &&
                    this.analyzedFrameCount % VisionConstants.DEBUG_FRAME_OUTPUT_GAP == 0) {
                this.openCVProvider.imwrite(
                        String.format("%simage%d-1.jpg", VisionConstants.DEBUG_OUTPUT_FOLDER, this.analyzedFrameCount),
                        image);
            }
        }

        if (this.streamEnabled ||
                (VisionConstants.DEBUG && VisionConstants.DEBUG_OUTPUT_FRAMES)) {
            this.frameInput.putFrame(image);
        }

        if (this.processingState == VisionProcessingState.Disabled) {
            return;
        }

        this.analyzedFrameCount++;
        if (VisionConstants.DEBUG &&
                VisionConstants.DEBUG_PRINT_OUTPUT &&
                this.analyzedFrameCount % VisionConstants.DEBUG_FPS_AVERAGING_INTERVAL == 0) {
            double now = this.timer.get();
            double elapsedTime = now - this.lastMeasuredTime;

            this.lastFpsMeasurement = ((double) VisionConstants.DEBUG_FPS_AVERAGING_INTERVAL) / elapsedTime;
            this.lastMeasuredTime = this.timer.get();
        }

        // first, undistort the image.
        if (this.shouldUndistort) {
            image = this.undistorter.undistortFrame(image);
        }

        // second, filter HSV
        image = this.hsvFilter.filterHSV(image);
        if (VisionConstants.DEBUG) {
            if (VisionConstants.DEBUG_SAVE_FRAMES &&
                    this.analyzedFrameCount % VisionConstants.DEBUG_FRAME_OUTPUT_GAP == 0) {
                this.openCVProvider.imwrite(
                        String.format("%simage%d-2.hsvfiltered.jpg", VisionConstants.DEBUG_OUTPUT_FOLDER, this.analyzedFrameCount),
                        image);
            }

            if (VisionConstants.DEBUG_OUTPUT_FRAMES) {
                this.hsvOutput.putFrame(image);
            }
        }

        List<IRotatedRect> rectangles = findRectangles(image);
        image.release();
        if (VisionConstants.DEBUG &&
                VisionConstants.DEBUG_PRINT_OUTPUT &&
                VisionConstants.DEBUG_PRINT_ANALYZER_DATA) {
            for (int i = 0; i < rectangles.size(); i++) {
                IRotatedRect rectangle = rectangles.get(i);
                System.out.println("R_" + i + " : "
                        + Arrays.toString(rectangle.getRawValues()));
            }
        }
        List<Set<IRotatedRect>> groupedRects = calc.groupRotatedRect(rectangles);
        if (VisionConstants.DEBUG &&
                VisionConstants.DEBUG_PRINT_OUTPUT &&
                VisionConstants.DEBUG_PRINT_ANALYZER_DATA) {
            int setNum = 0;
            for (Set<IRotatedRect> group : groupedRects) {
                List<IRotatedRect> groupList = new ArrayList<>();
                groupList.addAll(group);
                for (int i = 0; i < groupList.size(); i++) {
                    IRotatedRect rectangle = groupList.get(i);
                    System.out.println("S_" + setNum + "_R_" + i + " : "
                            + Arrays.toString(rectangle.getRawValues()));
                }
                setNum++;
            }
        }

        this.visionResult = calc.determineVisionResult(gamePiece, processingState);
        Set<IRotatedRect> row = calc.pickRow(groupedRects, this.visionResult);
        List<IRotatedRect> pair = calc.pickPairedRect(row);
        if (pair.size() == 2) {
            double avgPixel = calc.computeAvgPixel(pair);
            int interval = calc.findInterval(avgPixel, VisionConstants.PIXELS_TO_INCHES);
            double pixelsPerInch = calc.interpolateInchesFromPixels(avgPixel, interval, VisionConstants.PIXELS_TO_INCHES);
            double azimuth = calc.azimuth(avgPixel, pair);
            this.initialTurnAngle = calc.calculateIntitialApproachTurnAngle(azimuth, pixelsPerInch, pair, pixelsPerInch);
            this.travelDistance = calc.calculateTravelDistance(azimuth, pixelsPerInch);
            this.finalApproachTurn = calc.finalApproachTurn(azimuth, pixelsPerInch);

            this.glideRadius = calc.glideRadius(azimuth, pixelsPerInch);
            this.sweepAngle = calc.findSweepAngle(pixelsPerInch, azimuth, glideRadius);
            this.glideDistance = calc.findGlideDistance(glideRadius, sweepAngle);

            /*
            this.dockingMarkerCenter = pair.get(0).getCenter();
            double xOffsetMeasured = this.dockingMarkerCenter.getX() - VisionConstants.LIFECAM_CAMERA_CENTER_WIDTH;
            double yOffsetMeasured = VisionConstants.LIFECAM_CAMERA_CENTER_HEIGHT - this.dockingMarkerCenter.getY();
            this.measuredAngleX = Math.atan(xOffsetMeasured / VisionConstants.LIFECAM_CAMERA_FOCAL_LENGTH_X) * VisionConstants.RADIANS_TO_ANGLE;
            double measuredAngleY = Math.atan(yOffsetMeasured / VisionConstants.LIFECAM_CAMERA_FOCAL_LENGTH_Y) * VisionConstants.RADIANS_TO_ANGLE;

            double distanceFromCam = (VisionConstants.DOCKING_CAMERA_MOUNTING_HEIGHT - VisionConstants.ROCKET_TO_GROUND_TAPE_HEIGHT)/(Math.tan((VisionConstants.DOCKING_CAMERA_VERTICAL_MOUNTING_ANGLE - measuredAngleY) * VisionConstants.ANGLE_TO_RADIANS));
            this.distanceFromRobot = distanceFromCam * Math.cos(this.measuredAngleX * VisionConstants.ANGLE_TO_RADIANS) - VisionConstants.DOCKING_CAMERA_MOUNTING_DISTANCE;
            this.desiredAngleX = Math.asin((VisionConstants.DOCKING_CAMERA_HORIZONTAL_MOUNTING_OFFSET - VisionConstants.DOCKING_TAPE_OFFSET) / distanceFromCam) * VisionConstants.RADIANS_TO_ANGLE;
            */
        } else {
            this.visionResult = VisionResult.NONE;
        }
    }

    public List<IRotatedRect> findRectangles(IMat image) {
        List<IMatOfPoint> contours = ContourHelper.getAllContours(openCVProvider, image, VisionConstants.CONTOUR_MIN_AREA);

        List<IRotatedRect> rotatedRect = new ArrayList<IRotatedRect>(contours.size());
        for (IMatOfPoint contour : contours) {
            rotatedRect.add(openCVProvider.minAreaRect(openCVProvider.convertToMatOfPoints2f(contour)));
            contour.release();
        }

        return rotatedRect;
    }

    public void setMode(VisionProcessingState processingState) {
        this.processingState = processingState;
    }

    public void setGamePiece(GamePiece gamePiece) {
        this.gamePiece = gamePiece;
    }

    public void setStreamMode(boolean isEnabled) {
        this.streamEnabled = isEnabled;
    }

    public boolean isActive() {
        return this.processingState != VisionProcessingState.Disabled;
    }

    public double getInitialTurnAngle() {
        return this.initialTurnAngle;
    }

    public double getFinalApproachTurn() {
        return this.finalApproachTurn;
    }

    public double getTravelDistance() {
        return this.travelDistance;
    }

    public double getSweepAngle() {
        return this.sweepAngle;
    }

    public double getGlideDistance() {
        return this.glideDistance;
    }

    public double getGlideRadius() {
        return this.glideRadius;
    }

    public IPoint getCenter() {
        return this.dockingMarkerCenter;
    }

    public Double getDesiredAngleX() {
        return this.desiredAngleX;
    }

    public Double getMeasuredAngleX() {
        return this.measuredAngleX;
    }

    public Double getRobotDistance() {
        return this.distanceFromRobot;
    }

    public double getFps() {
        return this.lastFpsMeasurement;
    }
}