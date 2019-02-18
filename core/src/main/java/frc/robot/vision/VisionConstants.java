package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;


public class VisionConstants
{
    // Debug output settings:
    public static final boolean DEBUG = true;
    public static final boolean DEBUG_PRINT_OUTPUT = false;
    public static final boolean DEBUG_PRINT_ANALYZER_DATA = false;
    public static final int DEBUG_FPS_AVERAGING_INTERVAL = 25;
    public static final boolean DEBUG_OUTPUT_FRAMES = true;
    public static final boolean DEBUG_SAVE_FRAMES = false;
    public static final int DEBUG_FRAME_OUTPUT_GAP = 20; // the number of frames to wait between saving debug image output
    public static final String DEBUG_OUTPUT_FOLDER = "/home/lvuser/vision/";

	public static final double RING_LIGHT_OFF = 0.0;
	public static final double RING_LIGHT_ON = 1.0;

    // Conversion constants...
    public static final double ANGLE_TO_RADIANS = (Math.PI / 180.0f);
    public static final double RADIANS_TO_ANGLE = (180.0f / Math.PI);

    // Settings for AXIS IP-based camera
    public static final String AXIS_CAMERA_IP_ADDRESS = "10.13.18.11";
    public static final String AXIS_CAMERA_USERNAME_PASSWORD = "root:1318";
    public static final int AXIS_CAMERA_RESOLUTION_X = 320;
    public static final int AXIS_CAMERA_RESOLUTION_Y = 240;
    public static final double AXIS_CAMERA_ANGLE_OF_VIEW = 50.0; // note that documentation says 47 degrees, so we'll have to see whether this is accurate enough.
    public static final int AXIS_CAMERA_CENTER_WIDTH = VisionConstants.AXIS_CAMERA_RESOLUTION_X / 2; // distance from center to left/right sides in pixels
    public static final int AXIS_CAMERA_CENTER_HEIGHT = VisionConstants.AXIS_CAMERA_RESOLUTION_Y / 2; // distance from center to top/bottom in pixels
    public static final double AXIS_CAMERA_CENTER_VIEW_ANGLE = VisionConstants.AXIS_CAMERA_ANGLE_OF_VIEW / 2.0;

    // Settings for Microsoft LifeCam HD-3000 USB-based camera
    public static final int LIFECAM_CAMERA_RESOLUTION_X = 320;
    public static final int LIFECAM_CAMERA_RESOLUTION_Y = 240;
    public static final double LIFECAM_CAMERA_CENTER_WIDTH = VisionConstants.LIFECAM_CAMERA_RESOLUTION_X / 2.0 - 0.5; // distance from center to left/right sides in pixels
    public static final double LIFECAM_CAMERA_CENTER_HEIGHT = VisionConstants.LIFECAM_CAMERA_RESOLUTION_Y / 2.0 - 0.5; // distance from center to top/bottom in pixels
    public static final double LIFECAM_CAMERA_FIELD_OF_VIEW_X = 48.4; // 4:3 field of view along x axis. note that documentation says 68.5 degrees diagonal (at 16:9), so this is an estimate.
    public static final double LIFECAM_CAMERA_FIELD_OF_VIEW_Y = 36.3; // 4:3 field of view along y axis
    public static final double LIFECAM_CAMERA_FIELD_OF_VIEW_X_RADIANS = VisionConstants.LIFECAM_CAMERA_FIELD_OF_VIEW_X
        * VisionConstants.ANGLE_TO_RADIANS;
    public static final double LIFECAM_CAMERA_FIELD_OF_VIEW_Y_RADIANS = VisionConstants.LIFECAM_CAMERA_FIELD_OF_VIEW_Y
        * VisionConstants.ANGLE_TO_RADIANS;
    public static final double LIFECAM_CAMERA_CENTER_VIEW_ANGLE = VisionConstants.LIFECAM_CAMERA_FIELD_OF_VIEW_X / 2.0;
    public static final double LIFECAM_CAMERA_FOCAL_LENGTH_X = 356.016; // focal_length = res_* / (2.0 * tan (FOV_* / 2.0)
    public static final double LIFECAM_CAMERA_FOCAL_LENGTH_Y = 366.058; // focal_length = res_* / (2.0 * tan (FOV_* / 2.0)
    public static final int LIFECAM_CAMERA_VISION_EXPOSURE = 1;
    public static final int LIFECAM_CAMERA_VISION_BRIGHTNESS = 1;
    public static final int LIFECAM_CAMERA_OPERATOR_BRIGHTNESS = 35;
    public static final int LIFECAM_CAMERA_FPS = 20; // Max supported value is 30

    // Undistort constants
    public static final boolean SHOULD_UNDISTORT = false;

    // HSV Filtering constants
    public static final int LIFECAM_HSV_FILTER_LOW_V0 = 52;
    public static final int LIFECAM_HSV_FILTER_LOW_V1 = 150;
    public static final int LIFECAM_HSV_FILTER_LOW_V2 = 100;
    public static final int LIFECAM_HSV_FILTER_HIGH_V0 = 95;
    public static final int LIFECAM_HSV_FILTER_HIGH_V1 = 255;
    public static final int LIFECAM_HSV_FILTER_HIGH_V2 = 255;

    // Contour filtering constants
    public static final double CONTOUR_MIN_AREA = 0.0;

    //Real measurements
    public static final double DOCKING_RETROREFLECTIVE_TAPE_HEIGHT = 5.875; // 5.5 inches tall
    public static final double DOCKING_RETROREFLECTIVE_TAPE_WIDTH = 3.5; // 2 inches wide
    public static final double DOCKING_RETROREFLECTIVE_TAPE_HxW_RATIO = -1.679; // height-to-width ratio
    public static final double DOCKING_RETROREFLECTIVE_TAPE_RATIO_RANGE = -10.0; // change? --> allowable height-to-width ratio range
    public static final double DOCKING_CONTOUR_ALLOWABLE_RATIO = -10.0; // change? --> the ratio of the second-largest contour to the largest
    public static final double DOCKING_CAMERA_MOUNTING_DISTANCE = 19.5; // change? --> (Y) distance from the front of the robot to the viewport of the camera
    public static final double DOCKING_CAMERA_HORIZONTAL_MOUNTING_OFFSET = 8.0; // change? --> (X) distance from the center line of the robot to the viewport of the camera
    public static final double DOCKING_CAMERA_HORIZONTAL_MOUNTING_ANGLE = -20.0; // change? --> (Y) degrees camera is mounted from level

    public static final double ROCKET_TO_GROUND_TAPE_HEIGHT = 28.5875; // (Z) distance from floor to center of tape
    public static final double DOCKING_CAMERA_MOUNTING_HEIGHT = 42.5; // (Z) distance from floor to the viewport of the camera
    public static final double DOCKING_TAPE_OFFSET = 5.7065; // horizontal offset from center of the two tape strips to the center of one of the pieces of tape


    public static final List<PixelsToInches> PIXELS_TO_INCHES = new ArrayList<>() {{
        add(new PixelsToInches(60, 0));
        add(new PixelsToInches(40, 24.0));
        add(new PixelsToInches(29, 42.0));
        add(new PixelsToInches(10, 100));
    }};
}
