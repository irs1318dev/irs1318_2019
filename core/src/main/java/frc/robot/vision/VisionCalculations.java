package frc.robot.vision;

import frc.robot.GamePiece;
import frc.robot.common.robotprovider.*;
import frc.robot.vision.common.VisionProcessingState;
import frc.robot.vision.common.VisionResult;

import java.util.*;

public class VisionCalculations {

    private static double EPS = 0.01;

    public VisionCalculations() {
    }

    public List<Set<IRotatedRect>> groupRotatedRect(List<IRotatedRect> rotatedRect)
    {
        List<Set<IRotatedRect>> rows = new ArrayList<Set<IRotatedRect>>();
        for (IRotatedRect rect : rotatedRect)
        {
            boolean added = false;

            // skip malformed rectangles
            // reject rectangles that are malformed compared to the expected aspect ration of 5.5/2.0
            if (rect.getAspectRatio() > 5.0 || rect.getAspectRatio() < 1.0)
            {
                continue;
            }

            // put the rectangle in a row
            for (Set<IRotatedRect> row : rows)
            {
                IRotatedRect compare = row.stream().findFirst().get();
                if (Math.abs(rect.getCenter().getY() - compare.getCenter().getY()) < 30)
                {
                    row.add(rect);
                    added = true;
                    break;
                }
            }

            if (added)
            {
                continue;
            }

            // no row exists, create a new row
            Set<IRotatedRect> row = new HashSet<IRotatedRect>();
            row.add(rect);
            rows.add(row);
        }

        return rows;
    }

    public VisionResult determineVisionResult(GamePiece gamePiece, VisionProcessingState processingState)
    {
        if (GamePiece.Cargo.equals(gamePiece) &&
            VisionProcessingState.ActiveRocket.equals(processingState))
        {
            return VisionResult.HIGH_TARGET;
        }

        return VisionResult.LOW_TARGET;
    }

    public Set<IRotatedRect> pickRow(List<Set<IRotatedRect>> rows, VisionResult visionResult)
    {
        if (rows.size() == 1)
        {
            return rows.get(0);
        }

        if (VisionResult.HIGH_TARGET.equals(visionResult))
        {
            double lowestY = Double.MAX_VALUE;
            Set<IRotatedRect> lowestRow = null;
            for (Set<IRotatedRect> row : rows)
            {
                IRotatedRect rect = row.stream().findFirst().get();
                if (rect.getCenter().getY() < lowestY)
                {
                    lowestY = rect.getCenter().getY();
                    lowestRow = row;
                }
            }

            return lowestRow;
        }
        else
        {
            double highestY = Double.MIN_VALUE;
            Set<IRotatedRect> highestRow = null;
            for (Set<IRotatedRect> row : rows)
            {
                IRotatedRect rect = row.stream().findFirst().get();
                if (rect.getCenter().getY() > highestY)
                {
                    highestY = rect.getCenter().getY();
                    highestRow = row;
                }
            }

            return highestRow;
        }
    }

    boolean isLeft(IRotatedRect rect)
    {
        double rectAngle = rect.getAngle();
        if (rectAngle < -45.0 && rectAngle > -90.0)
        {
            return true;
        }

        return false;
    }

    boolean isRight(IRotatedRect rect)
    {
        double rectAngle = rect.getAngle();
        if (rectAngle < 0.0 && rectAngle > -44.0)
        {
            return true;
        }

        return false;
    }

    List<IRotatedRect> largestRect(List<IRotatedRect> rect)
    {
        double area = 0;
        List<IRotatedRect> pair = new ArrayList<IRotatedRect>();
        IRotatedRect left = null;
        IRotatedRect right = null;
        for (int i = 0; i < rect.size(); i++)
        {
            if (isLeft(rect.get(i)))
            {
                if (area < (rect.get(i).getSize().getHeight() * rect.get(i).getSize().getWidth()))
                {
                    area = (rect.get(i).getSize().getHeight() * rect.get(i).getSize().getWidth());
                    left = rect.get(i);
                    right = rect.get(i + 1);
                    if (pair.isEmpty())
                    {
                        pair.add(left);
                        pair.add(right);
                    }
                    else
                    {
                        pair.set(0, left);
                        pair.set(1, right);
                    }
                }
            }
        }

        return pair;
    }

    public List<IRotatedRect> pickPairedRect(Set<IRotatedRect> rects) {
        List<IRotatedRect> pairedRect = new ArrayList<>();
        List<IRotatedRect> rectList = sortByCenterX(rects);

        for (int i = 0; i < rectList.size(); i++)
        {
            if (isLeft(rectList.get(i)) && i + 1 < rectList.size())
            {
                pairedRect.add(rectList.get(i));
                pairedRect.add(rectList.get(i + 1));
            }
        }

        return largestRect(pairedRect);
    }

    public List<IRotatedRect> sortByCenterX(Collection<IRotatedRect> rects) {
        List<IRotatedRect> rectList = new ArrayList<>();
        rectList.addAll(rects);
        Collections.sort(rectList,
                Comparator.comparingDouble(arg0 -> arg0.getCenter().getX()));
        return rectList;

    }


    public int findInterval(double avgPixelValue, List<PixelsToInches> interpolateList) {
        for (int i = 0; i < interpolateList.size(); i++) {
            PixelsToInches pixelsToInches = VisionConstants.PIXELS_TO_INCHES.get(i);
            if (avgPixelValue > pixelsToInches.getPixels())
            {
                return i - 1;
            }
        }

        return 0;
    }

    /**
     * @param rects
     * @return the average of length of the target rectangles
     */
    public double computeAvgPixel(List<IRotatedRect> rects)
    {
        if (rects.size() != 2)
        {
            return -1.0;
        }

        IRotatedRect left = rects.get(0);
        IRotatedRect right = rects.get(1);
        return ((left.getSize().getWidth() + right.getSize().getHeight()) / 2.0);
    }

    public double interpolateInchesFromPixels(double avgPixel, int interval, List<PixelsToInches> interpolateList)
    {
        PixelsToInches low = interpolateList.get(interval);
        PixelsToInches high = interpolateList.get(interval + 1);
        double slope = (low.getInches() - high.getInches()) / (low.getPixels() - high.getPixels());
        double inches = slope * (avgPixel - low.getPixels()) + low.getInches();
        return inches;
    }

    public double azimuth(double avgPixels, List<IRotatedRect> rects)
    {
        if (rects.size() != 2)
        {
            return -1000.0;
        }
        if (Math.abs(avgPixels) < EPS) {
            return -2000.0;
        }

        IRotatedRect left = rects.get(0);
        IRotatedRect right = rects.get(1);

        double inchesPerPixels = VisionConstants.DOCKING_RETROREFLECTIVE_TAPE_HEIGHT_STRAIGHT / avgPixels;
        double observedPixels = (right.getCenter().getX() - left.getCenter().getX());
        double distanceBetweenCenters = VisionConstants.DOCKING_DISTANCE_BETWEEN_TAPE_TARGETS / inchesPerPixels;

        if (VisionConstants.DEBUG &&
            VisionConstants.DEBUG_PRINT_OUTPUT &&
            VisionConstants.DEBUG_PRINT_ANALYZER_DATA)
        {
            System.out.println(String.format("o=%f, d=%f", observedPixels, distanceBetweenCenters));
        }

        double azimuth = 0.0;
        // only look up angle if it is not close to straight on
        if ((Math.abs(observedPixels - distanceBetweenCenters)/distanceBetweenCenters) > EPS) {
            azimuth =
                    calculateAzimuthSign(left, right) *
                    Math.acos(observedPixels / distanceBetweenCenters);
        }

            if (VisionConstants.DEBUG &&
                    VisionConstants.DEBUG_PRINT_OUTPUT &&
                    VisionConstants.DEBUG_PRINT_ANALYZER_DATA) {
                System.out.println(String.format("a=%f, adeg=%f", azimuth, azimuth * 180 / Math.PI));
            }
        return azimuth;
    }

    public double calculateAzimuthSign(IRotatedRect left, IRotatedRect right) {
        // when looking at te targets, the further target is higher in the frame, (lower y)
        // we can use this to determine the direction, because the further tape will be higher in frame
        if (left.getCenter().getY() - right.getCenter().getY() < -EPS) {
            return -1.0;
        }
        return 1.0;
    }

    public double calculateIntitialApproachTurnAngle(double azimuth, double pixelsPerInch, List<IRotatedRect> rects, double distance) {
        if (rects.size() != 2) {
            return -1000.0;
        }

        IRotatedRect left = rects.get(0);
        IRotatedRect right = rects.get(1);

        //In radians
        double targetCenter = VisionConstants.LIFECAM_CAMERA_RESOLUTION_X
                + pixelsPerInch * VisionConstants.DOCKING_CAMERA_HORIZONTAL_MOUNTING_OFFSET
                * Math.cos(azimuth);

        double observedTargetCenter = left.getCenter().getX()
                + (right.getCenter().getX() - left.getCenter().getX()) / 2.0;
        double initialTurnAngle = (Math.atan(targetCenter - observedTargetCenter)) / distance;
        return initialTurnAngle;
    }

    public double calculateTravelDistance(double azimuth, double distance)
    {
        double travelDistance =
            Math.sqrt(
                Math.pow(
                        (distance * Math.sin(azimuth)
                                - VisionConstants.DOCKING_CAMERA_HORIZONTAL_MOUNTING_OFFSET), 2)
                        + Math.pow((distance * Math.cos(azimuth)), 2));

        return travelDistance;
    }

    public double finalApproachTurn(double azimuth, double distance)
    {
        //24 is fill-in for docking offset
        double approachAngle = Math.atan2(distance * Math.sin(azimuth) - 24, distance * Math.cos(azimuth));
        //convert to radians!!!!!
        return Math.PI / 2 - approachAngle;
    }

    public double glideRadius(double azimuth, double distance)
    {
        double radius = (Math.pow((distance *
                Math.cos(azimuth)
                - VisionConstants.DOCKING_CAMERA_MOUNTING_DISTANCE), 2)
                + Math.pow((distance * Math.sin(azimuth)
                - VisionConstants.DOCKING_CAMERA_HORIZONTAL_MOUNTING_OFFSET), 2))
                / (2 * distance * Math.sin(azimuth)
                + VisionConstants.DOCKING_CAMERA_HORIZONTAL_MOUNTING_OFFSET);

        return radius;
    }

    public double findSweepAngle(double distance, double azimuth, double radius)
    {
        double angle = (Math.atan(distance * Math.cos(azimuth)
                - VisionConstants.DOCKING_CAMERA_MOUNTING_DISTANCE))
                / (radius - (distance * Math.sin(azimuth)
                + VisionConstants.DOCKING_CAMERA_HORIZONTAL_MOUNTING_OFFSET));

        return angle;
    }

    public double findGlideDistance(double radius, double angle)
    {
        return radius * angle;
    }
}