package frc.robot.vision;

import frc.robot.GamePiece;
import frc.robot.common.robotprovider.*;
import frc.robot.vision.PixelsToInches;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.common.VisionProcessingState;

import java.util.*;

public class VisionCalculations
{


    public VisionCalculations() {
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

    public Set<IRotatedRect> pickRow(List<Set<IRotatedRect>> rows, GamePiece gamePiece, VisionProcessingState processingState)
    {
        if(rows.size() == 1){
            return rows.get(0);
        }
        if(GamePiece.Cargo.equals(gamePiece)
        && VisionProcessingState.ActiveRocket.equals(processingState))
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

    public double glideRadius(double azimuth, double distance)
    {
        double radius = (Math.pow((distance * 
                                Math.cos(azimuth) 
                                - VisionConstants.DOCKING_CAMERA_MOUNTING_DISTANCE),2) 
                        + Math.pow((distance * Math.sin(azimuth)
                                - VisionConstants.DOCKING_CAMERA_HORIZONTAL_MOUNTING_OFFSET), 2))
                        /(2*distance*Math.sin(azimuth)
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