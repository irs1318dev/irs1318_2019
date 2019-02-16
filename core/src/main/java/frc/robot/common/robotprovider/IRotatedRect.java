package frc.robot.common.robotprovider;

public interface IRotatedRect
{
    ISize size();
    IPoint getCenter();
    double getAngle();
    void set(double[] vals);
    IRect boundingRect();
    IRotatedRect clone();
}