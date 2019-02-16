package frc.robot.common.robotprovider;

public interface IRotatedRect
{
    double getX();
    double getY();
    ISize size();
    IPoint getCenter();
    double getAngle();
    double getHeight();
    void set(double[] vals);
    IRect boundingRect();
    IRotatedRect clone();
}