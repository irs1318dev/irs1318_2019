package frc.robot.common.robotprovider;

public interface IRotatedRect
{
    double getX();
    double getY();
    ISize size();
    double getAngle();
    void set(double[] vals);
    IRect boundingRect();
    IRotatedRect clone();
}