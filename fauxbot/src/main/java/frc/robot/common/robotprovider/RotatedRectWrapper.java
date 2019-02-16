package frc.robot.common.robotprovider;

import org.opencv.core.RotatedRect;

public class RotatedRectWrapper implements IRotatedRect
{
    final RotatedRect wrappedObject;

    public RotatedRectWrapper(RotatedRect wrappedObject)
    {
        this.wrappedObject = wrappedObject;
    }

    @Override
    public IPoint getCenter()
    {
        return new PointWrapper(this.wrappedObject.center);
    }

    @Override
    public ISize size()
    {
        return new SizeWrapper(this.wrappedObject.size);
    }

    @Override
    public double getAngle()
    {
        return this.wrappedObject.angle;
    }

    @Override
    public void set(double[] vals)
    {
        this.wrappedObject.set(vals);
    }

    @Override
    public IRect boundingRect()
    {
        return new RectWrapper(this.wrappedObject.boundingRect());
    }

	@Override
    public IRotatedRect clone()
    {
        return new RotatedRectWrapper(this.wrappedObject.clone());
    }
}