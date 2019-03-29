package frc.robot.vision;

import frc.robot.common.robotprovider.IRotatedRect;

public class RectanglePair
{
    private final IRotatedRect left;
    private final IRotatedRect right;

    public RectanglePair(IRotatedRect left, IRotatedRect right)
    {
        this.left = left;
        this.right = right;
    }

    public IRotatedRect getLeft()
    {
        return this.left;
    }

    public IRotatedRect getRight()
    {
        return this.right;
    }

    public IRotatedRect getPreferredRect()
    {
        if (this.left != null)
        {
            return this.left;
        }

        return this.right;
    }
}
