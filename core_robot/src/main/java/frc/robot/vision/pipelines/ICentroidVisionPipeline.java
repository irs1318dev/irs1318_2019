package frc.robot.vision.pipelines;

import frc.robot.GamePiece;
import frc.robot.common.robotprovider.*;
import frc.robot.vision.common.VisionProcessingState;

public interface ICentroidVisionPipeline extends IVisionPipeline
{
    void setMode(VisionProcessingState state);
    void setGamePiece(GamePiece gamePiece);
    void setStreamMode(boolean isEnabled);
    boolean isActive();
    IPoint getCenter();
    Double getDesiredAngleX();
    Double getMeasuredAngleX();
    Double getRobotDistance();
    double getFps();
}
