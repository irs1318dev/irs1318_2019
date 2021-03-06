package frc.robot.common.robotprovider;

import frc.robot.vision.VisionCalculations;

public interface IRobotProvider
{
    public IAnalogInput getAnalogInput(int channel);
    public ITalonSRX getTalonSRX(int deviceNumber);
    public IVictorSPX getVictorSPX(int deviceNumber);
    public ISparkMax getSparkMax(int deviceID, SparkMaxMotorType motorType);
    public ICompressor getCompressor();
    public ICompressor getCompressor(int module);
    public IDigitalInput getDigitalInput(int channel);
    public IDoubleSolenoid getDoubleSolenoid(int forwardChannel, int reverseChannel);
    public IDoubleSolenoid getDoubleSolenoid(int module, int forwardChannel, int reverseChannel);
    public IEncoder getEncoder(int channelA, int channelB);
    public IJoystick getJoystick(int port);
    public IMotor getTalon(int channel);
    public IMotor getVictor(int channel);
    public IServo getServo(int channel);
    public IPowerDistributionPanel getPDP();
    public IPowerDistributionPanel getPDP(int module);
    public IRelay getRelay(int channel);
    public IRelay getRelay(int channel, RelayDirection direction);
    public ISolenoid getSolenoid(int channel);
    public ISolenoid getSolenoid(int module, int channel);
    public INavx getNavx();
    public IVideoStream getMJPEGStream(String name, int width, int height);
    public IUsbCamera getUsbCamera(String name, int dev);
    public IDriverStation getDriverStation();
    public IOpenCVProvider getOpenCVProvider();
    public INetworkTableProvider getNetworkTableProvider();
    public VisionCalculations getVisionCalculations();
    public <V> ISendableChooser<V> getSendableChooser();
}
