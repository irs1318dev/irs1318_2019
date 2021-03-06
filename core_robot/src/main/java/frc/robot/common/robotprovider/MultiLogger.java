package frc.robot.common.robotprovider;

public class MultiLogger implements IDashboardLogger
{
    private final IDashboardLogger[] loggers;

    /**
     * Initializes a new instance of the MultiLogger class
     * @param loggers to log to
     */
    public MultiLogger(IDashboardLogger... loggers)
    {
        this.loggers = loggers;
    }

    /**
     * Write a boolean to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     */
    @Override
    public void logBoolean(String component, String key, boolean value)
    {
        for (IDashboardLogger logger : this.loggers)
        {
            logger.logBoolean(component, key, value);
        }
    }

    /**
     * Write a number (double) to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     */
    @Override
    public void logNumber(String component, String key, double value)
    {
        for (IDashboardLogger logger : this.loggers)
        {
            logger.logNumber(component, key, value);
        }
    }

    /**
     * Write a number (Double) to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     */
    @Override
    public void logNumber(String component, String key, Double value)
    {
        for (IDashboardLogger logger : this.loggers)
        {
            logger.logNumber(component, key, value);
        }
    }

    /**
     * Write a number (integer) to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     */
    @Override
    public void logInteger(String component, String key, int value)
    {
        for (IDashboardLogger logger : this.loggers)
        {
            logger.logInteger(component, key, value);
        }
    }

    /**
     * Write a number (integer) to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     * @param formatString to use
     */
    @Override
    public void logInteger(String component, String key, int value, String formatString)
    {
        for (IDashboardLogger logger : this.loggers)
        {
            logger.logInteger(component, key, value, formatString);
        }
    }

    /**
     * Write a point (x,y or N/A) to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     */
    @Override
    public void logPoint(String component, String key, IPoint value)
    {
        for (IDashboardLogger logger : this.loggers)
        {
            logger.logPoint(component, key, value);
        }
    }

    /**
     * Write a string to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     */
    @Override
    public void logString(String component, String key, String value)
    {
        for (IDashboardLogger logger : this.loggers)
        {
            logger.logString(component, key, value);
        }
    }

    /**
     * Flush the output stream, if appropriate..
     */
    @Override
    public void flush()
    {
        for (IDashboardLogger logger : this.loggers)
        {
            logger.flush();
        }
    }

    /**
     * Add a sendable chooser to the smart dashboard
     */
    @Override
    public <V> void addChooser(String name, ISendableChooser<V> chooser)
    {
        for (IDashboardLogger logger : this.loggers)
        {
            logger.addChooser(name, chooser);
        }
    }
}
