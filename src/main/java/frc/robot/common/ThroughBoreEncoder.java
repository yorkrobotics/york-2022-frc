package frc.robot.common;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;

public class ThroughBoreEncoder extends DutyCycleEncoder {

    private static final int MOVING_AVERAGE_FILTER_DEFAULT_TAPS = 5;

    private double lastValue, lastUpdateTime;

    private double velocityRaw, velocityFiltered;
    private LinearFilter velocityFilter;

    public ThroughBoreEncoder(int channel, int taps) {
        super(channel);
        velocityFilter = LinearFilter.movingAverage(taps);

        lastValue = this.get();
        lastUpdateTime = Timer.getFPGATimestamp();
    }

    public ThroughBoreEncoder(int channel) {
        this(channel, MOVING_AVERAGE_FILTER_DEFAULT_TAPS);
    }

    public void updateVelocity() {
        double currentTime = Timer.getFPGATimestamp();
        double currentValue = this.get();

        velocityRaw = (currentValue - lastValue) / (currentTime - lastUpdateTime);
        velocityFiltered = velocityFilter.calculate(velocityRaw);

        lastValue = currentValue;
        lastUpdateTime = currentTime;
    }

    public double getVelocityRaw() {
        return velocityRaw;
    }

    public double getVelocityFiltered() {
        return velocityFiltered;
    }
    
}
