package org.firstinspires.ftc.teamcode.util.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayDeque;

@Config
public class BrushlandColorSensor extends Component {
    DigitalChannel pin0;
    DigitalChannel pin1;
    AnalogInput pin0a;
    private double filteredVoltage = 0;
    public static double alpha = 0.6;  // adjust for desired responsiveness
    public BrushlandColorSensor(int port, String name, HardwareMap map) {
        super(port, name);
        pin0 = map.digitalChannel.get(name + "p0");
        pin1 = map.digitalChannel.get(name + "p1");
    }

    public BrushlandColorSensor(int port, String name, HardwareMap map, boolean analog) {
        super(port, name);
        pin0a = map.analogInput.get(name + "p0");
        pin1 = map.digitalChannel.get(name + "p1");
    }

    public boolean getPin0Digital() {
        return pin0.getState();
    }
    public double getPin0Analog() {
        double raw = pin0a.getVoltage() / 3.3 * 100;
        filteredVoltage = alpha * raw + (1 - alpha) * filteredVoltage;
        return filteredVoltage;
    }

    private long belowThresholdStartTime = -1;
    private boolean debouncedBelow = false;

    public boolean isDebouncedBelowThreshold(double threshold, long minDurationMs) {
        double value = getPin0Analog();  // filtered version

        long now = System.currentTimeMillis();
        if (value < threshold) {
            if (belowThresholdStartTime == -1) {
                belowThresholdStartTime = now;
            } else if (now - belowThresholdStartTime >= minDurationMs) {
                debouncedBelow = true;
            }
        } else {
            belowThresholdStartTime = -1;
            debouncedBelow = false;
        }

        return debouncedBelow;
    }

    private static class TimedReading {
        long timestamp;
        boolean below;
        TimedReading(long t, boolean b) { timestamp = t; below = b; }
    }

    private ArrayDeque<TimedReading> signalWindow = new ArrayDeque<>();
    public static long windowDurationMs = 40;
    public static double belowThresholdRatio = 0.7;

    public boolean rollingWindowBelow(double threshold) {
        long now = System.currentTimeMillis();
        boolean isBelow = getPin0Analog() < threshold;
        signalWindow.addLast(new TimedReading(now, isBelow));

        // Remove old entries outside the window
        while (!signalWindow.isEmpty() && now - signalWindow.peekFirst().timestamp > windowDurationMs) {
            signalWindow.pollFirst();
        }

        // Count how many readings are below threshold
        int total = signalWindow.size();
        int belowCount = 0;
        for (TimedReading r : signalWindow) {
            if (r.below) belowCount++;
        }

        return total >= 3 && ((double) belowCount / total) >= belowThresholdRatio;
    }

    public boolean getPin1() {
        return pin1.getState();
    }

    public boolean getBoth() {
        return getPin0Digital() && getPin1();
    }

    public boolean getEither() {
        return getPin0Digital() || getPin1();
    }

    public boolean getNeither() {
        return !getPin0Digital() && !getPin1();
    }

    public boolean onlyPin0() {
        return getPin0Digital() && !getPin1();
    }

    public boolean onlyPin1() {
        return getPin1() && !getPin0Digital();
    }
}