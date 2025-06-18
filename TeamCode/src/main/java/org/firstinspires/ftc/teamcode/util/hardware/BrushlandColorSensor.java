package org.firstinspires.ftc.teamcode.util.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class BrushlandColorSensor extends Component {
    DigitalChannel pin0;
    DigitalChannel pin1;
    AnalogInput pin0a;
    private double filteredVoltage = 0;
    public static double alpha = 0.2;  // adjust for desired responsiveness
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