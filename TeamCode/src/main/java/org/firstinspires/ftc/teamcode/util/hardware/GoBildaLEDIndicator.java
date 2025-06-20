package org.firstinspires.ftc.teamcode.util.hardware;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.enums.Levels;
import org.firstinspires.ftc.teamcode.util.hardware.Component;

import java.util.concurrent.TimeUnit;

public class GoBildaLEDIndicator extends Component {
    public Servo led;
    float color = 0;
    Animation currentAnimation = Animation.SOLID;

    private ElapsedTime elaspedTimeBlink = new ElapsedTime();
    private boolean blinkStat = false;
    private int colorAlt = 0;
    private int blips = 0;


    public GoBildaLEDIndicator(int port, String name, HardwareMap map) {
        super(port, name);
        led = map.get(Servo.class, name);
    }

    public void setColor(Colors c) {
        switch (c) {
            case OFF:
                color = 0;
                break;
            case RED:
                color = (float) 0.277;
                break;
            case ORANGE:
                color = (float) 0.333;
                break;
            case JOOS_ORANGE:
                color = (float) 0.334;
                break;
            case YELLOW:
                color = (float) 0.388;
                break;
            case SAGE:
                color = (float) 0.444;
                break;
            case GREEN:
                color = (float) 0.5;
                break;
            case AZURE:
                color = (float) 0.555;
                break;
            case BLUE:
                color = (float) 0.611;
                break;
            case INDIGO:
                color = (float) 0.666;
                break;
            case VIOLET:
                color = (float) 0.722;
                break;
            case WHITE:
                color = (float) 1;
                break;
        }
    }

    public void setColor(float pwmSignal) {
        color = pwmSignal;
    }

    public void setColor(int r, int g, int b) {
        float[] hsv = new float[3];
        Color.RGBToHSV(r, g, b, hsv);

        color = (float) ((hsv[0] / (275/0.445)) + 0.277);
    }

    public void setPreset(Levels level) {
        if (level == Levels.INIT) {
            setColor(Colors.JOOS_ORANGE);
            setAnimation(Animation.SOLID);
        } else if (level == Levels.INTAKE) {
            // abstract out
        } else if (level == Levels.INTAKE_INTERMEDIATE) {
            // abstract out
        } else if (level == Levels.INTERMEDIATE) {
            // abstract out
        } else if (level == Levels.LOW_BASKET) {
            setColor(Colors.GREEN);
            setAnimation(Animation.SLOW_BLINK);
        } else if (level == Levels.HIGH_BASKET) {
            setColor(Colors.GREEN);
            setAnimation(Animation.BLINK);
        } else if (level == Levels.HIGH_RUNG) {
            setColor(Colors.GREEN);
            setAnimation(Animation.BLINK);
        }
    }

    public void setAnimation(Animation a) {
        currentAnimation = a;
        if (a == Animation.THREE_BLIPS) blips = 0;
    }

    public void set(Colors color, Animation animation) {
        setAnimation(animation);
        setColor(color);
    }

    public void update() {
        if (currentAnimation == Animation.SOLID) {
            led.setPosition(color);
        } else if (currentAnimation == Animation.BLINK) {
            if (elaspedTimeBlink.time(TimeUnit.MILLISECONDS) >= 150) {
                if (blinkStat) {
                    led.setPosition(0);
                    blinkStat = !blinkStat;
                    elaspedTimeBlink.reset();
                } else {
                    led.setPosition(color);
                    blinkStat = !blinkStat;
                    elaspedTimeBlink.reset();
                }
            }
        } else if (currentAnimation == Animation.SLOW_BLINK) {
            if (elaspedTimeBlink.time(TimeUnit.MILLISECONDS) >= 500) {
                if (blinkStat) {
                    led.setPosition(0);
                    blinkStat = !blinkStat;
                    elaspedTimeBlink.reset();
                } else {
                    led.setPosition(color);
                    blinkStat = !blinkStat;
                    elaspedTimeBlink.reset();
                }
            }
        } else if (currentAnimation == Animation.OFFSET_SLOW_BLINK_RED) {
            if (elaspedTimeBlink.time(TimeUnit.MILLISECONDS) >= 500) {
               if (colorAlt == 0) {
                   led.setPosition(color);
                   colorAlt = 1;
                   elaspedTimeBlink.reset();
               } else if (colorAlt == 1) {
                   led.setPosition(0);
                   elaspedTimeBlink.reset();
                   colorAlt = 2;
               } else if (colorAlt == 2) {
                   led.setPosition(0.277);
                   colorAlt = 3;
                   elaspedTimeBlink.reset();
               } else {
                   led.setPosition(0);
                   colorAlt = 0;
                   elaspedTimeBlink.reset();
               }
            }
        } else if (currentAnimation == Animation.THREE_BLIPS) {
            if (blips < 4) {
                if (elaspedTimeBlink.time(TimeUnit.MILLISECONDS) >= 150) {
                    if (blinkStat) {
                        led.setPosition(0);
                        blinkStat = !blinkStat;
                        elaspedTimeBlink.reset();
                    } else {
                        led.setPosition(color);
                        blinkStat = !blinkStat;
                        elaspedTimeBlink.reset();
                        blips += 1;
                    }
                }
            } else {
                led.setPosition(0);
            }
        }
    }

    public enum Colors {
        OFF,
        RED,
        ORANGE,
        JOOS_ORANGE,
        YELLOW,
        SAGE,
        GREEN,
        AZURE,
        BLUE,
        INDIGO,
        VIOLET,
        WHITE
    }

    public enum Animation {
        SOLID,
        BLINK,
        SLOW_BLINK,
        OFFSET_SLOW_BLINK_RED,
        THREE_BLIPS
    }
}