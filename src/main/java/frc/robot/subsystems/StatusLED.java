package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDconstants;
import frc.robot.LEDColor;

public class StatusLED extends SubsystemBase {
    final AddressableLED led = new AddressableLED(LEDconstants.ledPort);
    final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDconstants.ledLength);

    final Timer timer = new Timer();

    boolean isOn = false;

    public StatusLED() {
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    // It doesnt work
    public Command interpolateStripColor(LEDColor a, LEDColor b) {
        return new FunctionalCommand(
            () -> { // init
                setStripColorHSV(a);
                timer.reset();
                timer.start();
                isOn = true;
            }, 
            () -> { // exec
                LEDColor lerpColor = a;
                if (isOn) {
                    lerpColor = LEDColor.lerpRGB(a, b, timer.get());
                    if (!lerpColor.equals(b)) {
                        isOn = false;
                        timer.restart();
                    }
                } else {
                    lerpColor = LEDColor.lerpRGB(b, a, timer.get());
                    if (lerpColor.equals(a)) {
                        isOn = true;
                        timer.restart();
                    }
                }
                setStripColorHSV(lerpColor);
                SmartDashboard.putBoolean("isOn", isOn);
            }, 
            isFinished -> timer.stop(), // end
            () -> false, // isFinished
            this
        );
    }
    /*
     * Accepts two colors 
     * Color A is bottom
     * Color B is top
     */
    public Command setHalfSolidColor(LEDColor colorA, LEDColor colorB) {
        return run(() -> setPartsOfStripColors(colorA, colorB));
    }

    public Command setSolidColor(LEDColor color) {
        return run(() -> setStripColorHSV(color));
    }

    public Command breathStripColor(LEDColor color, double speed) {
        return new FunctionalCommand(
            () -> { // init
                setStripColorHSV(color);
                isOn = true;
            },
            () -> { // exec
                LEDColor hsvColor = color;
                if (isOn){
                    hsvColor.value = hsvColor.value > 0 ? hsvColor.value -= speed * 2.55 : 0;
                    hsvColor.setHSV(hsvColor.hue, hsvColor.saturation, hsvColor.value);
                    isOn = !(hsvColor.value == 0);
                } else {
                    hsvColor.value = hsvColor.value < 255 ? hsvColor.value += speed * 2.55 : 255;
                    hsvColor.setHSV(hsvColor.hue, hsvColor.saturation, hsvColor.value);
                    isOn = hsvColor.value == 255;
                }

                setStripColorHSV(hsvColor);
            },
            isFinished -> {}, // end
            () -> false, // isFinished
            this
        );
    }

    public Command raceColorsUpStrip(LEDColor raceColor, LEDColor backColor){
        LEDColor[] colorArray = makeRaceArray(backColor, raceColor);
        return new FunctionalCommand(
            () -> {}, 
            null, 
            isFinished -> {}, 
            () -> false, 
            this
        );
    }

    public Command staggeredBlinkStripColor(LEDColor color1, LEDColor color2, double blinkInterval, double delay, int blinkNum) {
        var counterWrapper = new Object() { int counter = 0; };
        return new FunctionalCommand(
            () -> { // init
                setStripColorHSV(color1);
                timer.reset();
                timer.start();
                isOn = false;
            },
            () -> { // exec
                if (counterWrapper.counter % blinkNum == 0) {
                    isOn = false;
                    timer.restart();
                }

                isOn = timer.hasElapsed(blinkInterval);
                if (isOn && timer.hasElapsed(blinkInterval*2)) {
                    timer.restart();
                    counterWrapper.counter++;
                }

                if (isOn) {
                    setStripColorHSV(color2);
                    return;
                }

                setStripColorHSV(color1);
            },
            isFinished -> timer.stop(), // end
            () -> false, // isFinished
            this
        );
    }

    public Command blinkStripColor(LEDColor color1, LEDColor color2, double interval) {
        return new FunctionalCommand(
            () -> { // init
                setStripColorHSV(color1);
                timer.reset();
                timer.start();
                isOn = false;
            }, 
            () -> { // exec
                isOn = timer.hasElapsed(interval);
                if (isOn && timer.hasElapsed(interval*2)) {
                    timer.restart();
                }

                if (isOn)
                    setStripColorHSV(color2);
                else 
                    setStripColorHSV(color1);
            }, 
            isFinished -> timer.stop(), // end 
            () -> false, // isFinished
            this
        );
    }

    LEDColor[] makeRaceArray(LEDColor backColor, LEDColor raceColor) {
        LEDColor[] colorArray = new LEDColor[ledBuffer.getLength()];
        for(var i = LEDconstants.ledChaserLength; i < (ledBuffer.getLength() - LEDconstants.ledChaserLength); i++) {
            colorArray[i] = backColor;
        }

        for(var i = 0; i < LEDconstants.ledChaserLength; i++) {
            colorArray[i] = raceColor;
        }
        
        return colorArray;
    }

    void setPartsOfStripColors(LEDColor... colors) {
        for (int i = 0; i < colors.length; i++) {
            setPartOfStripColor(colors[i], i, colors.length);
        }
    }

    void setPartOfStripColor(LEDColor color, int part, int length){
        for (int i = (int)((ledBuffer.getLength() / length) * part); i < (int)((ledBuffer.getLength() / length) * (part + 1)); i++) {
            setHSV(i, color);
        }
        led.setData(ledBuffer);
    }
    
    void setHSV(int index, LEDColor color) {
        ledBuffer.setHSV(index, (int)color.hue, (int)color.saturation, (int)color.value);
    }

    void setStripColorHSV(LEDColor color) {
        for (int i=0; i < ledBuffer.getLength(); i++) {
            setHSV(i, color);
        }
        led.setData(ledBuffer);
    }

    void setRGB(int index, Color color) {
        ledBuffer.setRGB(index, (int)color.red, (int)color.green, (int)color.blue);
    }

    void setStripColorRGB(Color color) {
        for (int i=0; i < ledBuffer.getLength(); i++) {
            setRGB(i, color);
        }
        led.setData(ledBuffer);
    }
}
