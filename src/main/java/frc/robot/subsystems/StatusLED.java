package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

    /**
     * Linearly interpolates back and forth between two colors, at a set speed
     * @param a First color to interpolate between
     * @param b Second color to interpolate between
     * @param speed Speed at which to interpolate, in percentage (0.0-1.0)
     * @return
     */
    public Command interpolateStripColor(LEDColor a, LEDColor b, double speed) {
        return new FunctionalCommand(
            () -> { // init
                setStripColorHSV(a);
                timer.reset();
                timer.start();
                isOn = true;
            }, 
            () -> { // exec
                LEDColor lerpColor;
                if (isOn) {
                    lerpColor = LEDColor.lerpRGB(a, b, timer.get() * speed);
                    if (lerpColor.equals(b)) {
                        isOn = false;
                        timer.restart();
                    }
                    
                    setStripColorHSV(lerpColor);
                    return;
                }

                lerpColor = LEDColor.lerpRGB(b, a, timer.get() * speed);
                if (lerpColor.equals(a)) {
                    isOn = true;
                    timer.restart();
                }
                setStripColorHSV(lerpColor);
            }, 
            isFinished -> timer.stop(), // end
            () -> false, // isFinished
            this
        );
    }
 
    /**
     * Sets half of the LED strip to colorB, and the other half to colorA
     * @param colorA Bottom half color
     * @param colorB Top half color
     * @return Command to constantly set the strip
     * @see LEDColor
     * @see RunCommand
     */
    public Command setHalfSolidColor(LEDColor colorA, LEDColor colorB) {
        return run(() -> setPartsOfStripColors(colorA, colorB));
    }

    /**
     * Sets the whole LED strip to a color
     * @param color color to set the strip to
     * @return Command to constantly set the strip
     * @see LEDColor
     * @see RunCommand
     */
    public Command setSolidColor(LEDColor color) {
        return run(() -> setStripColorHSV(color));
    }

    /**
     * Pulsates a color across the whole strip, to give it a 'breathing' motion
     * @param color Color to pulsate
     * @param speed Speed at which the strip pulsates, arbitrary units
     * @return Command to set the strip color
     * @see LEDColor
     * @see FunctionalCommand
     */
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

    /**
     * 'Races' a color up the strip by continously setting an incrementing buffer of LEDs to a color
     * @param raceColor Color to race up the strip
     * @param backColor Color of the background; the non-racing color
     * @param interval Interval of time between updates, in seconds
     * @return Command to set the strip
     * @see LEDColor
     * @see FunctionalCommand
     */
    public Command raceColorsUpStrip(LEDColor raceColor, LEDColor backColor, double interval){
        LEDColor[] colorArray = makeRaceArray(backColor, raceColor);
        return new FunctionalCommand(
            () -> { // init
                setStripColorHSV(backColor);
                timer.reset();
                timer.start();
            }, 
            () -> { // exec
                for (int i = 0; i < colorArray.length;) {
                    if (timer.advanceIfElapsed(interval))
                        i++;

                    setHSV(i, colorArray[i]);
                }
            }, 
            isFinished -> timer.stop(), // end 
            () -> false, // isFinished
            this
        );
    }

    /**
     * Blinks colors across the strip in a 'staggered' motion, blinking a number of times before stopping for a delay.
     * @param color1 First color to blink between
     * @param color2 Second color to blink between
     * @param blinkInterval Interval of time between blinks, in seconds
     * @param delay Delay between sets of blinks, in seconds
     * @param blinkNum Number of blinks in a set 
     * @return Command to set the strip
     * @see LEDColor
     * @see FunctionalCommand
     */
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

                isOn = timer.advanceIfElapsed(blinkInterval);
                if (isOn && timer.advanceIfElapsed(blinkInterval*2))
                    counterWrapper.counter++;

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

    /**
     * Blinks the LED strip between two colors
     * @param color1 First color to blink between
     * @param color2 Second color to blink between
     * @param delay Delay between blinks, in seconds
     * @return Command to set the strip
     * @see LEDColor
     * @see FunctionalCommand
     */
    public Command blinkStripColor(LEDColor color1, LEDColor color2, double delay) {
        return new FunctionalCommand(
            () -> { // init
                setStripColorHSV(color1);
                timer.reset();
                timer.start();
                isOn = false;
            }, 
            () -> { // exec
                isOn = timer.hasElapsed(delay);
                if (isOn && timer.hasElapsed(delay*2)) {
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
        for(int i = LEDconstants.ledChaserLength; i < (ledBuffer.getLength() - LEDconstants.ledChaserLength); i++)
            colorArray[i] = backColor;

        for(int i = 0; i < LEDconstants.ledChaserLength; i++)
            colorArray[i] = raceColor;
        
        return colorArray;
    }

    void setPartsOfStripColors(LEDColor... colors) {
        for (int i = 0; i < colors.length; i++)
            setPartOfStripColor(colors[i], i, colors.length);
    }

    void setPartOfStripColor(LEDColor color, int part, int length){
        for (int i = (int)((ledBuffer.getLength() / length) * part); i < (int)((ledBuffer.getLength() / length) * (part + 1)); i++) 
            setHSV(i, color);
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
