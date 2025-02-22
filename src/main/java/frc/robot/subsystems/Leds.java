package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;

import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer.LevelTarget;

public class Leds extends SubsystemBase {
    private final int LED_COUNT = 18;
    // TODO: Find CAN id
    private final CANdle candle = new CANdle(0);

    private final Color l1Color = new Color(166, 7, 28), // Crimson
            l2Color = new Color(234, 10, 142), // T-mobile pink
            l3Color = new Color(109, 170, 194), // Cora blue
            l4Color = new Color(0, 255, 0); // Blinding green

    private final int l1Red = 166, l1Green = 7, l1Blue = 28,
            l2Red = 234, l2Green = 10, l2Blue = 142,
            l3Red = 109, l3Green = 170, l3Blue = 194,
            l4Red = 0, l4Green = 255, l4Blue = 0;

    Color currentColor;

    private boolean isBlinking;

    private int currentR = 0, currentG = 0, currentB = 0;

    boolean blinking;

    public Leds() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.brightnessScalar = 0.2;
        config.disableWhenLOS = true;
        config.v5Enabled = true;
        config.vBatOutputMode = VBatOutputMode.On;
        config.enableOptimizations = true;
        config.statusLedOffWhenActive = false;
        config.stripType = LEDStripType.RGB;

        candle.configAllSettings(config);

    }

    private void setRGB(int r, int g, int b) {
        currentR = r;
        currentG = g;
        currentB = b;
    }

    // sets the current led colors based on target, perodic sets colors
    public void setLevel(LevelTarget level) {
        switch (level) {
            case L1:
                setRGB(l1Red, l1Green, l1Blue);
                break;
            case L2:
                setRGB(l2Red, l2Green, l2Blue);
                break;
            case L3:
                setRGB(l3Red, l3Green, l3Blue);
                break;

            default:
                setRGB(l4Red, l4Green, l4Blue);
                break;
        }
    }

    public Command rainbowAnim() {
        return this.runOnce(() -> candle.animate(new RainbowAnimation(1, 0.2, LED_COUNT)))
                .ignoringDisable(true);
    }

    public Command blink() {
        return this.runOnce(() -> {
            // set blinking var to true to override periodic if statement
            isBlinking = true;
            candle.clearAnimation(0);
            // make blinking animation
            candle.animate(new StrobeAnimation(currentR, currentG, currentB, 0, 0.2, LED_COUNT));
        }).andThen(
                // after 2 seconds, we clear the animation and set var to false, allowing
                // periodic to take over again
                new WaitCommand(2),
                Commands.runOnce(() -> {
                    candle.clearAnimation(0);
                    isBlinking = false;
                }));
    }

    @Override
    public void periodic() {
        // set rgb if we're not blinking
        if (!isBlinking) {
            candle.setLEDs(currentR, currentG, currentB);
        }
    }

}
