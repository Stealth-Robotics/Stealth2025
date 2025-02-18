package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Superstructure;
import frc.robot.Superstructure.ScoringLevel;

public class Leds extends SubsystemBase {
    private final int LED_COUNT = 0;
    // TODO: Find CAN id
    private final CANdle candle = new CANdle(0);

    private final Color l1Color = new Color(166, 7, 28), // Crimson
            l2Color = new Color(234, 10, 142), // T-mobile pink
            l3Color = new Color(109, 170, 194), // Cora blue
            l4Color = new Color(0, 255, 0); // Blinding green

    Color currentColor;

    boolean blinking;

    public Leds() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.brightnessScalar = 0.30;
        config.disableWhenLOS = true;
        config.v5Enabled = true; // What does this mean?
        config.vBatOutputMode = VBatOutputMode.Off; // what?
        config.enableOptimizations = true; // What does this mean?
        config.statusLedOffWhenActive = false;

        candle.configAllSettings(config);
    }

    private void setRGB(int r, int g, int b) {
        currentColor = new Color(r, g, b);
        candle.clearAnimation(0);
        candle.setLEDs(r, g, b, 0, 0, LED_COUNT);
    }

    public Command setColor(int r, int g, int b) {
        return this.runOnce(() -> setRGB(r, g, b));
    }

    public Command setColor(Color color) {
        return this.runOnce(() -> setRGB((int) color.red, (int) color.green, (int) color.blue));
    }

    public Command setLevel(ScoringLevel level) {
        return this.runOnce(() -> {
            if (level == ScoringLevel.L1) {
                setColor(l1Color);
            } else if (level == ScoringLevel.L2) {
                setColor(l2Color);
            } else if (level == ScoringLevel.L3) {
                setColor(l3Color);
            } else {
                setColor(l4Color);
            }
        });
    }

    private void setBlinkingState(boolean state) {
        blinking = state;
        if (blinking) {
            animate(new StrobeAnimation((int) currentColor.red, (int) currentColor.green, (int) currentColor.blue, 0, 1,
                    LED_COUNT));
        }
    }

    // private void toggleBlinking(){
    // blinking = !blinking;
    // if(blinking){
    // animate(new
    // StrobeAnimation((int)currentColor.red,(int)currentColor.green,(int)currentColor.blue));
    // }
    // }
    public Command blink() {
        return new SequentialCommandGroup(
                this.run(() -> setBlinkingState(true)),
                new WaitCommand(2),
                this.run(() -> setBlinkingState(false)),
                setColor(currentColor));
    }

    private void animate(Animation animation) {
        candle.clearAnimation(0);
        candle.animate(animation);
    }

}
