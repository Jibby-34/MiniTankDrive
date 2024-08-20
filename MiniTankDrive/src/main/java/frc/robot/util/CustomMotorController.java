package frc.robot.util;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.DigitalOutput;

public class CustomMotorController {
    private PWM pwm;
    private DigitalOutput in1;
    private DigitalOutput in2;

    public CustomMotorController(int pwmChannel, int in1Channel, int in2Channel) {
        pwm = new PWM(pwmChannel);
        in1 = new DigitalOutput(in1Channel);
        in2 = new DigitalOutput(in2Channel);
    }

    public void set(double speed) {
        // Set the speed (0 to 1 for forward, -1 to 0 for reverse)
        pwm.setSpeed(Math.abs(speed));

        // Set the direction
        if (speed > 0) {
            in1.set(true);
            in2.set(false);
        } else if (speed < 0) {
            in1.set(false);
            in2.set(true);
        } else {
            in1.set(false);
            in2.set(false);
        }
    }
}
