package org.firstinspires.ftc.teamcode.xcentrics.util.qus;

import com.qualcomm.robotcore.hardware.CRServo;

@SuppressWarnings("ALL")
public class CRServoQUS extends QUS {
    private boolean first_cache = false; // Have we cached a value yet

    private double queued_power;

    @SuppressWarnings("CanBeFinal")
    private boolean do_cache;

    @SuppressWarnings("CanBeFinal")
    public CRServo servo;

    @SuppressWarnings("unused")
    public CRServoQUS(CRServo servo, boolean do_cache) {
        this.servo = servo;
        this.do_cache = do_cache;
    }

    @SuppressWarnings("unused")
    public CRServoQUS(CRServo servo) {
        this(servo, true);
    }

    public void queue_power(double speed) {
        if (do_cache && speed == queued_power && first_cache) {
            return;
        }

        queued_power = speed;
        needs_write = true;
        first_cache = true;
    }

    @Override
    protected void write() {
        servo.setPower(queued_power);
    }
}