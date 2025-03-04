package code.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class ZDClaw extends DoubleClaw {
    public ZDClaw (Servo left_claw, Servo right_claw) {
        super(left_claw, right_claw);
    }

    @Override
    public void open() {
        this.left.setPosition(1.00);
        this.right.setPosition(0.00);
    }

    @Override
    public void close() {
        this.left.setPosition(0.6);
        this.right.setPosition(0.4);
    }

    @Deprecated
    public void Rclose() {
        this.left.setPosition(0.0);
        this.right.setPosition(0.9);
    }

}
