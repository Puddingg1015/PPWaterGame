package code.hardware;

import com.qualcomm.robotcore.hardware.Servo;

import code.hardware.hardwarebase.Claw;

public class DoubleClaw extends Claw {
    Servo left;
    Servo right;

    public DoubleClaw(Servo left_claw, Servo right_claw) {
        super();
        this.left = left_claw;
        this.right = right_claw;
    }

    @Override
    public void open() {
        this.left.setPosition(1.00);
        this.right.setPosition(0.00);
    }

    @Override
    public void close() {
        this.left.setPosition(0.16);
        this.right.setPosition(0.44);
    }
}
