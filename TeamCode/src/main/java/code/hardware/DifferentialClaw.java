package code.hardware;

import com.qualcomm.robotcore.hardware.Servo;

import code.control.identifiers.CStatus;
import code.hardware.hardwarebase.Claw;

public class DifferentialClaw extends Claw {

    Servo left;
    Servo right;
    protected CStatus status;
    public DifferentialClaw() {}

    public DifferentialClaw(Servo actuator, Servo left, Servo right) {
        super(actuator);
        this.left = left;
        this.right = right;
    }

    // Assumes left and right actuators maps pos to the same angular orientation

    // Rotates vertically
    public void rotatePos(double pos) {
        left.setPosition(left.getPosition()-pos/2);
        right.setPosition(right.getPosition()+pos/2);
    }

    // Rotates the joint itself
    public void rotateWrist(double pos) {
        left.setPosition(left.getPosition()+pos);
        right.setPosition(right.getPosition()+pos);
    }

    public double getLeftPosition() {
        return left.getPosition();
    }

    public double getRightPosition() {
        return right.getPosition();
    }

    public void setRotation(double value) {
        left.setPosition(value);
        right.setPosition(value);
    }

    public void setRotation(double vl, double vr) {
        left.setPosition(vl);
        right.setPosition(vr);
    }


    public void close() {
        this.actuator.setPosition(0.69);
        this.status = CStatus.CLOSED;
    }

    public void open() {
        this.actuator.setPosition(0.31);
        this.status = CStatus.OPEN;
    }

    public void setUp() {
        this.left.setPosition(0);
        this.right.setPosition(1);
    }

    public void setHorizontal() {
        this.left.setPosition(0.6);
        this.right.setPosition(0.1);
    }

    public void setDown() {
        this.left.setPosition(1);
        this.right.setPosition(0.6);
    }

    public CStatus getStatus() {
        return this.status;
    }
}
