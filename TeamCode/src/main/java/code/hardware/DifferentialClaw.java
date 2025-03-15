package code.hardware;

import com.qualcomm.robotcore.hardware.Servo;

import code.control.identifiers.CStatus;
import code.control.identifiers.OrthoType;
import code.hardware.hardwarebase.Claw;
import code.hardware.hardwarebase.ThreeAxisClaw;

public class DifferentialClaw extends ThreeAxisClaw {

    Servo left;
    Servo right;
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
    public void rotateSwivel(double pos) {
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

    public void setUp() {
        this.left.setPosition(0);
        this.right.setPosition(1);
    }


    public void setStraight() {
        this.left.setPosition(0.655);
        this.right.setPosition(0.9622);
    }

    public void setDown(OrthoType type) {
        if (type == OrthoType.VERTICAL) {
            this.left.setPosition(0.5783);
            this.right.setPosition(0.3244);
        } else {
            this.left.setPosition(0.9289);
            this.right.setPosition(0.7022);
        }
    }

    public void setDown() {
        this.setDown(OrthoType.HORIZONTAL);
    }
    
    public void setBucket() {
        this.left.setPosition(0.3);
        this.left.setPosition(1);
    }
}
