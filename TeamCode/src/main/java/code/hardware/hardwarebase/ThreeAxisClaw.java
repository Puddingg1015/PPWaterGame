package code.hardware.hardwarebase;

import com.qualcomm.robotcore.hardware.Servo;

import code.control.identifiers.CStatus;

public class ThreeAxisClaw extends Claw {
    protected CStatus status;
    public ThreeAxisClaw() {}

    public ThreeAxisClaw(Servo actuator) {
        super(actuator);
    }

    // Assumes joint1 and joint2 actuators maps pos to the same angular orientation

    // Rotates vertically
    public void rotatePos(double pos) {
    }

    // Rotates the joint itself
    public void rotateSwivel(double pos) {
    }

    public double getLeftPosition() {
        return 0;
    }

    public double getRightPosition() {
        return 0;
    }

    public void setRotation(double value) {
    }

    public void setRotation(double vl, double vr) {
    }


    public void close() {
        this.actuator.setPosition(0.85);
        this.status = CStatus.CLOSED;
    }

    public void open() {
        this.actuator.setPosition(0.31);
        this.status = CStatus.OPEN;
    }

    public void setUp() {
    }

    public void setHorizontal() {
    }

    public void setDown() {
    }

    public CStatus getStatus() {
        return this.status;
    }

}
