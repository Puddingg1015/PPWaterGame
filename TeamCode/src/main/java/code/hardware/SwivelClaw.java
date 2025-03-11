package code.hardware;

import com.qualcomm.robotcore.hardware.Servo;

import code.control.identifiers.CStatus;
import code.hardware.hardwarebase.ThreeAxisClaw;

public class SwivelClaw extends ThreeAxisClaw {

    Servo wrist;
    Servo swivel;
    public SwivelClaw() {}

    public SwivelClaw(Servo actuator, Servo wrist, Servo swivel) {
        super(actuator);
        this.wrist = wrist;
        this.swivel = swivel;
    }

    // Assumes wrist and swivel actuators maps pos to the same angular orientation

    // Rotates vertically
    public void rotatePos(double pos) {
        wrist.setPosition(wrist.getPosition()+pos);
    }

    // Rotates the joint itself
    public void rotateSwivel(double pos) {
        swivel.setPosition(swivel.getPosition()+pos);
    }

    public double getLeftPosition() {
        return wrist.getPosition();
    }

    public double getRightPosition() {
        return swivel.getPosition();
    }

    public void setRotation(double value) {
        swivel.setPosition(value);
    }

    public void setRotation(double vl, double vr) {
        swivel.setPosition(vl);
    }

    public void setUp() {
        this.wrist.setPosition(0);
    }

    public void setHorizontal() {
        this.wrist.setPosition(0.6);
    }

    public void setDown() {
        this.wrist.setPosition(1);
    }
}
