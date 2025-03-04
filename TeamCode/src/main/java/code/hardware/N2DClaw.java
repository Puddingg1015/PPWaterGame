package code.hardware;

import com.qualcomm.robotcore.hardware.Servo;

import code.hardware.hardwarebase.Claw;

public class N2DClaw extends Claw {

    Servo wrist;
    public N2DClaw() {}
    double wrist_incr = 0.1;

    public N2DClaw(Servo actuator, Servo wrist) {
        super(actuator);
        this.wrist = wrist;
    }

    public void setAux(Servo wrist) {
        this.wrist = wrist;
    }

    public void setWristPos(double pos) {
        wrist.setPosition(pos);
    }

    public double getWristPos() {
        return wrist.getPosition();
    }

    public void incrWrist() {
        wrist.setPosition(wrist.getPosition() + wrist_incr);
    }

    public void decrWrist() {
        wrist.setPosition(wrist.getPosition() - wrist_incr);
    }

    public void setWristUp() {
        wrist.setPosition(1);
    }

    public void setWristForward() {
        wrist.setPosition(1);
    }

    public void setWristDown() {
        wrist.setPosition(0);
    }
}
