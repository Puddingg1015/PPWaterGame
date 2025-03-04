package code.hardware.hardwarebase;

import com.qualcomm.robotcore.hardware.Servo;

import code.control.identifiers.CStatus;

public class Claw {

    protected Servo actuator;
    CStatus status;
    public Claw() {}

    public Claw(Servo actuator) {
        this.actuator = actuator;
    }

    public void setActuator(Servo actuator) {
        this.actuator = actuator;
    }

    public void close() {
        this.actuator.setPosition(0.69);
        this.status = CStatus.CLOSED;
    }

    public void open() {
        this.actuator.setPosition(0.31);
        this.status = CStatus.OPEN;
    }
}
