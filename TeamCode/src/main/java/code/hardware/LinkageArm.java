package code.hardware;

import com.qualcomm.robotcore.hardware.Servo;

import code.hardware.hardwarebase.Arm;

public class LinkageArm extends Arm {

    Servo linkage;

    public LinkageArm() {

    }

    @Override
    public void setActuators(Object... actuators) {
        super.setActuators(actuators[0]);
        this.linkage = (Servo) actuators[1];
    }

    @Override
    public boolean __validParameters() {
        super.__validParameters();
        if (this.linkage != null) {
            return true;
        }
        throw new IllegalStateException("Invalid Parameters");
    }

    public void extendArm() {
        this.linkage.setPosition(0.0900);
    }


    public void partialArm(int partial) {
        switch (partial) {
            case 1: {
                this.linkage.setPosition(0.1500);
                break;
            } case 2: {
                this.linkage.setPosition(0.2400);
                break;
            } case 3: {
                this.linkage.setPosition(0.2934);
                break;
            } case 4: {
                this.linkage.setPosition(0.3200);
                break;
            } default: {
                throw new IllegalArgumentException("The partial value of " + partial + " is improper.");
            }
        }
    }


    public void contractArm() {
        this.linkage.setPosition(0.3500);
    }
}
