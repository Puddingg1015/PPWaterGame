package code.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import code.hardware.hardwarebase.Arm;
import code.control.identifiers.MotorType;

public class ZauxArm extends Arm {

    private DcMotorEx aux;

    public ZauxArm() {

    }

    @Override
    public void setActuators(Object... actuators) {
        super.setActuators(actuators[0]);
        this.aux = (DcMotorEx) actuators[1];
    }

    @Override
    public boolean __validParameters() {
        super.__validParameters();
        if (this.aux != null) {
            return true;
        }
        throw new IllegalStateException("Invalid Parameters");
    }

    @Override
    public void setPower(MotorType motor_type, double power) {
        if (motor_type == MotorType.PRIMARY) {
             super.setPower(motor_type, power);
        } else if (motor_type == MotorType.SECONDARY) {
            if (aux.getCurrent(CurrentUnit.AMPS) > 5) {
                aux.setPower(0);
            } else {
                aux.setPower(power);
            }
        }
    }

    public double getAuxCurrentDraw() {
        return this.aux.getCurrent(CurrentUnit.AMPS);
    }

    public double getAuxRoughPosition() {
        return this.aux.getCurrentPosition();
    }

    public double getAuxCurrentAlert() {
        return this.aux.getCurrentAlert(CurrentUnit.AMPS);
    }

    @Override
    public double getPower(MotorType motor) {
        this.__validParameters();
        if (motor == MotorType.PRIMARY) {
            return this.actuator.getPower();
        } else {
            return this.aux.getPower();
        }
    }
}
