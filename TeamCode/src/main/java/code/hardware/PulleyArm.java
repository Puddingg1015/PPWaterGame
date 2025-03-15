package code.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import code.hardware.hardwarebase.Arm;
import code.control.identifiers.MotorType;

public class PulleyArm extends Arm {

    public DcMotorEx spool;
    public TouchSensor toucher;
    public DigitalChannel magtoucher;


    public PulleyArm() {

    }

    @Override
    public void setActuators(Object... actuators) {
        super.setActuators(actuators[0]);
        this.spool = (DcMotorEx) actuators[1];
        this.spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        this.toucher = (TouchSensor) actuators[2];
        this.magtoucher = (DigitalChannel) actuators[2];
        this.magtoucher.setMode(DigitalChannel.Mode.INPUT);

    }

    @Override
    public boolean __validParameters() {
        super.__validParameters();
        if (this.spool != null && this.magtoucher != null) {
            return true;
        }
        throw new IllegalStateException("Invalid Parameters");
    }

    // Positive is retract, negative is extend
    public void powerSpool(double power) {
        if ((spool.getCurrent(CurrentUnit.AMPS) > 5 && power < 0) || (this.toucherIsPressed() && power > 0)) {
            spool.setPower(0);
        } else {
            spool.setPower(power);
        }
    }


    // total range is 0 ~ 1000 (actual range is 0 ~ -1000 but i switched negatives)
    public void setExtension(int delta_pos, double abs_power) throws Exception {
        this.__validParameters();
        if (delta_pos < 0) abs_power = -Math.abs(abs_power);
        else abs_power = Math.abs(abs_power);
        int new_pos = this.spool.getCurrentPosition() - delta_pos; //negatives switched
        this.spool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.spool.setTargetPosition(new_pos);
        this.spool.setTargetPositionTolerance(this.primary_position_tolerance);
        this.spool.setPower(abs_power);
        this.spool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(this.spool.isBusy()) {
                if ((spool.getCurrent(CurrentUnit.AMPS) > 5 && delta_pos > 0) || (this.toucher.isPressed() && delta_pos < 0)) {
                    spool.setPower(0);
                    this.spool.setTargetPosition(spool.getCurrentPosition());
                    throw new Exception("Erm what the sigma");
                }
        }
        this.spool.setPower(0);
    }

    @Deprecated
    public void runUsingPower() {
        this.spool.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getSpoolCurrentDraw() {
        return this.spool.getCurrent(CurrentUnit.AMPS);
    }

    public double getSpoolRoughPosition() {
        return this.spool.getCurrentPosition();
    }

    @Override
    public double getPower(MotorType motor) {
        this.__validParameters();
        if (motor == MotorType.PRIMARY) {
            return this.actuator.getPower();
        } else {
            return this.spool.getPower();
        }
    }

    public boolean toucherIsPressed() {

//        return this.toucher.isPressed();
        return !this.magtoucher.getState();
    }
}
