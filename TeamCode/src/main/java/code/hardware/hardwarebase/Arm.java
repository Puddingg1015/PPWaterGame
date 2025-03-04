package code.hardware.hardwarebase;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import code.control.identifiers.MotorType;
import code.control.identifiers.ZeroPowerBehaviourInputMode;

public class Arm extends Object {

    public DcMotorEx actuator;
    public final double CONST_ZEROPOWERBEHAVIOUR_BRAKE_ADJUSTMENT = 0.004; // 0.01, 0.004
    @Deprecated
    public final float SWITCH_POSITION = 0.0f;
    public final int GREATERTHAN_DIR = -1;
    public final double FORWARD_MULTI = 1.00;
    private double adjustment_power = 0;
    private final double INT_ADJ_CONST = 0.00001;
    public ZeroPowerBehaviourInputMode zero_power_behaviour_mode = ZeroPowerBehaviourInputMode.POSITIONINPUT;
    public int primary_position_tolerance = 5;

    public Arm() {
    }

    public void setActuators(Object... actuators) {
        this.actuator = (DcMotorEx) actuators[0];
        this.actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public boolean __validParameters() {
        if (this.actuator != null) {
            return true;
        }
        throw new IllegalStateException("Invalid Parameters");
    }

    public int getBoreEncValue(MotorType motor) {
        this.__validParameters();
        switch (motor) {

        }
        return 0;
    }

        public int getRoughArmPosition() {
        return this.actuator.getCurrentPosition();
    }

    public void preciseExtend(int count) {
        this.__validParameters();
    }

    public void changePosition(int delta_pos, double abs_power) {
        this.__validParameters();
        if (delta_pos < 0) abs_power = -Math.abs(abs_power);
        else abs_power = Math.abs(abs_power);
        int new_pos = this.actuator.getCurrentPosition() - delta_pos;
        this.actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.actuator.setTargetPosition(new_pos);
        this.actuator.setTargetPositionTolerance(this.primary_position_tolerance);
        this.actuator.setPower(abs_power);
        this.actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(this.actuator.isBusy()) {
        }
        this.setPower(MotorType.PRIMARY, 0);
    }


    public void setPower(MotorType motor, double value) {
        this.__validParameters();
        if (motor == MotorType.PRIMARY) {
              double const_power = 0;
              if (this.zero_power_behaviour_mode == ZeroPowerBehaviourInputMode.POSITIONINPUT) {
                  const_power = (this.actuator.getCurrentPosition() > -400 ?
                          CONST_ZEROPOWERBEHAVIOUR_BRAKE_ADJUSTMENT * GREATERTHAN_DIR :
                          FORWARD_MULTI * CONST_ZEROPOWERBEHAVIOUR_BRAKE_ADJUSTMENT * (-GREATERTHAN_DIR));
              } else {
                  const_power = (this.actuator.getVelocity(AngleUnit.DEGREES) > 0 ?
                          CONST_ZEROPOWERBEHAVIOUR_BRAKE_ADJUSTMENT * GREATERTHAN_DIR :
                          FORWARD_MULTI * CONST_ZEROPOWERBEHAVIOUR_BRAKE_ADJUSTMENT * (-GREATERTHAN_DIR));
              }
//              const_power = Math.pow(60*Math.sin(1.0/700.0*this.getRoughArmPosition()*0.5*Math.PI), 2);
    //          if (Math.abs(this.actuator.getVelocity()) > 200) const_power = 0;

              this.adjustment_power = const_power;
              this.actuator.setMotorEnable();
              this.actuator.setPower(value + const_power); //  * Math.abs(INT_ADJ_CONST * this.actuator.getVelocity())
        } else {

        }
    }

    public double getAdjustmentPower() {
        return this.adjustment_power;
    }

    public double getPower(MotorType motor) {
        this.__validParameters();
        if (motor == MotorType.PRIMARY) {
            return this.actuator.getPower();
        } else {
            return 0;
        }
    }

    public double getVelocity() {
        this.__validParameters();
        return this.actuator.getVelocity(AngleUnit.DEGREES);
    }

    public void stop(MotorType motor) {
        this.__validParameters();
        if (motor == MotorType.PRIMARY) {
            actuator.setPower(0);
        } else {

        }
    }
}
