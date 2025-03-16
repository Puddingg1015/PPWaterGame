package code.hardware;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.config.Config;

import code.control.identifiers.MotorType;

@Config
public class ExArm extends PulleyArm {

    private PIDController controller;
    private DcMotorEx auxillary;
    public static double p = 0.006, i = 0.22, d = 0.0008;
    public static double f = -0.01;
    public static int target = 0;
    public final double TICKS_IN_DEG = 1300.0/180.0;
    public final int LOOP_INCR = 20;

    public ExArm() {
        super();
        this.controller = new PIDController(p, i, d);
        try {
            target = this.getRoughArmPosition();
        } catch (NullPointerException ignored) {}
    }

    @Override
    public void setActuators(Object... actuators) {
        super.setActuators(actuators[0], actuators[1], actuators[2]);
        try {
            this.auxillary = (DcMotorEx) actuators[3];
            this.auxillary.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception ignored) {}
        target = this.getRoughArmPosition();
    }

    @Override
    public void setPower(MotorType motor, double incr_proportion) {
        this.__validParameters();
        if (motor == MotorType.PRIMARY) {
            double power = incr_proportion;
//            controller.setPID(p, i, d);
            // engage zero power behaviour
            if (power == 0) {
                int arm_pos = this.actuator.getCurrentPosition();
                double pid = controller.calculate(arm_pos, target);
                double ff = Math.cos(Math.toRadians(target/TICKS_IN_DEG))*f;
                power = ff;
//                power = -0.1;
//                power = -0.001;
            } else { // just have regular power being power;
                // whenever we apply power, ZPB is off
                // kinda scuffed but it hopefully works
                power = incr_proportion;
                target = this.getRoughArmPosition();
//                int arm_pos = this.actuator.getCurrentPosition();
//                double pid = controller.calculate(arm_pos, target);
//                double ff = Math.cos(Math.toRadians(target / TICKS_IN_DEG)) * f;
//                power = pid + ff;
            }
            this.actuator.setPower(power);
            this.auxillary.setPower(power);
        } else {

        }
    }

    @Override
    public void changePosition(int delta_pos, double abs_power) {
        this.__validParameters();
        if (delta_pos < 0) abs_power = -Math.abs(abs_power);
        else abs_power = Math.abs(abs_power);
        int new_pos = this.actuator.getCurrentPosition() - delta_pos;
        this.actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.actuator.setTargetPosition(new_pos);
//        this.actuator.setTargetPositionTolerance(this.primary_position_tolerance);
        while(this.actuator.isBusy()) {
            controller.setPID(p, i, d);
            target = this.actuator.getTargetPosition();
            int arm_pos = this.actuator.getCurrentPosition();
            double pid = controller.calculate(arm_pos, target);
            double ff = Math.cos(Math.toRadians(target/TICKS_IN_DEG))*f;
            double power;
//            double power = (pid + ff > 0 ? Math.min(pid + ff, abs_power) : Math.max(pid + ff, abs_power));
            power = pid+ff;
            this.actuator.setPower(power);
            this.auxillary.setPower(power);
        }
        this.setPower(MotorType.PRIMARY, 0);
    }

}
