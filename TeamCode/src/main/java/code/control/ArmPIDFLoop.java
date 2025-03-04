package code.control;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "Arm PIDF Test")
public class ArmPIDFLoop extends LinearOpMode {

    private PIDController controller;
    public static double p = 0.006, i = 0.22, d = 0.0008;
    public static double f = -0.15;
    public static int target = 0;
    public final double TICKS_IN_DEG = 1300.0/180.0;

    private DcMotorEx arm;
    private DcMotorEx aux;

    public void cinit() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm = hardwareMap.get(DcMotorEx.class, "Arm");
        aux = hardwareMap.get(DcMotorEx.class, "ZAux");

    }

    @Override
    public void runOpMode() {
        cinit();
        waitForStart();
        while (opModeIsActive()) {
            cloop();
        }
    }

    public void cloop() {
        controller.setPID(p, i, d);
        int arm_pos = arm.getCurrentPosition();
        double pid = controller.calculate(arm_pos, target);
        double ff = Math.cos(Math.toRadians(target/TICKS_IN_DEG))*f;
        double power = pid + ff;
        target += (int) gamepad1.right_stick_y*10;
        arm.setPower(power);
        aux.setPower(power);
        telemetry.addData("Pos", arm_pos);
        telemetry.addData("Target", target);
        telemetry.addData("Experimental", aux.getCurrentPosition());
        telemetry.addData("pid", pid);
        telemetry.addData("ff", ff);
        telemetry.addData("cum", power);
        telemetry.addData("AuxP", aux.getPower());
        telemetry.update();
    }
}
