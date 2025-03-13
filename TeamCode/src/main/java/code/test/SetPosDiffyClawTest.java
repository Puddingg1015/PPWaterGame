package code.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import code.hardware.DifferentialClaw;

@Config
@TeleOp(name = "SET POS Differential Claw Test", group = "TEST")
public class SetPosDiffyClawTest extends LinearOpMode {
    protected DifferentialClaw claw;
    static double left = 0.5;
    static double right = 0.5;
    double CLAW_DELTA = 0.02;
    final double CLAW_0 = 0.6017;
    final double CLAW_90 = 1;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        claw = new DifferentialClaw(
                hardwareMap.get(Servo.class, "MainClaw"),
                hardwareMap.get(Servo.class, "LeftClaw"),
                hardwareMap.get(Servo.class, "RightClaw")
        );
        waitForStart();
        while (opModeIsActive()) {
            claw.setRotation(left, right);
            telemetry.addData("Left", claw.getLeftPosition());
            telemetry.addData("Right", claw.getRightPosition());
            telemetry.addData("Status", claw.getStatus());
            telemetry.update();
        }
    }
}
