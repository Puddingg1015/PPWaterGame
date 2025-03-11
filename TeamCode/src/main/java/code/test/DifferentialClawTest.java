package code.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import code.hardware.DifferentialClaw;

@TeleOp(name = "differential claw test", group = "TEST")
public class DifferentialClawTest extends LinearOpMode {
    protected DifferentialClaw claw;

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
            if (gamepad1.x) {
                claw.rotateSwivel(0.02);
            } if (gamepad1.y) {
                claw.rotateSwivel(-0.02);
            }
            if (gamepad1.left_bumper) {
                claw.rotatePos(0.02);
            } if (gamepad1.right_bumper) {
                claw.rotatePos(-0.02);
            }
            telemetry.addData("Left", claw.getLeftPosition());
            telemetry.addData("Right", claw.getRightPosition());
            telemetry.update();
        }
    }
}
