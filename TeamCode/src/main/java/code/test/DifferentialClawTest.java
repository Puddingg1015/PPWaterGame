package code.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import code.hardware.DifferentialClaw;

@TeleOp(name = "Differential Claw Test", group = "TEST")
public class DifferentialClawTest extends LinearOpMode {
    protected DifferentialClaw claw;
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
            if (gamepad2.a) {
                claw.open();
            } if (gamepad2.b) {
                claw.close();
            }

            if (gamepad2.x) {
                claw.rotateSwivel(0.02);
            } if (gamepad2.y) {
                claw.rotateSwivel(-0.02);
            }
            if (gamepad2.left_bumper) {
                claw.rotatePos(0.02);
            } if (gamepad2.right_bumper) {
                claw.rotatePos(-0.02);
            }

            if (gamepad1.left_bumper) {
                claw.setRotation(CLAW_0);
            }
            if (gamepad1.right_bumper) {
                claw.setRotation(CLAW_90);
            }
            if (gamepad1.x) {
                claw.setRotation(gamepad1.left_trigger, gamepad1.right_trigger);
            }
            if (gamepad1.y) {
                claw.setRotation(0.5, 0.5);
            }

            double claw_multi = 1.0;
            if (gamepad1.right_stick_button) {
                claw_multi = 2.0;
            }

            claw.rotateSwivel(-gamepad1.right_stick_x*CLAW_DELTA*claw_multi);
            claw.rotatePos(gamepad1.right_stick_y*CLAW_DELTA*claw_multi);
            telemetry.addData("Left", claw.getLeftPosition());
            telemetry.addData("Right", claw.getRightPosition());
            telemetry.addData("Status", claw.getStatus());
            telemetry.update();
        }
    }
}
