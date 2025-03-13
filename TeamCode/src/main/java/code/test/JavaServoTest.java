package code.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTest J", group = "TEST")
public class JavaServoTest extends LinearOpMode {

    private Servo TestServo;

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        double position;
        double dtheta;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TestServo = hardwareMap.get(Servo.class, "TestServo");

        // Put initialization blocks here.
        position = 0;
        dtheta = 0.01;
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                if (gamepad1.left_bumper && gamepad1.right_bumper) {
                    position = 0.5;
                } else if (gamepad1.left_bumper) {
                    position = 0;
                } else if (gamepad1.a) {
                    position = 0.2;
                } else if (gamepad1.b) {
                    position = 0.4;
                } else if (gamepad1.y) {
                    position = 0.6;
                } else if (gamepad1.x) {
                    position = 0.8;
                } else if (gamepad1.right_bumper) {
                    position = 1;
                }
                position += dtheta * (gamepad1.right_trigger - gamepad1.left_trigger);
                position = Math.min(Math.max(position, 0), 1);
                TestServo.setPosition(position);
                telemetry.addData("Position", position);
                telemetry.addData("Delta", dtheta);
                telemetry.addData("Direction", TestServo.getDirection());
//                telemetry.addData("PWN", LeftClaw.isPwmEnabled());
                telemetry.update();
            }
        }
    }
}