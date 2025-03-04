package code.execution;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "FrankJrTeleOp (Courtesy of Mr. Ziemmer)")
public class FrankJrTeleOp extends LinearOpMode {

    private DcMotor right;
    private DcMotor left;
    private DcMotor arm;
    private Servo claw;
    private boolean precision = false;

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        right = hardwareMap.get(DcMotor.class, "right");
        left = hardwareMap.get(DcMotor.class, "left");
        arm = hardwareMap.get(DcMotor.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");

        // Put initialization blocks here.
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setDirection(DcMotor.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        claw.setPosition(0.5);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                if (gamepad1.x) {
                    arm.setPower(1 * (gamepad1.right_trigger - gamepad1.left_trigger));
                } else {
                    arm.setPower(0.75 * (gamepad1.right_trigger - gamepad1.left_trigger));
                }
                if (gamepad1.a) {
                    claw.setPosition(0.3);
                } else if (gamepad1.b) {
                    claw.setPosition(0.7);
                }
                if (gamepad1.y) {
                    precision = !precision;
                }
                double drive_multi = (precision?0.3:(gamepad1.left_stick_button?1.0:0.7));
                right.setPower(drive_multi*(gamepad1.left_stick_y - 0.5 * gamepad1.right_stick_x));
                left.setPower(drive_multi*(gamepad1.left_stick_y + 0.5 * gamepad1.right_stick_x));
                telemetry.update();
            }
        }
    }
}
