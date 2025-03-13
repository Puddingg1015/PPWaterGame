package code.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import code.hardware.DifferentialClaw;
import code.vision.GameObjectCVProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
// Recognition - Return type for TenserFlow processor
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;


@Config
@TeleOp(name = "Diffy Funnies", group = "TEST")
public class DifferentialOrientationTest extends LinearOpMode {

    private DcMotor Right_Front;
    private DcMotor Right_Back;
    private DcMotor Left_Front;
    private DcMotor Left_Back;
    private AprilTagProcessor AprilTagProcessor;
    private VisionPortal VisionPortal;
    private DifferentialClaw claw;

    /* CONSTS */
    final double CLAW_DELTA = 0.01;
    // 90 : 0.60167
    //  0 : 0.24167
    // 180: 1.00000
    //
    // v2:
    /*
    0:
    L: 0.6017
    R: 0.3289
    90:
    L: 1.0000
    R: 0.7272
    (Right is 0.3983 ticks behind Left bc gears skipped)
     */
    final double CLAW_0 = 0.6017;
    final double CLAW_90 = 1;
    final double RIGHTCLAW_POSTFIX = -0.3983;
//    final double CLAW_180 = 1.00000;

    /* GLOBAL VARIABLES */
    boolean USE_WEBCAM = true;
    GameObjectCVProcessor cvProcessor;
    ElapsedTime cooldown;


    private void initVisionProcessor() {
        this.cvProcessor = new GameObjectCVProcessor(hardwareMap, "ArmCam");
    }

    /**
     * This function is executed when this OpMode is selected from the Driver
     * Station.
     */
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Right_Front = hardwareMap.get(DcMotor.class, "FrontRight"); // 2
        Right_Back = hardwareMap.get(DcMotor.class, "BackRight"); // 3
        Left_Front = hardwareMap.get(DcMotor.class, "FrontLeft"); // 0
        Left_Back = hardwareMap.get(DcMotor.class, "BackLeft"); // 1
//    lClaw = hardwareMap.get(Servo.class, "lClaw");
//    rClaw = hardwareMap.get(Servo.class, "rClaw");
//    launcher = hardwareMap.get(Servo.class, "launcher");

        // Put initialization blocks here.
//        Right_Back.setDirection(DcMotor.Direction.REVERSE);
        Right_Front.setDirection(DcMotor.Direction.REVERSE);
//        Left_Front.setDirection(DcMotor.Direction.REVERSE);
        Right_Front.setDirection(DcMotor.Direction.REVERSE);
        Left_Back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        claw = new DifferentialClaw(
                hardwareMap.get(Servo.class, "MainClaw"),
                hardwareMap.get(Servo.class, "LeftClaw"),
                hardwareMap.get(Servo.class, "RightClaw")
        );

        // lClaw.setPosition(0.5);
        // rClaw.setPosition(0.5);
        initVisionProcessor();
        this.cvProcessor.safeStart();
        waitForStart();
        int err = 0;
        if (opModeIsActive()) {
            cooldown = new ElapsedTime();
            // Put run blocks here.
            while (opModeIsActive()) {
                if (gamepad1.left_bumper) {
                    claw.close();
                }
                if (gamepad1.right_bumper) {
                    claw.open();
                }
//                 driving();
                double orientation_updated_position = CLAW_0 +
                        (CLAW_90-CLAW_0)*(90-Math.abs(this.cvProcessor.pipeline.getOrientation()))/90;
                if (this.cvProcessor.isStreaming()) {
                    telemetry.addData("Streaming", this.cvProcessor.isStreaming());
                    telemetry.addData("Press A", "Stop Streaming");
                    if (super.gamepad1.a && cooldown.milliseconds() > 500) {
                        this.cvProcessor.stop();
                        cooldown.reset();
                    }
                    this.claw.setRotation(orientation_updated_position, orientation_updated_position + RIGHTCLAW_POSTFIX);
                } else {
                    telemetry.addData("Streaming", "OFF");
                    telemetry.addData("Press A", "Start Streaming");
                    if (super.gamepad1.a && cooldown.milliseconds() > 500) {
                        err = this.cvProcessor.safeStart();
                        cooldown.reset();
                    }
                    double claw_multi = 1.0;
                    if (gamepad2.right_stick_button) {
                        claw_multi = 2.0;
                    }
                    claw.rotateSwivel(-gamepad2.right_stick_x*CLAW_DELTA*claw_multi);
                    claw.rotatePos(-gamepad2.right_stick_y*CLAW_DELTA*claw_multi);

                }
                telemetry.addData("Status", err);
                telemetry.addData("Streaming", this.cvProcessor.isStreaming());
                telemetry.addData("Orientation", this.cvProcessor.pipeline.getOrientation());
                telemetry.addData("Left", claw.getLeftPosition());
                telemetry.addData("Right", claw.getRightPosition());
                telemetry.addData("Interpreted Pos", orientation_updated_position);
                telemetry.update();
            }
        }
    }

    /**
     * /**
     * Opens claw and waits a bit
     */
    private void Open_Claw() {
//    lClaw.setPosition(0.20);
//    rClaw.setPosition(0.40);
        sleep(100);
    }

    /**
     * Lowers arm and closes claw and waits a bit longer
     */
    private void Close_Claw() {

//    lClaw.setPosition(0.59);
//    rClaw.setPosition(0.01);
        sleep(100);
    }

    /**
     * Describe this function...
     */
    private void driving() {
        Left_Back.setPower(0.75 * (gamepad1.left_stick_y + gamepad1.left_stick_x + -gamepad1.right_stick_x));
        Left_Front.setPower(0.75 * (gamepad1.left_stick_y + -gamepad1.left_stick_x + -gamepad1.right_stick_x));
        Right_Back.setPower(0.75 * (gamepad1.left_stick_y + -gamepad1.left_stick_x + gamepad1.right_stick_x));
        Right_Front.setPower(0.75 * (gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
    }
}

