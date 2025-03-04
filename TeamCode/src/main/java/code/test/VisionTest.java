// Main Bot TeleOp v0.1
package code.test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import code.hardware.hardwarebase.Arm;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
// Recognition - Return type for TenserFlow processor
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;


@TeleOp(name = "VisionTest")
public class VisionTest extends LinearOpMode {

    enum SpeedMode {
        TURBO, STANDARD, PRECISION
    }

    enum AprilTag {
        NULL, BLU_LEFT, BLU_CENTER, BLU_RIGHT, RED_LEFT, RED_CENTER, RED_RIGHT
    }

    private DcMotor Right_Front;
    private DcMotor Right_Back;
    private DcMotor Left_Front;
    private DcMotor Left_Back;
    private Servo lClaw;
    private Servo rClaw;
    private Servo launcher;
    private Arm arm;
    private BNO055IMU imu;
    private AprilTagProcessor AprilTagProcessor;
    private VisionPortal VisionPortal;

    /* CONSTS */
    final double TURBO_MULTI = 1.5;
    final double PRECISION_MULTI = 0.375;
    final double STD_SPEED = 0.69;
    final double ARM_AMB_POWER = 0.0;
    // NOTE: Switch the SPARK mini white box to B (break) if it is on F (float)
    // This will set the arm's zero power behaviour to break automatically instead
    // of having to hardcode it
    // If it does not work, use ARM_ZERO_POWER = 0.075 to mimic zero power behaviour
    // as closely as possible
    // However, this will most likely also cause the robot to DC and not function
    // more frequently.

    /* GLOBAL VARIABLES */
    double vert;
    double horizontal;
    double speed_div = 1.0;
    double pivot;
    int forward_;
    double pivot_sign = 1.0;
    boolean USE_WEBCAM = true;


    private void initVisionProcessor() {
        // AprilTag Config
        AprilTagProcessor AprilTagProcessor = org.firstinspires.ftc.vision.apriltag.AprilTagProcessor.easyCreateWithDefaults();
        // Camera Configuration
        VisionPortal.Builder VPCVBuilder = new VisionPortal.Builder();
        VPCVBuilder.setCamera(hardwareMap.get(WebcamName.class, "ArmCam"));
        VPCVBuilder.addProcessors(AprilTagProcessor);
        VisionPortal = VPCVBuilder.build();
    }

    /**
     * This function is executed when this OpMode is selected from the Driver
     * Station.
     */
    @Override
    public void runOpMode() {
        Right_Front = hardwareMap.get(DcMotor.class, "FrontRight"); // 2
        Right_Back = hardwareMap.get(DcMotor.class, "BackRight"); // 3
        Left_Front = hardwareMap.get(DcMotor.class, "FrontLeft"); // 0
        Left_Back = hardwareMap.get(DcMotor.class, "BackLeft"); // 1
//    lClaw = hardwareMap.get(Servo.class, "lClaw");
//    rClaw = hardwareMap.get(Servo.class, "rClaw");
//    launcher = hardwareMap.get(Servo.class, "launcher");
        arm = new Arm();
        arm.setActuators(
                hardwareMap.get(DcMotor.class, "Arm"),
                hardwareMap.get(DcMotor.class, "Spool")
        );
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        SpeedMode speed_mode = SpeedMode.STANDARD;
        // Put initialization blocks here.
//        Right_Back.setDirection(DcMotor.Direction.REVERSE);
        Right_Front.setDirection(DcMotor.Direction.REVERSE);
//        Left_Front.setDirection(DcMotor.Direction.REVERSE);
        Right_Front.setDirection(DcMotor.Direction.REVERSE);
        Left_Back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // lClaw.setPosition(0.5);
        // rClaw.setPosition(0.5);
        initVisionProcessor();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
//                 driving();
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

