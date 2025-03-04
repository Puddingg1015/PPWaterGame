// Main Bot TeleOp v0.1
package code.control.IMU;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "Tele Op v0 - -2")
public class IMUTest extends LinearOpMode {

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
    private CRServo arm;
    private BNO055IMU imu;
    private AprilTagProcessor myAprilTagProcessor;
    private VisionPortal myVisionPortal;

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

    /**
     * This function is executed when this OpMode is selected from the Driver
     * Station.
     */
    @Override
    public void runOpMode() {
        Right_Front = hardwareMap.get(DcMotor.class, "FrontRight");
        Right_Back = hardwareMap.get(DcMotor.class, "BackRight");
        Left_Front = hardwareMap.get(DcMotor.class, "FrontLeft");
        Left_Back = hardwareMap.get(DcMotor.class, "BackLeft");
//    lClaw = hardwareMap.get(Servo.class, "lClaw");
//    rClaw = hardwareMap.get(Servo.class, "rClaw");
//    launcher = hardwareMap.get(Servo.class, "launcher");
//    arm = hardwareMap.get(CRServo.class, "arm");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        SpeedMode speed_mode = SpeedMode.STANDARD;
        // Put initialization blocks here.
        Right_Back.setDirection(DcMotor.Direction.REVERSE);
        Right_Front.setDirection(DcMotor.Direction.REVERSE);
        Left_Front.setDirection(DcMotor.Direction.REVERSE);
        // Right_Front.setDirection(DcMotor.Direction.REVERSE);
        Left_Back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // lClaw.setPosition(0.5);
        // rClaw.setPosition(0.5);
        Close_Claw();
        // launcher.setPosition(0.9);
        initIMU();
        // initAprilTag();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                double arm_power;
                if (Math.abs(gamepad2.right_stick_y) < Math.abs(ARM_AMB_POWER)) {
                    arm_power = -ARM_AMB_POWER;
                } else {
                    arm_power = 0.6 * gamepad2.right_stick_y;
                }
//        arm.setPower(arm_power);
                if (gamepad1.left_bumper) {
                    speed_mode = SpeedMode.TURBO;
                } else if (gamepad1.right_bumper) {
                    speed_mode = SpeedMode.PRECISION;
                } else {
                    speed_mode = SpeedMode.STANDARD;
                }
                changeSpeed(speed_mode);
                if (gamepad1.y) {
                    initIMU();
                }
                if (gamepad2.b) {
                    //   lClaw.setPosition(0.2);
                    //   rClaw.setPosition(0.35);
                }
                if (gamepad2.right_trigger > 0.2) {
                    //   launcher.setPosition(0.1);
                } else {
                    //   launcher.setPosition(0.9);
                }
                if (gamepad2.a) {
                    //   lClaw.setPosition(0.55);
                    //   rClaw.setPosition(0.05);
                }
                // driving();
                fieldCentricDriving();
                if (gamepad2.left_bumper) {
                    Open_Claw();
                }
                if (gamepad2.right_bumper) {
                    Close_Claw();
                }
                // telemetryAprilTag();

                telemetry.addData("Drivetrain Mode", speed_mode);
                telemetry.addData("Arm Power", arm_power);
                telemetry.addData("Gyroscope Measurement", getZAxisOrientation());
                telemetry.addData("IMU Gyroscope Calibration Status", imu.isGyroCalibrated());
                if (!imu.isGyroCalibrated()) {
                    telemetry.addData("WARNING", "IMU OUT OF SYNC; RECALIBRATE AS SOON AS POSSIBLE");
                }
                telemetry.update();
            }
        }
    }

    /**
     * Field Centric Driving
     */
    private void fieldCentricDriving() {
        telemetry.addData("THis", "Works");
        float angle;
        double x_rotated;
        double y_rotated;
        angle = -getZAxisOrientation();
        // Sets the axis parameters depending on the gamepad stick value and speed
        // division variable
        vert = (gamepad1.left_stick_y * STD_SPEED / speed_div);
        horizontal = -gamepad1.left_stick_x * STD_SPEED / speed_div;
        pivot = -(gamepad1.right_stick_x * STD_SPEED * pivot_sign) / speed_div;
        // Odometry angle calculations for x and y rotations
        x_rotated = horizontal * Math.cos(angle / 180 * Math.PI) - vert * Math.sin(angle / 180 * Math.PI);
        y_rotated = horizontal * Math.sin(angle / 180 * Math.PI) + vert * Math.cos(angle / 180 * Math.PI);
        // Sets the robot motor power depending on the pivot sign and the x and y
        // rotation value
        Right_Front.setPower(-pivot + (y_rotated + x_rotated));
        Right_Back.setPower(-pivot + y_rotated - x_rotated);
        Left_Front.setPower(pivot + y_rotated + x_rotated);
        Left_Back.setPower(pivot + (y_rotated - x_rotated));
        // telemetry.update();
        // if (gamepad1.b) {
        // initIMU();
        // }
        // telemetry.update();
    }

    /**
     * Sets the IMU parameters (mode, angle unit, and acceleration unit)
     */
    private void initIMU() {
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        telemetry.addData("Status", "Init IMU... Please wait");
        telemetry.update();
        imu.initialize(imuParameters);
        telemetry.addData("Status", "IMU Initialized");
        telemetry.update();
    }

    private float getZAxisOrientation() {
        Orientation angles;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float yaw = angles.firstAngle;
        // yaw += 90.0f;
        // if (yaw > 180.0f) {
        // yaw = 360.0f - yaw;
        // }
        return yaw;
    }

    private void changeSpeed(SpeedMode speed_mode) {
        // ElapsedTime time = new ElapsedTime();
        // time.reset();
        if (speed_mode == SpeedMode.TURBO)
            speed_div = 1.0 / TURBO_MULTI;
        else if (speed_mode == SpeedMode.PRECISION)
            speed_div = 1.0 / PRECISION_MULTI;
        else
            speed_div = 1.0;
        // while (time.milliseconds() < 200) {
        // fieldCentricDriving();
        // telemetry.addData("FORWARD?", forward_);
        // telemetry.addData("Speed Mode", speed_mode);
        // telemetry.addData("Speed Division", speed_div);
        // }
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

/* END DriverbotTeleop_VFARII.java */