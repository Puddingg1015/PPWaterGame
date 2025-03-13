package code.test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import code.control.identifiers.MotorType;
import code.hardware.hardwarebase.Claw;
import code.hardware.hardwarebase.Drivetrain;
import code.hardware.DrivetrainHandler;
import code.hardware.LinkageArm;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Tiger Auto Methods test", group = "TEST")
public class TigerAutoMethodTest extends LinearOpMode {

    enum SpeedMode {
        TURBO, STANDARD, PRECISION
    }

    enum AprilTag {
        NULL, BLU_LEFT, BLU_CENTER, BLU_RIGHT, RED_LEFT, RED_CENTER, RED_RIGHT
    }

    Drivetrain drivetrain;
    DrivetrainHandler drivetrainHandler;
    private Claw claw;
    private Servo launcher;
    private LinkageArm arm;
    private BNO055IMU imu;
    private AprilTagProcessor myAprilTagProcessor;
    private VisionPortal myVisionPortal;

    /* CONSTS */
    final double TURBO_MULTI = 1.5;
    final double PRECISION_MULTI = 0.375;
    final double STD_SPEED = 0.69;
    @Deprecated
    final double ARM_AMB_POWER = 0.0;
    final double CLAW_TRIGGER_INV_SENS = 0.3;
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

    @Override
    public void runOpMode() {

        /* Initialization */
        this.drivetrain = new Drivetrain(hardwareMap);
        this.drivetrain.initializeDriveTrain();
        arm = new LinkageArm();
        arm.setActuators(
                hardwareMap.get(DcMotor.class, "Arm"),
                hardwareMap.get(Servo.class, "Linkage")
        );
        claw = new Claw(hardwareMap.get(Servo.class, "LeftClaw"));
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");
        this.initIMU();
        this.drivetrainHandler = new DrivetrainHandler(this.drivetrain, this.telemetry);


        /* Initialization Telemetry */
        telemetry.addData("Camera Stream", "ACTIVE");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        /* Wait for the match to start */
        waitForStart();
        claw.close();
        arm.setPower(MotorType.PRIMARY, -0.7);
        sleep(300);
        arm.setPower(MotorType.PRIMARY, -0.14);
        sleep(300);
        if (opModeIsActive()) {
            this.drivetrainHandler.resetEncoders();
            this.drivetrainHandler.runToPos();
            this.drivetrainHandler.Forward(500, 0.2);
            sleep(1000);
            this.drivetrainHandler.Left(500, 0.2);
            sleep(1000);
            this.drivetrainHandler.Backward(500, 0.2);
            sleep(1000);
            this.drivetrainHandler.Right(500, 0.2);
            sleep(1000);

            endAuto();
        }
    }

    private void endAuto() {
        this.drivetrainHandler.stopMotors();
        sleep(30000);
    }

    private void fieldCentricPowerVectorMovement(double x, double y) {
        this.fieldCentricPowerVectorMovement(x, y, 0);
    }

    /**
     * Field Centric Driving
     */
    private void fieldCentricPowerVectorMovement(double x, double y, double r) {
        telemetry.addData("This", "Works");
        float angle;
        double x_rotated;
        double y_rotated;
        angle = -getZAxisOrientation();
        // Sets the axis parameters depending on the gamepad stick value and speed
        // division variable
        vert = (y * STD_SPEED / speed_div);
        horizontal = -x * STD_SPEED / speed_div;
        pivot = -(r * STD_SPEED * pivot_sign) / speed_div;
        // Odometry angle calculations for x and y rotations
        x_rotated = horizontal * Math.cos(angle / 180 * Math.PI) - vert * Math.sin(angle / 180 * Math.PI);
        y_rotated = horizontal * Math.sin(angle / 180 * Math.PI) + vert * Math.cos(angle / 180 * Math.PI);
        // Sets the robot motor power depending on the pivot sign and the x and y
        // rotation
        this.drivetrain.Right_Front.setPower(-pivot - (y_rotated - x_rotated));
        this.drivetrain.Right_Back.setPower(-pivot - (y_rotated + x_rotated));
        this.drivetrain.Left_Front.setPower(pivot - (y_rotated + x_rotated));
        this.drivetrain.Left_Back.setPower(pivot - (y_rotated - x_rotated));
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


}

