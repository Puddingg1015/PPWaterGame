// Main Bot TeleOp v0.1
package code.test;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import code.hardware.hardwarebase.Drivetrain;
import code.hardware.DrivetrainHandler;
import code.hardware.N2DClaw;
import code.hardware.PulleyArm;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@Config
@Autonomous(name = "PID Drivetrain Demo", group = "TEST")
public class TigerPIDDrivetrainTest extends LinearOpMode {

    Drivetrain drivetrain;
    DrivetrainHandler drivetrainHandler;
    public int power = 1;
    protected N2DClaw claw;
    protected PulleyArm arm;
    protected IMU imu;
    protected AprilTagProcessor myAprilTagProcessor;
    protected VisionPortal myVisionPortal;
//    IMUHandler imuHandler;

    /* CONSTS */
    /* GLOBAL VARIABLES */
    double vert;
    double horizontal;
    double speed_div = 1.0;
    double pivot;
    int forward_;
    double pivot_sign = 1.0;
    double STD_SPEED = 1.0;


    final double PI = 3.14159265358979323;

    @Override
    public void runOpMode() {

        /* Initialization */
        this.drivetrain = new Drivetrain(hardwareMap);
        this.drivetrain.initializeDriveTrain();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.imu = hardwareMap.get(IMU.class, "imu");
        this.initIMU();
//        this.imuHandler = new IMUHandler(imu);
        this.drivetrainHandler = new DrivetrainHandler(this.drivetrain, this.telemetry, this.imu);
        /* Wait for the match to start */
        waitForStart();
        this.logEvent(-1);
        telemetry.update();
        if (opModeIsActive()) {
            this.logEvent(1, "STA");
            drivetrainHandler.movePIDDisplacementVector(300, 500, power);
            this.logEvent(2);
            drivetrainHandler.movePIDDisplacementVector(-300, 500, power);
            drivetrainHandler.movePIDDisplacementVector(-300, -500, power);
            drivetrainHandler.movePIDDisplacementVector(300, -500, power);
            this.logEvent(99, "FIN");

        }
    }

    protected void moveAndPlaceSpecimen() throws Exception {
        // Extend
        // this.arm.setExtension(1000, 1);
        claw.setWristPos(0.5);
        this.drivetrainHandler.movePIDDisplacementVector(0, 200, 0.5); // 0.5
//        sleep(300);
        // Hook
        arm.changePosition(-75, 0.5);
//        sleep(300);
        this.drivetrainHandler.movePIDDisplacementVector(0, -200, 0.5);
        claw.open();
        // Drop
//        sleep(300);
        claw.setWristPos(0.8);
        // this.arm.setExtension(-1000, 0.9);
        sleep(0);
        this.claw.close();
        arm.changePosition(150, 0.5);
    }

    protected void rest(int ms) {
        this.drivetrainHandler.stopMotors();
        sleep(ms);
    }

    protected void logEvent(int event) {
        logEvent(event, "null");
        telemetry.update();
    }

    protected void logEvent(int event, Object msg) {
        telemetry.addData("Raised Event", event + " - " + msg.toString());
    }

    /**
     * Sets the IMU parameters (mode, angle unit, and acceleration unit)
     */
    protected void initIMU() {
        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )
        );
        imu.initialize(imuParameters);
        imu.resetYaw();
//        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                new Quaternion(
//                        1.0f, // w
//                        0.0f, // x
//                        0.0f, // y
//                        0.0f, // z
//                        0     // acquisitionTime
//                )
//            )
//        );
//        imu.initialize(imuParameters);

//        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
//        imuParameters.mode = BNO055IMU.SensorMode.IMU;
//        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        telemetry.addData("Status", "Init IMU... Please wait");
//        telemetry.update();
//        imu.initialize(imuParameters);
//        telemetry.addData("Status", "IMU Initialized");
//        telemetry.update();
    }

    protected double getZAxisOrientation() {
        Orientation angles;
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        float yaw = angles.firstAngle;
        // yaw += 90.0f;
        // if (yaw > 180.0f) {
        // yaw = 360.0f - yaw;
        // }
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}

