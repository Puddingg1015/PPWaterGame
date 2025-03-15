// Main Bot TeleOp v0.1
package code.execution.autonomous.base;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import teamcode.IMU.IMUHandler;
import code.control.identifiers.OpModeEndBehaviour;
import code.hardware.hardwarebase.Drivetrain;
import code.hardware.DrivetrainHandler;
import code.hardware.N2DClaw;
import code.hardware.PulleyArm;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@Autonomous(name = "!!AUTOBASE", group = "AUTOBASE")
public class TigerCavemanAutoBase extends LinearOpMode {

    Drivetrain drivetrain;
    protected DrivetrainHandler drivetrainHandler;
    protected N2DClaw claw;
    protected PulleyArm arm;
    protected IMU imu;
    protected AprilTagProcessor myAprilTagProcessor;
    protected VisionPortal myVisionPortal;
//    IMUHandler imuHandler;

    /* CONSTS */
    final double TURBO_MULTI = 1.5;
    final double PRECISION_MULTI = 0.375;
    final double STD_SPEED = 0.69;
    @Deprecated
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


    public final double PI = 3.14159265358979323;

    @Override
    public void runOpMode() {

        /* Initialization */
        this.drivetrain = new Drivetrain(hardwareMap);
        this.drivetrain.initializeDriveTrain();
//        arm = new LinkageArm();
//        arm.setActuators(
//                hardwareMap.get(DcMotor.class, "Arm"),
//                hardwareMap.get(Servo.class, "Linkage")
//        );
//        claw = new Claw(hardwareMap.get(Servo.class, "LeftClaw"));
        arm = new PulleyArm();
        arm.setActuators(
                hardwareMap.get(DcMotor.class, "Arm"),
                hardwareMap.get(DcMotorEx.class, "Spool"),
                hardwareMap.get(DigitalChannel.class, "MagneticSensor")
        );
        this.arm.actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.arm.actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        claw = new DoubleClaw(
//                hardwareMap.get(Servo.class, "LeftClaw"),
//                hardwareMap.get(Servo.class, "RightClaw")
//        );

        claw = new N2DClaw(hardwareMap.get(Servo.class, "MainClaw"), hardwareMap.get(Servo.class, "Wrist"));

        this.imu = hardwareMap.get(IMU.class, "imu");
        this.initIMU();
//        this.imuHandler = new IMUHandler(imu);
        this.drivetrainHandler = new DrivetrainHandler(this.drivetrain, this.telemetry);


        /* Initialization Telemetry */
        telemetry.addData("Camera Stream", "ACTIVE");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        /* Wait for the match to start */
        waitForStart();
        claw.close();
        sleep(1000);
        this.fieldCentricPowerVectorMovement(0, -0.3, 0);
        sleep(500);
        this.drivetrainHandler.stopMotors();
        telemetry.addData("Event", -2);
        telemetry.update();
        arm.changePosition(420, 0.3);
//        arm.changePosition(600 - 567, 0.4);
//        sleep(300);
        telemetry.addData("Event", -1);
        telemetry.update();
        claw.setWristPos(1);
        if (opModeIsActive()) {
            telemetry.addData("Event", 0);
            telemetry.update();
            try {
                telemetry.addData("Event", 1);
                telemetry.update();
                this.standardBehaviour();
                telemetry.addData("Event", 2);
                telemetry.update();
            } catch (Exception e) {
                telemetry.addData("ERROR: ", e);
                telemetry.update();
                throw new RuntimeException(e);
            }
        }
        this.endAuto();


    }

    protected void standardBehaviour() throws Exception {
        throw new InternalError("Implement this class");
    }

    protected void extendArm() {
        this.arm.powerSpool(-1);
        sleep(500);
        this.arm.powerSpool(0);
    }

    protected void retractArm() {
        this.arm.powerSpool(1);
        sleep(500);
        this.arm.powerSpool(0);
    }


    protected void endAuto() {
        this.drivetrainHandler.stopMotors();
        sleep(30000);
    }

    protected void endAuto(OpModeEndBehaviour end_behaviour) {
        switch (end_behaviour) {
            case STANDARD: {
                this.endAuto();
            } case TIMEOUT: {
                this.endAuto();
            } case BREAK: {
                this.drivetrainHandler.stopMotors();
                this.stop();
            } default: {
                this.endAuto();
            }
        }
    }

    protected void fieldCentricPowerVectorMovement(double x, double y) {
        this.fieldCentricPowerVectorMovement(x, y, 0);
    }

    /**
     * Field Centric Driving
     */
    protected void fieldCentricPowerVectorMovement(double x, double y, double r) {
        telemetry.addData("This", "Works");
        double angle;
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

    protected void fieldCentricDisplacementVectorMovement(double power, int dx, int dy, int dr) {
        this.drivetrainHandler.resetEncoders();
        this.drivetrainHandler.runToPos();
        telemetry.addData("This", "Works");
        double angle;
        int dx_rotated;
        int dy_rotated;
        angle = -getZAxisOrientation();
        // Sets the axis parameters depending on the gamepad stick value and speed
        // division variable
        // Odometry angle calculations for x and y rotations
        dx_rotated = (int) (-dx * Math.cos(angle / 180 * Math.PI) - dy * Math.sin(angle / 180 * Math.PI));
        dy_rotated = (int) (-dx * Math.sin(angle / 180 * Math.PI) + dy * Math.cos(angle / 180 * Math.PI));
        // Sets the robot motor power depending on the pivot sign and the x and y
        // rotation
        this.drivetrain.Right_Front.setTargetPosition(-dr - (dy_rotated - dx_rotated));
        this.drivetrain.Right_Back.setTargetPosition(-dr - (dy_rotated + dx_rotated));
        this.drivetrain.Left_Front.setTargetPosition(dr - (dy_rotated + dx_rotated));
        this.drivetrain.Left_Back.setTargetPosition(dr - (dy_rotated - dx_rotated));
        this.drivetrain.Right_Front.setPower(-dr - (dy_rotated - dx_rotated));
        this.drivetrain.Right_Back.setPower(-dr - (dy_rotated + dx_rotated));
        this.drivetrain.Left_Front.setPower(dr - (dy_rotated + dx_rotated));
        this.drivetrain.Left_Back.setPower(dr - (dy_rotated - dx_rotated));
        drivetrainHandler.runToPos();
        while (!(!this.drivetrain.Left_Back.isBusy() && !this.drivetrain.Right_Front.isBusy() && !this.drivetrain.Right_Back.isBusy() && !this.drivetrain.Left_Front.isBusy())) {
            telemetry.update();
        }
        drivetrainHandler.stopMotors();
        drivetrainHandler.resetEncoders();
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

    /**
     * /**
     * Opens claw and waits a bit
     */
    protected void Open_Claw() {
//    lClaw.setPosition(0.20);
//    rClaw.setPosition(0.40);
        sleep(100);
    }

    /**
     * Lowers arm and closes claw and waits a bit longer
     */
    protected void Close_Claw() {

//    lClaw.setPosition(0.59);
//    rClaw.setPosition(0.01);
        sleep(100);
    }


}

