// Main Bot TeleOp v0.1
package code.execution;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import code.control.identifiers.MotorType;
import code.control.identifiers.ZeroPowerBehaviourInputMode;
import code.hardware.DifferentialClaw;
import code.hardware.ExArm;
import code.hardware.ZDClaw;
import code.hardware.ZauxArm;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Deprecated
@TeleOp(name = "Tiger Tele Op v4.1", group = "TELEOP")
public class TigerTeleOp extends LinearOpMode {

    enum SpeedMode {
        TURBO, STANDARD, PRECISION
    }

    enum AprilTag {
        NULL, BLU_LEFT, BLU_CENTER, BLU_RIGHT, RED_LEFT, RED_CENTER, RED_RIGHT
    }

    protected DcMotor Right_Front;
    protected DcMotor Right_Back;
    protected DcMotor Left_Front;
    protected DcMotor Left_Back;
    protected DifferentialClaw claw;
    protected ZDClaw zclaw;
    protected ExArm arm;
    protected ZauxArm zarm;
    protected IMU imu;
    protected Limelight3A limelight;

    protected AprilTagProcessor myAprilTagProcessor;
    protected VisionPortal myVisionPortal;
    protected double joelmode = 1.0;

    /* CONSTS */
    final double JOEL_MULTI = 0.3;
    final double TURBO_MULTI = 1.5;
    final double PRECISION_MULTI = 0.320;
    final double STD_SPEED = 0.69;
    @Deprecated
    final double ARM_AMB_POWER = 0.0;
    final double CLAW_DELTA = 0.05;
    final double CLAW_TRIGGER_INV_SENS = 0.3;
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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Right_Front = hardwareMap.get(DcMotor.class, "FrontRight"); // 2
        Right_Back = hardwareMap.get(DcMotor.class, "BackRight"); // 3
        Left_Front = hardwareMap.get(DcMotor.class, "FrontLeft"); // 0
        Left_Back = hardwareMap.get(DcMotor.class, "BackLeft"); // 1
//    lClaw = hardwareMap.get(Servo.class, "lClaw");
//    rClaw = hardwareMap.get(Servo.class, "rClaw");
//    launcher = hardwareMap.get(Servo.class, "launcher");
        arm = new ExArm();
//        arm.setActuators(
//                hardwareMap.get(DcMotor.class, "Arm"),
//                hardwareMap.get(Servo.class, "Linkage")
//        );
        arm.setActuators(
                hardwareMap.get(DcMotorEx.class, "Arm"),
                hardwareMap.get(DcMotorEx.class, "Spool"),
                hardwareMap.get(DigitalChannel.class, "MagneticSensor"),
                hardwareMap.get(DcMotorEx.class, "ZAux")
        );
        zarm = new ZauxArm();
        zarm.setActuators(
                hardwareMap.get(DcMotor.class, "Arm"),
                hardwareMap.get(DcMotorEx.class, "ZAux")
        );
        this.arm.actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.arm.actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        claw = new DifferentialClaw(
                hardwareMap.get(Servo.class, "MainClaw"),
                hardwareMap.get(Servo.class, "LeftClaw"),
                hardwareMap.get(Servo.class, "RightClaw")
        );
        //        claw = new DoubleClaw(
//                hardwareMap.get(Servo.class, "LeftClaw"),
//                hardwareMap.get(Servo.class, "RightClaw")
//        );
//        claw = new N2DClaw(hardwareMap.get(Servo.class, "MainClaw"), hardwareMap.get(Servo.class, "Wrist"));

        zclaw = new ZDClaw(hardwareMap.get(Servo.class, "LeftClaw"), hardwareMap.get(Servo.class, "RightClaw"));

        imu = hardwareMap.get(IMU.class, "imu");

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
        Close_Claw();
        // launcher.setPosition(0.9);
        initIMU();
        this.extendedSetup();
        // initAprilTag();
        waitForStart();
        double arm_multi = 0.4;
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                double arm_power;
                if (Math.abs(gamepad2.left_stick_y) < Math.abs(ARM_AMB_POWER)) {
                    arm_power = -ARM_AMB_POWER;
                } else {
                    if (gamepad2.left_stick_button) {
                        arm_multi = 1;
                    } else {
                        arm_multi = 0.69;
                    }
                    arm_power = joelmode * arm_multi * gamepad2.left_stick_y;
                }
//                 soft stop
                if (gamepad1.left_bumper) {
                    speed_mode = SpeedMode.PRECISION;
                } else if (gamepad1.right_bumper) {
                    speed_mode = SpeedMode.TURBO;
                } else {
                    speed_mode = SpeedMode.STANDARD;
                }
                changeSpeed(speed_mode);
                if (gamepad1.y) {
                    initIMU();
                }
                if (gamepad2.x) {
                    joelmode = JOEL_MULTI;
                    // toggleJoelMode();
                } else {
                    joelmode = 1;
                }
                this.clawActions();
//                if (gamepad2.left_trigger > CLAW_TRIGGER_INV_SENS) {
//                    this.claw.close();
//                }
                // negative is extend, positive is
                if (arm.getRoughArmPosition() < -900) {
//                    arm.powerSpool(0.5); // retracts
                } else if (arm.getRoughArmPosition() < -1000) {
                    arm.setPower(MotorType.PRIMARY,0.3);
                } else {
                    arm.powerSpool(joelmode * (-gamepad2.right_trigger + gamepad2.left_trigger));
                    arm.setPower(MotorType.PRIMARY, (float) arm_power);
                }

                //                zarm.setPower(MotorType.SECONDARY, gamepad2.right_stick_y);
                if (gamepad2.y) {
                    if (arm.zero_power_behaviour_mode == ZeroPowerBehaviourInputMode.VELOCITYINPUT) {
                        arm.zero_power_behaviour_mode = ZeroPowerBehaviourInputMode.POSITIONINPUT;
                    } else {
                        arm.zero_power_behaviour_mode = ZeroPowerBehaviourInputMode.VELOCITYINPUT;
                    }
                }
                if (gamepad2.dpad_up) {
//                    arm.changePosition(10, 0.6);
//                    arm.runUsingPower();
                    arm.setPower(MotorType.PRIMARY, -0.1); // 0.41
//                    sleep(69);
//                    arm.stop(MotorType.PRIMARY);
                }
                if (gamepad2.dpad_down) {
                    arm.setPower(MotorType.PRIMARY, 0.1); // -0.41
//                    sleep(69);
//                    arm.stop(MotorType.PRIMARY);
//                    arm.changePosition(-10, -0.6);
//                    arm.runUsingPower();
                }
//                if (gamepad2.dpad_left && gamepad2.dpad_right) {
//                    this.arm.actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    this.arm.actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                }
//                if (gamepad2.dpad_down) {
//                    arm.partialArm(1);
//                }
//                if (gamepad2.dpad_left) {
//                    arm.partialArm(2);
//                }
//                if (gamepad2.dpad_up) {
//                    arm.partialArm(3);
//                }
//                if (gamepad2.dpad_right) {
//                    arm.partialArm(4);
//                }
                // driving();
//                dumbDriveBecauseIMUisDed();
                fieldCentricDriving();
                // telemetryAprilTag();

                telemetry.addData("Voltage", hardwareMap.voltageSensor.iterator().next().getVoltage());
                telemetry.addData("Drivetrain Mode", speed_mode);
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                telemetry.addData("Gyroscope Measurement", getZAxisOrientation());
//                telemetry.addData("IMU Gyroscope Calibration Status", imu.isGyroCalibrated());
//                if (!imu.isGyroCalibrated()) {
//                    telemetry.addData("WARNING", "IMU OUT OF SYNC; RECALIBRATE AS SOON AS POSSIBLE");
//                }
                telemetry.addData("Gyro Yaw", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.addData("Gyro Roll", orientation.getRoll(AngleUnit.DEGREES));
                telemetry.addData("Gyro Pitch", orientation.getPitch(AngleUnit.DEGREES));
                telemetry.addData("Gyroscope Measurement", getZAxisOrientation());

                telemetry.addData("Arm Power", arm.getPower(MotorType.PRIMARY));
                telemetry.addData("Arm Position", arm.getRoughArmPosition());
                telemetry.addData("Arm Power", arm.getPower(MotorType.PRIMARY));
                telemetry.addData("Arm Velocity", arm.getVelocity());
                telemetry.addData("Arm Adjustment Power", arm.getAdjustmentPower());
                telemetry.addData("Spool Amps", arm.getSpoolCurrentDraw());
                telemetry.addData("Spool Position", arm.getSpoolRoughPosition());
                telemetry.addData("Spool Power", arm.getPower(MotorType.SECONDARY));
                telemetry.addData("Butt Toucher Status", arm.toucherIsPressed());
//                telemetry.addData("Claw Wrist Position", claw.getWristPos());
                telemetry.addData("Left", claw.getLeftPosition());
                telemetry.addData("Right", claw.getRightPosition());
                telemetry.addData("Claw Status", claw.getStatus());
                telemetry.addData("(RB) Drivetrain ds reading", this.Right_Back.getCurrentPosition());
                try {
                    telemetry.addData("Zaux Amps", zarm.getAuxCurrentDraw());
                    telemetry.addData("Zaux Position", zarm.getAuxRoughPosition());
                    telemetry.addData("Zaux Power", zarm.getPower(MotorType.SECONDARY));
                } catch (Exception e) {
                    telemetry.addData("Zaux not initialized", ":!");
                }
                telemetry.update();
            }
        }
    }

    protected void extendedSetup() {

    }

    protected void clawActions() {
        if (gamepad2.a) {
            this.claw.open();
//            this.zclaw.open();
        }
//                if (gamepad2.right_trigger > CLAW_TRIGGER_INV_SENS) {
//                    this.claw.open();
//                }
        if (gamepad2.b) {
            this.claw.close();
//            this.zclaw.close();
        }
//        this.claw.wrist_incr = joelmode * 0.1;
//        if (gamepad2.right_bumper) {
//            this.claw.incrWrist();
//        } else if (gamepad2.left_bumper) {
//            this.claw.decrWrist();
//        }
        double claw_multi = 1.0;
        if (gamepad2.right_stick_button) {
            claw_multi = 2.0;
        }
        claw.rotateSwivel(-gamepad2.right_stick_x*CLAW_DELTA*claw_multi);
        claw.rotatePos(-gamepad2.right_stick_y*CLAW_DELTA*claw_multi);
//        if (gamepad2.dpad_left) {
//            claw.setWristPos(0);
//        } else if (gamepad2.dpad_right) {
//            claw.setWristPos(0.5);
//        }
    }

    protected void toggleJoelMode() {
        if (joelmode == 1) {
            joelmode = JOEL_MULTI;
        } else {
            joelmode = 1;
        }
    }



    protected void dumbDriveBecauseIMUisDed() {
        telemetry.addData("dumb", "bot");
        vert = (gamepad1.left_stick_y * STD_SPEED / speed_div);
        horizontal = -gamepad1.left_stick_x * STD_SPEED / speed_div;
        pivot = -(gamepad1.right_stick_x * STD_SPEED * pivot_sign) / speed_div;
        Right_Front.setPower(-pivot - (vert - horizontal));
        Right_Back.setPower(-pivot - (vert + horizontal));
        Left_Front.setPower(pivot - (vert + horizontal));
        Left_Back.setPower(pivot - (vert - horizontal));
    }       

    protected void fieldCentricDriving() {
        telemetry.addData("This", "Works");
        double angle;
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
        // rotation
        Right_Front.setPower(-pivot - (y_rotated - x_rotated));
        Right_Back.setPower(-pivot - (y_rotated + x_rotated));
        Left_Front.setPower(pivot - (y_rotated + x_rotated));
        Left_Back.setPower(pivot - (y_rotated - x_rotated));
        // telemetry.update();
        // if (gamepad1.b) {
        // initIMU();
        // }
        // telemetry.update();
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
    }

    protected double getZAxisOrientation() {
//        Orientation angles;
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        float yaw = angles.firstAngle;
        // yaw += 90.0f;
        // if (yaw > 180.0f) {
        // yaw = 360.0f - yaw;
        // }
//        return yaw;
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    protected void changeSpeed(SpeedMode speed_mode) {
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

    /**
     * Describe this function...
     */
    protected void driving() {
        Left_Back.setPower(0.75 * (gamepad1.left_stick_y + gamepad1.left_stick_x + -gamepad1.right_stick_x));
        Left_Front.setPower(0.75 * (gamepad1.left_stick_y + -gamepad1.left_stick_x + -gamepad1.right_stick_x));
        Right_Back.setPower(0.75 * (gamepad1.left_stick_y + -gamepad1.left_stick_x + gamepad1.right_stick_x));
        Right_Front.setPower(0.75 * (gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
    }
}

/* END DriverbotTeleop_VFARII.java */