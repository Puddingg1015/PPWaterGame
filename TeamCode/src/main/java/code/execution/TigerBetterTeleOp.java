// Main Bot TeleOp v0.1
package code.execution;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import code.control.identifiers.MotorType;
import code.control.identifiers.OrthoType;
import code.control.identifiers.SpeedMode;
import code.control.identifiers.ZeroPowerBehaviourInputMode;
import code.hardware.DifferentialClaw;
import code.hardware.ExArm;
import code.hardware.ZDClaw;
import code.hardware.ZauxArm;
import code.vision.GameObjectCVProcessor;

@Config
@TeleOp(name = "Tiger Better TeleOp v5.1", group = "TELEOP")
public class TigerBetterTeleOp extends LinearOpMode {

    protected DcMotor Right_Front;
    protected DcMotor Right_Back;
    protected DcMotor Left_Front;
    protected DcMotor Left_Back;
    protected Servo sweeper;
    protected DifferentialClaw claw;
    protected ZDClaw zclaw;
    protected ExArm arm;
    protected ZauxArm zarm;
    protected IMU imu;
    protected Limelight3A limelight;
    protected GameObjectCVProcessor cvProcessor;
    ElapsedTime action_cooldown;
    protected Follower follower;
    protected final Pose startPose = new Pose(0,0,0);
    protected double joelmode = 1.0;

    /* CONSTS */
    final double JOEL_MULTI = 0.4;
    final double TURBO_MULTI = 1.5;
    final double PRECISION_MULTI = 0.320;
    final double STD_SPEED = 0.69;
    @Deprecated
    final double ARM_AMB_POWER = 0.0;
    final double CLAW_TRIGGER_INV_SENS = 0.3;
    final double CLAW_DELTA = 0.04;
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
    final double RIGHTCLAW_POSTFIX = 0;
//    final double CLAW_180 = 1.00000;

    /* GLOBAL VARIABLES */
    double vert;
    double horizontal;
    double speed_div = 1.0;
    double pivot;
    int forward_;
    double pivot_sign = 1.0;
    boolean USE_WEBCAM = true;
    boolean hold_claw = true;
    boolean perma_arm_down = false;

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
        arm = new ExArm();
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
        this.sweeper = hardwareMap.get(Servo.class, "Sweeper");
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
        this.initIMU();
        this.initVisionProcessor();
        this.cvProcessor.safeStart();
        this.contractSweeper();
        this.extendedSetup();
        waitForStart();
        action_cooldown = new ElapsedTime();
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

                if (gamepad2.a && gamepad2.b) {
                    while(opModeIsActive()) {
                        this.arm.setPower(MotorType.PRIMARY, 1);
                    }
                }
                
                if (gamepad1.right_trigger > 0.69) {
                    this.extendSweeper();
                }
                if (gamepad1.left_trigger > 0.69) {
                    this.contractSweeper();
                }
                if (gamepad2.x) {
                    joelmode = JOEL_MULTI;
                    // toggleJoelMode();
                } else {
                    joelmode = 1;
                }
                this.clawActions();
                // negative is extend, positive is
                if (arm.getRoughArmPosition() < -900 && !(gamepad2.y && gamepad2.x)) {
                    arm.setPower(MotorType.PRIMARY,0.1);
                } else {
                    arm.powerSpool(joelmode * (-gamepad2.right_trigger + gamepad2.left_trigger));
                    arm.setPower(MotorType.PRIMARY, (float) arm_power);
                }

                //                zarm.setPower(MotorType.SECONDARY, gamepad2.right_stick_y);
//                if (gamepad2.y) {
//                    if (arm.zero_power_behaviour_mode == ZeroPowerBehaviourInputMode.VELOCITYINPUT) {
//                        arm.zero_power_behaviour_mode = ZeroPowerBehaviourInputMode.POSITIONINPUT;
//                    } else {
//                        arm.zero_power_behaviour_mode = ZeroPowerBehaviourInputMode.VELOCITYINPUT;
//                    }
//                }
                if (gamepad2.dpad_up) {
                    arm.setPower(MotorType.PRIMARY, -0.1); // 0.41
                }
                if (gamepad2.dpad_down) {
                    arm.setPower(MotorType.PRIMARY, 0.1); // -0.41
                }
                // driving();
//                dumbDriveBecauseIMUisDed();
                fieldCentricDriving();

                telemetry.addData("Voltage", hardwareMap.voltageSensor.iterator().next().getVoltage());
                telemetry.addData("Drivetrain Mode", speed_mode);
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                telemetry.addData("Gyroscope Measurement", getZAxisOrientation());
                telemetry.addData("Gyro Yaw", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.addData("Gyro Roll", orientation.getRoll(AngleUnit.DEGREES));
                telemetry.addData("Gyro Pitch", orientation.getPitch(AngleUnit.DEGREES));
                telemetry.addData("Gyroscope Measurement", getZAxisOrientation());

                telemetry.addData("Streaming", this.cvProcessor.isStreaming());
                telemetry.addData("Press A", "resume stream / freeze screenshot");

                telemetry.addData("Arm Power", arm.getPower(MotorType.PRIMARY));
                telemetry.addData("Arm Position", arm.getRoughArmPosition());
                telemetry.addData("Arm Power", arm.getPower(MotorType.PRIMARY));
                telemetry.addData("Arm Velocity", arm.getVelocity());
                telemetry.addData("Arm Adjustment Power", arm.getAdjustmentPower());
                telemetry.addData("Spool Amps", arm.getSpoolCurrentDraw());
                telemetry.addData("Spool Position", arm.getSpoolRoughPosition());
                telemetry.addData("Spool Power", arm.getPower(MotorType.SECONDARY));
                telemetry.addData("Butt Toucher Status", arm.toucherIsPressed());
                telemetry.addData("Claw Left", claw.getLeftPosition());
                telemetry.addData("Claw Right", claw.getRightPosition());
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

    protected void extendSweeper() {
        this.sweeper.setPosition(0.5);
    }

    protected void contractSweeper() {
        this.sweeper.setPosition(1);
    }

    private void initVisionProcessor() {
        this.cvProcessor = new GameObjectCVProcessor(hardwareMap, "ArmCam");
    }

    protected void extendedSetup() {

    }

    protected void clawActions() {
        if (gamepad2.left_bumper) {
            this.claw.open();
        }
        if (gamepad2.right_bumper) {
            this.claw.close();
        }
        if (gamepad2.dpad_left) {
            this.claw.setDown(OrthoType.HORIZONTAL);
        }
        if (gamepad2.dpad_right) {
            this.claw.setDown(OrthoType.VERTICAL);
        }
        if (gamepad2.x) {
            this.claw.setStraight();
        }
        double orientation_updated_position = CLAW_0 +
                (CLAW_90-CLAW_0)*(
                        gamepad2.b ?
                        90-Math.abs(this.cvProcessor.pipeline.getOrientation())
                        :
                        Math.abs(this.cvProcessor.pipeline.getOrientation())
                )/90;
        if (gamepad1.a) {
            telemetry.addData("Streaming", this.cvProcessor.isStreaming());
            telemetry.addData("Press A", "Stop Streaming");
            if (gamepad2.a && action_cooldown.milliseconds() > 500) {
//                this.cvProcessor.stop();
//                hold_claw = true;
                action_cooldown.reset();
            }
//            this.claw.setRotation(orientation_updated_position, orientation_updated_position + RIGHTCLAW_POSTFIX);
        } else {
            telemetry.addData("Streaming", "OFF");
            telemetry.addData("Press A", "Start Streaming");
            telemetry.addData("Holding Position", "Manual Control Enabled");
            if (gamepad2.a && action_cooldown.milliseconds() > 500) {
//                this.cvProcessor.safeStart();
//                hold_claw = false;
                action_cooldown.reset();
            }
        }

        double claw_multi = 1.0;
        if (gamepad2.right_stick_button) {
            claw_multi = 2.0;
        }
        claw.rotateSwivel(-gamepad2.right_stick_x*CLAW_DELTA*claw_multi);
        claw.rotatePos(-gamepad2.right_stick_y*CLAW_DELTA*claw_multi);

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
        if (speed_mode == SpeedMode.TURBO)
            speed_div = 1.0 / TURBO_MULTI;
        else if (speed_mode == SpeedMode.PRECISION)
            speed_div = 1.0 / PRECISION_MULTI;
        else
            speed_div = 1.0;
    }

    protected void driving() {
        Left_Back.setPower(0.75 * (gamepad1.left_stick_y + gamepad1.left_stick_x + -gamepad1.right_stick_x));
        Left_Front.setPower(0.75 * (gamepad1.left_stick_y + -gamepad1.left_stick_x + -gamepad1.right_stick_x));
        Right_Back.setPower(0.75 * (gamepad1.left_stick_y + -gamepad1.left_stick_x + gamepad1.right_stick_x));
        Right_Front.setPower(0.75 * (gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
    }
}

/* END DriverbotTeleop_VFARII.java */