package code.execution.autonomous.pedro;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import code.control.identifiers.PathNavStatus;
import code.control.identifiers.PathStatus;
import code.hardware.DifferentialClaw;
import code.hardware.DrivetrainHandler;
import code.hardware.PulleyArm;
import code.hardware.hardwarebase.Drivetrain;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is the Circle autonomous OpMode. It runs the robot in a PathChain that's actually not quite
 * a circle, but some Bezier curves that have control points set essentially in a square. However,
 * it turns enough to tune your centripetal force correction and some of your heading. Some lag in
 * heading is to be expected.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous (name = "Sample Auto", group = "AUTO")
public class SampleAuto extends OpMode {

    protected Drivetrain drivetrain;
    protected DrivetrainHandler drivetrainHandler;
    protected DifferentialClaw claw;
    protected PulleyArm arm;

    public static double RADIUS = 10;

    private Follower follower;

    private PathChain path;
    private PathNavStatus status = PathStatus.START;
    private PathChain scorePreload, moveToScore, grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, grabPickup4, scorePickup4, park;

    /**
     * This initializes the Follower and creates the PathChain for the "circle". Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        arm = new PulleyArm();
        arm.setActuators(
                hardwareMap.get(DcMotor.class, "Arm"),
                hardwareMap.get(DcMotorEx.class, "Spool"),
                hardwareMap.get(DigitalChannel.class, "MagneticSensor")
        );
        this.arm.actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.arm.actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        claw = new DifferentialClaw(
                hardwareMap.get(Servo.class, "MainClaw"),
                hardwareMap.get(Servo.class, "LeftClaw"),
                hardwareMap.get(Servo.class, "RightClaw")
        );
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        this.drivetrainHandler = new DrivetrainHandler(this.drivetrain, this.telemetry, null);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        buildPath();
        telemetry.update();
    }

    @Override
    public void start() {
        status = PathStatus.PRELOAD;
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        if (follower.atParametricEnd()) {
            telemetry.addData("End", true);
        }
        telemetry.addData("path state", status);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        follower.telemetryDebug(telemetry);
        telemetry.update();
    }

    public void buildPath() {
                 scorePreload = follower.pathBuilder().addPath(
                        // Line 1
                        new BezierLine(
                                new Point(8.000, 80.000, Point.CARTESIAN),
                                new Point(30.000, 80.000, Point.CARTESIAN)

                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
                moveToScore = follower.pathBuilder().addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(30.000, 80.000, Point.CARTESIAN),
                                new Point(30.000, 129.000, Point.CARTESIAN),
                                new Point(14.000, 129.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                .build();
                grabPickup1 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(14.000, 129.000, Point.CARTESIAN),
                                new Point(31.000, 121.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(0))
                .build();
                scorePickup1 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(31.000, 121.000, Point.CARTESIAN),
                                new Point(14.000, 129.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                .build();
                grabPickup2 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(14.000, 129.000, Point.CARTESIAN),
                                new Point(31.000, 129.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(0))
                .build();
                scorePickup2 = follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(31.000, 129.000, Point.CARTESIAN),
                                new Point(14.000, 129.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                .build();
                grabPickup3 = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(14.000, 129.000, Point.CARTESIAN),
                                new Point(51.000, 115.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(90))
                .build();
                scorePickup3 = follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(51.000, 115.000, Point.CARTESIAN),
                                new Point(14.000, 129.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(315))
                .build();
                grabPickup4 = follower.pathBuilder().addPath(
                        // Line 9
                        new BezierCurve(
                                new Point(14.000, 129.000, Point.CARTESIAN),
                                new Point(60.000, 130.000, Point.CARTESIAN),
                                new Point(65.000, 100.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(-80))
                .build();
                scorePickup4 = follower.pathBuilder().addPath(
                        // Line 10
                        new BezierCurve(
                                new Point(65.000, 100.000, Point.CARTESIAN),
                                new Point(50.000, 130.000, Point.CARTESIAN),
                                new Point(14.000, 129.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-80), Math.toRadians(315))
                .build();
                 park = follower.pathBuilder().addPath(
                        // Line 11
                        new BezierCurve(
                                new Point(14.000, 129.000, Point.CARTESIAN),
                                new Point(60.000, 130.000, Point.CARTESIAN),
                                new Point(65.000, 100.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(90))
                .build();
     }

    public void setPathStatus(PathNavStatus status) {
        this.status = status;
    }

    public void autonomousPathUpdate() throws Exception {
        if (status.equals(PathStatus.PRELOAD)) {
            follower.followPath(scorePreload);
            setPathStatus(PathStatus.Sample.SAMPLE1_PICKUP);
            if (!follower.isBusy()) {
                /* Score Preload */
                this.arm.setExtension(1000, 1);
                this.drivetrainHandler.Forward(200, 0.5); // 0.5
                arm.changePosition(-75, 0.5);
                this.drivetrainHandler.Backward(200, 0.5);
                claw.open();
                this.arm.setExtension(-1000, 0.9);
                this.claw.close();
                arm.changePosition(150, 0.5);
                /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
            }
        } else if (status.equals(PathStatus.Sample.SAMPLE1_PICKUP)) {/* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

            /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
            if (!follower.isBusy()) {
                /* Score Preload */

                /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                follower.followPath(grabPickup1, true);
                setPathStatus(PathStatus.Sample.SAMPLE1_SCORE);
            }
        } else if (status.equals(PathStatus.Sample.SAMPLE1_SCORE)) {/* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
            if (!follower.isBusy()) {
                /* Grab Sample */

                /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                follower.followPath(scorePickup1, true);
                setPathStatus(PathStatus.Sample.SAMPLE2_PICKUP);
            }
        } else if (status.equals(PathStatus.Sample.SAMPLE2_PICKUP)) {/* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
            if (!follower.isBusy()) {
                /* Score Sample */

                /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                follower.followPath(grabPickup2, true);
                setPathStatus(PathStatus.Sample.SAMPLE2_SCORE);
            }
        } else if (status.equals(PathStatus.Sample.SAMPLE2_SCORE)) {/* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
            if (!follower.isBusy()) {
                /* Grab Sample */

                /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                follower.followPath(scorePickup2, true);
                setPathStatus(PathStatus.Sample.SAMPLE3_PICKUP);
            }
        } else if (status.equals(PathStatus.Sample.SAMPLE3_PICKUP)) {/* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
            if (!follower.isBusy()) {
                /* Score Sample */

                /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                follower.followPath(grabPickup3, true);
                setPathStatus(PathStatus.Sample.SAMPLE3_SCORE);
            }
        } else if (status.equals(PathStatus.Sample.SAMPLE3_SCORE)) {/* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
            if (!follower.isBusy()) {
                /* Grab Sample */

                /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                follower.followPath(scorePickup3, true);
                setPathStatus(PathStatus.PARK);
            }
        } else if (status.equals(PathStatus.PARK)) {/* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
            if (!follower.isBusy()) {
                /* Score Sample */

                /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                follower.followPath(park, true);
                setPathStatus(PathStatus.END);
            }
        } else if (status.equals(PathStatus.END)) {/* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
            if (!follower.isBusy()) {
                /* Level 1 Ascent */

                /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                setPathStatus(PathStatus.BUSY);
            }
        }
        else if (status.equals(PathStatus.BUSY)) {/* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
            if (!follower.isBusy()) {
                /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                setPathStatus(PathStatus.BUSY);
            }
        }
    }


}