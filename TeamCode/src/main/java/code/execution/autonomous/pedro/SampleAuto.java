package code.execution.autonomous.pedro;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

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
    private Telemetry telemetryA;

    public static double RADIUS = 10;

    private Follower follower;

    private PathChain path;

    /**
     * This initializes the Follower and creates the PathChain for the "circle". Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        path = this.buildPath();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
    }

    @Override
    public void start() {
        follower.followPath(path);
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();
        if (follower.atParametricEnd()) {
            telemetryA.addData("End", true);
        }
        follower.telemetryDebug(telemetryA);
    }

    public PathChain buildPath() {

        return follower.pathBuilder().addPath(
                        // Line 1
                        new BezierLine(
                                new Point(8.000, 80.000, Point.CARTESIAN),
                                new Point(30.000, 80.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(360))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(30.000, 80.000, Point.CARTESIAN),
                                new Point(30.000, 129.000, Point.CARTESIAN),
                                new Point(14.000, 129.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(14.000, 129.000, Point.CARTESIAN),
                                new Point(31.000, 121.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(0))
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(31.000, 121.000, Point.CARTESIAN),
                                new Point(14.000, 129.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(14.000, 129.000, Point.CARTESIAN),
                                new Point(31.000, 129.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(0))
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(31.000, 129.000, Point.CARTESIAN),
                                new Point(14.000, 129.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(14.000, 129.000, Point.CARTESIAN),
                                new Point(51.000, 115.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(90))
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(51.000, 115.000, Point.CARTESIAN),
                                new Point(14.000, 129.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(315))
                .addPath(
                        // Line 9
                        new BezierCurve(
                                new Point(14.000, 129.000, Point.CARTESIAN),
                                new Point(60.000, 130.000, Point.CARTESIAN),
                                new Point(65.000, 100.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(-80))
                .addPath(
                        // Line 10
                        new BezierCurve(
                                new Point(65.000, 100.000, Point.CARTESIAN),
                                new Point(50.000, 130.000, Point.CARTESIAN),
                                new Point(14.000, 129.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-80), Math.toRadians(315))
                .addPath(
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
}