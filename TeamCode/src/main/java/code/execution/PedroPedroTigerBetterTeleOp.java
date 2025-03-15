// Main Bot TeleOp v0.1
package code.execution;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Drawing;
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
import code.control.identifiers.ZeroPowerBehaviourInputMode;
import code.hardware.DifferentialClaw;
import code.hardware.ExArm;
import code.hardware.ZDClaw;
import code.hardware.ZauxArm;
import code.vision.GameObjectCVProcessor;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "Tiger PEDRO PEDRO TeleOp v6.1", group = "TELEOP")
public class PedroPedroTigerBetterTeleOp extends TigerBetterTeleOp {

    boolean USE_PEDRO_FCD = false;

    @Override
    protected void extendedSetup() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();
    }

    @Override
    protected void initIMU() {
        super.initIMU();
        if (USE_PEDRO_FCD) {
            try {
                follower.startTeleopDrive();
            } catch (NullPointerException ignored) {
            }
        }
    }

    @Override
    protected void fieldCentricDriving() {
        if (this.USE_PEDRO_FCD) {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x, false);
            follower.update();
        } else {
            super.fieldCentricDriving();
        }
        follower.updatePose();
        telemetry.addData("PP", "----------------");
        telemetry.addData("OTOS X", follower.getPose().getX());
        telemetry.addData("OTOS Y", follower.getPose().getY());
        telemetry.addData("Heading (DEG)", Math.toDegrees(follower.getPose().getHeading()));
//        follower.telemetryDebug(telemetry);
        // causes error ^ v
//        Drawing.drawDebug(follower);
        telemetry.addData("STD", "----------------");
    }

}

/* END DriverbotTeleop_VFARII.java */