package code.control;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import code.hardware.hardwarebase.Drivetrain;

@Config
@TeleOp(name = "Drivetrain Angular PID Test")
public class DrivetrainAngularPIDLoop extends LinearOpMode {

    Drivetrain drivetrain;
    private PIDController controller;
    //    public static double p = 0.005, i = 0.00006, d = 0.00003;
    public static double p = 0.005, i = 0.0000001, d = 0.0000008;
    public static int targetX = 0;
    public static int targetY = 0;
    public static int targetR = 0;

    private DcMotorEx arm;
    DcMotor[] motors;
    public void cinit() {
        this.drivetrain = new Drivetrain(hardwareMap);
        this.drivetrain.initializeDriveTrain();
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm = hardwareMap.get(DcMotorEx.class, "Arm");
        this.motors = new DcMotor[]{drivetrain.Left_Front, drivetrain.Left_Back, drivetrain.Right_Front, drivetrain.Right_Back};
        for (int i = 0; i < 4; i++) {
            this.motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    @Override
    public void runOpMode() {
        cinit();
        waitForStart();
        while (opModeIsActive()) {
            cloop();
        }
    }

    public void cloop() {
        if (!gamepad1.a) {

            controller.setPID(p, i, d);
            targetX += (int) gamepad1.right_stick_x * 10;
            targetY += (int) gamepad1.right_stick_y * 10;
            this.drivetrain.Right_Front.setTargetPosition(-targetR - (targetY - targetX));
            this.drivetrain.Right_Back.setTargetPosition(-targetR - (targetY + targetX));
            this.drivetrain.Left_Front.setTargetPosition(targetR - (targetY + targetX));
            this.drivetrain.Left_Back.setTargetPosition(targetR - (targetY - targetX));
            this.motors = new DcMotor[]{drivetrain.Left_Front, drivetrain.Left_Back, drivetrain.Right_Front, drivetrain.Right_Back};
            for (int i = 0; i < 4; i++) {
                double pid = controller.calculate(motors[i].getCurrentPosition(), motors[i].getTargetPosition());
                double power = pid;
                motors[i].setPower(power);
                telemetry.addData("Pos " + i, motors[i].getCurrentPosition());
                telemetry.addData("Target " + i, motors[i].getTargetPosition());
                telemetry.addData("Cum " + i, motors[i].getPower());
            }
        } else {
            for (int i = 0; i < 4; i++) {
                this.motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
        telemetry.update();
    }
}
