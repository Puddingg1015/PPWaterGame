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
@TeleOp(name = "Drivetrain Forward PID Test")
public class DrivetrainPIDLoop extends LinearOpMode {

    Drivetrain drivetrain;
    private PIDController controller;
    public static double p = 0.001, i = 0.00006, d = 0.00003;
    public static int target = 0;

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
            for (int i = 0; i < 4; i++) {
                double pid = controller.calculate(motors[i].getCurrentPosition(), target);
                double power = pid;
                target += (int) gamepad1.right_stick_y * 10;
                motors[i].setPower(power);
                telemetry.addData("Pos " + i, motors[i].getCurrentPosition());
                telemetry.addData("Target " + i, target);
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
