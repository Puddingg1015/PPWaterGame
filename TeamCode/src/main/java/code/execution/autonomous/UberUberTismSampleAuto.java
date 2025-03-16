package code.execution.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import code.hardware.DrivetrainHandler;
import code.hardware.hardwarebase.Drivetrain;

@Autonomous(group = "AUTO")
public class UberUberTismSampleAuto extends LinearOpMode {
    protected Drivetrain drivetrain;
    protected DrivetrainHandler drivetrainHandler;

    @Override
    public void runOpMode() {
        this.drivetrain = new Drivetrain(hardwareMap);
        this.drivetrain.initializeDriveTrain();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        if (opModeIsActive()) {
            drivetrainHandler.Left(400, 0.5);
        }
    }
}
