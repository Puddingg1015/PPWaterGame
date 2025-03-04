package code.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import code.control.IMU.IMUHandler;
import code.hardware.hardwarebase.Drivetrain;

@Config
public class DrivetrainHandler {
    Drivetrain drivetrain;
    Telemetry telemetry;
    IMUHandler imuHandler = null;
    private PIDController drivetrain_controller;
    public static double p = 0.005, i = 0.00006, d = 0.00003;

    public DrivetrainHandler(Drivetrain dt, Telemetry telemetry) {
        this.drivetrain = dt;
        this.telemetry = telemetry;
        this.drivetrain_controller = new PIDController(p, i, d);
    }

    public DrivetrainHandler(Drivetrain dt, Telemetry telemetry, IMU imu) {
        this.drivetrain = dt;
        this.telemetry = telemetry;
        this.imuHandler = new IMUHandler(imu);
        this.drivetrain_controller = new PIDController(p, i, d);
    }

    public void stopMotors() {
        this.drivetrain.Left_Back.setPower(0);
        this.drivetrain.Right_Back.setPower(0);
        this.drivetrain.Left_Front.setPower(0);
        this.drivetrain.Right_Front.setPower(0);
    }

    /**
     * Sets the motor behaviour to run to position
     */
    public void runToPos() {
        this.drivetrain.Left_Back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.drivetrain.Left_Front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.drivetrain.Right_Back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.drivetrain.Right_Front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Resets the motor encoders
     */
    public void resetEncoders() {
        this.drivetrain.Left_Back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.drivetrain.Left_Front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.drivetrain.Right_Back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.drivetrain.Right_Front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.drivetrain.Left_Back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.drivetrain.Left_Front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.drivetrain.Right_Back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.drivetrain.Right_Front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void MoveXY(int forward, int right, double power) {
        Forward(forward, power);
        Right(right, power);
        // if (right == 0) {
        // OptimizedMovePolarVector(forward, (float) Math.PI / 2f);
        // } else if (forward == 0) {
        // OptimizedMovePolarVector(right, 0f);
        // } else {
        // OptimizedMovePolarVector(Math.sqrt((double) (forward * forward + right *
        // right)),
        // (float) Math.atan((double) forward / right));
        // }
    }

    public void OptimizedMovePolarVector(double modulus, double thetaRAD, double power) {
        resetEncoders();
        double xp, xd;
        double yp, yd;
        double max;
        double forward = power;
        double sideways = power;
//        yp = sideways * Math.cos(thetaRAD) - forward * Math.sin(thetaRAD);
//        xp = sideways * Math.sin(thetaRAD) + forward * Math.cos(thetaRAD);
        xp = power;
        yp = power;
//        max = Math.max(Math.abs(xp), Math.abs(yp));
//        xp /= max;
//        yp /= max;
        xd = modulus * Math.cos(thetaRAD);
        yd = modulus * Math.sin(thetaRAD);
        // max = Math.max(Math.max(Math.abs(yp - xp), Math.abs(yp + xp)),
        // Math.max(Math.abs(yp + xp), Math.abs(yp - xp)));
        telemetry.addData("Event", 4.5);
        this.drivetrain.Left_Back.setTargetPosition((int) (yd - xd));
        this.drivetrain.Left_Front.setTargetPosition((int) (yd + xd));
        this.drivetrain.Right_Back.setTargetPosition((int) (yd + xd));
        this.drivetrain.Right_Front.setTargetPosition((int) (yd - xd));
        this.drivetrain.Right_Front.setPower((yp - xp));
        this.drivetrain.Right_Back.setPower((yp + xp));
        this.drivetrain.Left_Front.setPower((yp + xp));
        this.drivetrain.Left_Back.setPower((yp - xp));
        runToPos();
        telemetry.addData("Event", 4.6);
        while (!(!this.drivetrain.Left_Back.isBusy() && !this.drivetrain.Right_Front.isBusy() && !this.drivetrain.Right_Back.isBusy() && !this.drivetrain.Left_Front.isBusy())) {
            telemetry.update();
        }
        stopMotors();
        resetEncoders();
    }

    public void FCOptimizedMoveDisplacementVector(int dx, int dy, double power) {
        this.FCOptimizedMoveDisplacementVector(dx, dy, 0, power);
    }

    public boolean withinThresholdPosition(DcMotorEx motor) {
        return Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) < motor.getTargetPositionTolerance();
    }

    public void movePIDDisplacementVector(int dx, int dy, double power) {
        this.movePIDDisplacementVector(dx, dy, 0, power);
    }
    public void movePIDDisplacementVector(int dx, int dy, int dr, double power) {
        // positive dy goes forward
        dy *= -1;
        resetEncoders();
        double max;
        double angle = -this.imuHandler.getYaw();
        int dx_rotated = (int) (-dx * Math.cos(angle / 180 * Math.PI) - dy * Math.sin(angle / 180 * Math.PI));
        int dy_rotated = (int) (-dx * Math.sin(angle / 180 * Math.PI) + dy * Math.cos(angle / 180 * Math.PI));
        // Sets the robot motor power depending on the pivot sign and the x and y
        // rotation
        this.drivetrain.Right_Front.setTargetPosition(-dr - (dy_rotated - dx_rotated));
        this.drivetrain.Right_Back.setTargetPosition(-dr - (dy_rotated + dx_rotated));
        this.drivetrain.Left_Front.setTargetPosition(dr - (dy_rotated + dx_rotated));
        this.drivetrain.Left_Back.setTargetPosition(dr - (dy_rotated - dx_rotated));
        drivetrain_controller.setPID(p, i, d);
//        runToPos();
        while (!this.withinThresholdPosition(this.drivetrain.Right_Front) && !this.withinThresholdPosition(this.drivetrain.Right_Back)
        && !this.withinThresholdPosition(this.drivetrain.Left_Front) && !this.withinThresholdPosition(this.drivetrain.Left_Back)) {
//            angle = -this.imuHandler.getYaw();
//            dx_rotated = (int) (-dx * Math.cos(angle / 180 * Math.PI) - dy * Math.sin(angle / 180 * Math.PI));
//            dy_rotated = (int) (-dx * Math.sin(angle / 180 * Math.PI) + dy * Math.cos(angle / 180 * Math.PI));
            DcMotor[] motors = new DcMotor[]{drivetrain.Left_Front, drivetrain.Left_Back, drivetrain.Right_Front, drivetrain.Right_Back};
            for (int i = 0; i < 4; i++) {
                double pid = drivetrain_controller.calculate(motors[i].getCurrentPosition(), motors[i].getTargetPosition());
                motors[i].setPower(pid*power);
                telemetry.addData("Pos " + i, motors[i].getCurrentPosition());
                telemetry.addData("Target " + i, motors[i].getTargetPosition());
                telemetry.addData("Cum " + i, motors[i].getPower());
            }
            telemetry.update();
        }
        stopMotors();
        resetEncoders();
    }

    public void FCOptimizedMoveDisplacementVector(int dx, int dy, int dr, double power) {
        resetEncoders();
        double xp;
        double yp;
        double max;
        telemetry.addData("Event", 4.8);
        double angle = -this.imuHandler.getYaw();
        int dx_rotated = (int) (-dx * Math.cos(angle / 180 * Math.PI) - dy * Math.sin(angle / 180 * Math.PI));
        int dy_rotated = (int) (-dx * Math.sin(angle / 180 * Math.PI) + dy * Math.cos(angle / 180 * Math.PI));
        // Sets the robot motor power depending on the pivot sign and the x and y
        // rotation
        this.drivetrain.Right_Front.setTargetPosition(-dr - (dy_rotated - dx_rotated));
        this.drivetrain.Right_Back.setTargetPosition(-dr - (dy_rotated + dx_rotated));
        this.drivetrain.Left_Front.setTargetPosition(dr - (dy_rotated + dx_rotated));
        this.drivetrain.Left_Back.setTargetPosition(dr - (dy_rotated - dx_rotated));
//        this.drivetrain.Right_Front.setPower(-dr - (dy_rotated - dx_rotated));
//        this.drivetrain.Right_Back.setPower(-dr - (dy_rotated + dx_rotated));
//        this.drivetrain.Left_Front.setPower( dr - (dy_rotated + dx_rotated));
//        this.drivetrain.Left_Back.setPower( dr - (dy_rotated - dx_rotated));
        double rfp = -dr - (dy_rotated - dx_rotated);
        double rbp = -dr - (dy_rotated + dx_rotated);
        double lfp = dr - (dy_rotated + dx_rotated);
        double lbp = dr - (dy_rotated - dx_rotated);
        max = Math.max(Math.max(Math.abs(rfp), Math.abs(rbp)), Math.max(Math.abs(lfp), Math.abs(lbp)));
        rfp /= max;
        rbp /= max;
        lfp /= max;
        lbp /= max;
        rfp *= power;
        rbp *= power;
        lfp *= power;
        lbp *= power;
        this.drivetrain.Right_Front.setPower(rfp);
        this.drivetrain.Right_Back.setPower(rbp);
        this.drivetrain.Left_Front.setPower(lfp);
        this.drivetrain.Left_Back.setPower(lbp);
        runToPos();
        telemetry.addData("Event", 4.9);
        while (!(!this.drivetrain.Left_Back.isBusy() && !this.drivetrain.Right_Front.isBusy() && !this.drivetrain.Right_Back.isBusy() && !this.drivetrain.Left_Front.isBusy())) {
            angle = -this.imuHandler.getYaw();
            dx_rotated = (int) (-dx * Math.cos(angle / 180 * Math.PI) - dy * Math.sin(angle / 180 * Math.PI));
            dy_rotated = (int) (-dx * Math.sin(angle / 180 * Math.PI) + dy * Math.cos(angle / 180 * Math.PI));
            rfp = -dr - (dy_rotated - dx_rotated);
            rbp = -dr - (dy_rotated + dx_rotated);
            lfp = dr - (dy_rotated + dx_rotated);
            lbp = dr - (dy_rotated - dx_rotated);
            max = Math.max(Math.max(Math.abs(rfp), Math.abs(rbp)), Math.max(Math.abs(lfp), Math.abs(lbp)));
            rfp /= max;
            rbp /= max;
            lfp /= max;
            lbp /= max;
            rfp *= power;
            rbp *= power;
            lfp *= power;
            lbp *= power;
            this.drivetrain.Right_Front.setPower(rfp);
            this.drivetrain.Right_Back.setPower(rbp);
            this.drivetrain.Left_Front.setPower(lfp);
            this.drivetrain.Left_Back.setPower(lbp);
            telemetry.update();
        }
        stopMotors();
        resetEncoders();
    }

    /**
     * Moves forward using encoders acording to the distance variable
     */
    public void Forward(int distance, double power) {
        resetEncoders();
        this.drivetrain.Left_Back.setTargetPosition(distance);
        this.drivetrain.Left_Front.setTargetPosition(distance);
        this.drivetrain.Right_Back.setTargetPosition(distance);
        this.drivetrain.Right_Front.setTargetPosition(distance);
        this.drivetrain.Left_Back.setPower(power);
        this.drivetrain.Left_Front.setPower(power);
        this.drivetrain.Right_Front.setPower(power);
        this.drivetrain.Right_Back.setPower(power);
        runToPos();
        while (!(!this.drivetrain.Left_Back.isBusy() && !this.drivetrain.Right_Front.isBusy() && !this.drivetrain.Right_Back.isBusy() && !this.drivetrain.Left_Front.isBusy())) {
            telemetry.update();
        }
        stopMotors();
        resetEncoders();
    }

    /**
     * Moves backwards using encoders acording to the distance variable
     */
    public void Backward(int distance, double power) {
        resetEncoders();
        this.drivetrain.Left_Back.setTargetPosition(-distance);
        this.drivetrain.Left_Front.setTargetPosition(-distance);
        this.drivetrain.Right_Back.setTargetPosition(-distance);
        this.drivetrain.Right_Front.setTargetPosition(-distance);
        this.drivetrain.Left_Back.setPower(-power);
        this.drivetrain.Left_Front.setPower(-power);
        this.drivetrain.Right_Front.setPower(-power);
        this.drivetrain.Right_Back.setPower(-power);
        runToPos();
        while (!(!this.drivetrain.Left_Back.isBusy() && !this.drivetrain.Right_Front.isBusy() && !this.drivetrain.Right_Back.isBusy() && !this.drivetrain.Left_Front.isBusy())) {
            telemetry.update();
        }
        stopMotors();
        resetEncoders();
    }

    /**
     * Moves left using encoders acording to the distance variable
     */
    public void Left(int distance, double power) {
        resetEncoders();
        this.drivetrain.Left_Back.setTargetPosition(distance);
        this.drivetrain.Left_Front.setTargetPosition(-distance);
        this.drivetrain.Right_Back.setTargetPosition(-distance);
        this.drivetrain.Right_Front.setTargetPosition(distance);
        this.drivetrain.Left_Back.setPower(power);
        this.drivetrain.Left_Front.setPower(-power);
        this.drivetrain.Right_Front.setPower(power);
        this.drivetrain.Right_Back.setPower(-power);
        runToPos();
        while (!(!this.drivetrain.Left_Back.isBusy() && !this.drivetrain.Right_Front.isBusy() && !this.drivetrain.Right_Back.isBusy() && !this.drivetrain.Left_Front.isBusy())) {
            telemetry.update();
        }
        stopMotors();
        resetEncoders();
    }

    /**
     * Moves right using encoders acording to the distance variable
     */
    public void Right(int distance, double power) {
        resetEncoders();
        this.drivetrain.Left_Back.setTargetPosition(-distance);
        this.drivetrain.Left_Front.setTargetPosition(distance);
        this.drivetrain.Right_Back.setTargetPosition(distance);
        this.drivetrain.Right_Front.setTargetPosition(-distance);
        this.drivetrain.Left_Back.setPower(-power);
        this.drivetrain.Left_Front.setPower(power);
        this.drivetrain.Right_Front.setPower(-power);
        this.drivetrain.Right_Back.setPower(power);
        runToPos();
        while (!(!this.drivetrain.Left_Back.isBusy() && !this.drivetrain.Right_Front.isBusy() && !this.drivetrain.Right_Back.isBusy() && !this.drivetrain.Left_Front.isBusy())) {
            telemetry.update();
        }
        stopMotors();
        resetEncoders();
    }
}
