package code.control.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IMUHandler {
    IMU imu;
    AngleUnit angle_unit;   
    public IMUHandler(IMU imu) {
        this.imu = imu;
        this.setAngleUnit(AngleUnit.DEGREES);
        this.initIMU();
        // stuff
    }

    private void initIMU() {
        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )
        );
        imu.initialize(imuParameters);
        imu.resetYaw();
    }

//    public short getGyroStatus() {
//
//    }
//
    public double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(this.angle_unit);
    }

    public double getPitch() {
        return imu.getRobotYawPitchRollAngles().getPitch(this.angle_unit);
    }

    public double getRoll() {
        return imu.getRobotYawPitchRollAngles().getRoll(this.angle_unit);
    }

    public void setAngleUnit(AngleUnit unit) {
        this.angle_unit = unit;
    }

}
