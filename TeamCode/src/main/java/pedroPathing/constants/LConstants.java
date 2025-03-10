package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants {
    static {
        DriveEncoderConstants.forwardTicksToInches = 1;
        DriveEncoderConstants.strafeTicksToInches = 1;
        DriveEncoderConstants.turnTicksToInches = 1;

        DriveEncoderConstants.robot_Width = 1;
        DriveEncoderConstants.robot_Length = 1;

        DriveEncoderConstants.leftFrontEncoderDirection = Encoder.FORWARD;
        DriveEncoderConstants.rightFrontEncoderDirection = Encoder.REVERSE;
        DriveEncoderConstants.leftRearEncoderDirection = Encoder.FORWARD;
        DriveEncoderConstants.rightRearEncoderDirection = Encoder.FORWARD;
//        OTOSConstants.useCorrectedOTOSClass = true;
//        OTOSConstants.hardwareMapName = "OTOS";
//        OTOSConstants.linearUnit = DistanceUnit.INCH;
//        OTOSConstants.angleUnit = AngleUnit.RADIANS;
//        OTOSConstants.offset = new SparkFunOTOS.Pose2D(0.3, 0.6, -Math.PI / 2);
//        OTOSConstants.linearScalar = -1.0831395068493153;
//        OTOSConstants.angularScalar = 1.0;
    }
}




