package code.hardware.hardwarebase;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {

    protected HardwareMap hardware;
    public DcMotorEx Right_Front;
    public DcMotorEx Right_Back;
    public DcMotorEx Left_Front;
    public DcMotorEx Left_Back;


    public Drivetrain(HardwareMap hardware) {
        this.hardware = hardware;
    }

    public void initializeDriveTrain() {
        this.Right_Front = this.hardware.get(DcMotorEx.class, "FrontRight"); // 2
        this.Right_Back = this.hardware.get(DcMotorEx.class, "BackRight"); // 3
        this.Left_Front = this.hardware.get(DcMotorEx.class, "FrontLeft"); // 0
        this.Left_Back = this.hardware.get(DcMotorEx.class, "BackLeft"); // 1
//        Right_Back.setDirection(DcMotorEx.Direction.REVERSE);
        Right_Front.setDirection(DcMotorEx.Direction.REVERSE);
//        Left_Front.setDirection(DcMotorEx.Direction.REVERSE);
        Right_Front.setDirection(DcMotorEx.Direction.REVERSE);
        Left_Back.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Left_Front.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Right_Back.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Right_Front.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }


}