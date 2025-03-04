package code.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import code.execution.autonomous.base.TigerCavemanAutoBase;

@Autonomous(name = "AngularTest")
public class AngularTest extends TigerCavemanAutoBase {

    @Override
    protected void standardBehaviour() {
        this.fieldCentricPowerVectorMovement(-0.6, -0.4);
        sleep(750);
        ElapsedTime time = new ElapsedTime();
        while (time.milliseconds() < 3000) {
            this.fieldCentricPowerVectorMovement(0, -0.6, 0.4);
        }
//        this.drivetrainHandler.stopMotors();
        time.reset();
        while (time.milliseconds() < 3000) {
            this.fieldCentricPowerVectorMovement(0, 0.6, -0.4);
        }
        this.fieldCentricPowerVectorMovement(0.2, 0.6);
        sleep(800);
        this.drivetrainHandler.stopMotors();
        this.endAuto();
    }
}
