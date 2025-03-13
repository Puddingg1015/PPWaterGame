package code.execution.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import code.control.identifiers.OpModeEndBehaviour;
import code.execution.autonomous.base.TigerCavemanAutoBase;

@Autonomous(name = "!!Tiger Caveman Auto -- INNER VER -- DONT USE", group = "AUTO")
@Deprecated
public class TigerCavemanAutoInner extends TigerCavemanAutoBase {

    @Override
    protected void standardBehaviour() {
        this.moveAndPlaceSpecimen();
        this.park();
        this.endAuto(OpModeEndBehaviour.BREAK);
    }

    private void moveAndPlaceSpecimen() {
        this.fieldCentricPowerVectorMovement(0.5, -0.25, 0);
        sleep(1750);
        this.drivetrainHandler.stopMotors();
//        arm.changePosition(567, 0.4);
        sleep(300);
        super.extendArm();
        sleep(1000);
        this.fieldCentricPowerVectorMovement(0, -0.4, 0);
        sleep(500);
        this.drivetrainHandler.stopMotors();
        arm.changePosition(200, 1);
//        this.fieldCentricPowerVectorMovement(0, 0.1, 0);
        sleep(500);
        this.drivetrainHandler.stopMotors();
        arm.changePosition(-200-369-31, 1);
        sleep(400);
        this.fieldCentricPowerVectorMovement(0, 0.3, 0);
        super.retractArm();
        sleep(500);
        this.drivetrainHandler.stopMotors();
        this.claw.open();
        sleep(800);
        arm.changePosition(100, 0.5);
        this.claw.close();
    }

    private void park() {
        this.fieldCentricPowerVectorMovement(-0.64, 0.15, 0);
        sleep(3000);
        this.fieldCentricPowerVectorMovement(0.35, -0.425, 0);
        sleep(2000);
        this.fieldCentricPowerVectorMovement(0.4, 0, 0.37);
        sleep(1000);
        this.fieldCentricPowerVectorMovement(0.3, 0, 0);
        super.extendArm();
        sleep(1250);
        this.drivetrainHandler.stopMotors();
    }
}
