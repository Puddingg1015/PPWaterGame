package code.execution.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import code.control.identifiers.OpModeEndBehaviour;
import code.execution.autonomous.base.TigerCavemanAutoBase;

@Autonomous(name = "Tiger Caveman Auto -- CORNER VER")
public class TigerCavemanAutoOuter extends TigerCavemanAutoBase {

    @Override
    protected void standardBehaviour() throws Exception {
        this.moveAndPlaceSpecimen();
        this.fieldCentricPowerVectorMovement(1, 0.1, 0);
        sleep(750);
        this.rest(500);
        this.fieldCentricPowerVectorMovement(0, -1, 0);
        sleep(1000);
        this.rest(500);
        this.fieldCentricPowerVectorMovement(0.69, 0, 0);
        sleep(600);
        this.rest(500);
        arm.changePosition(-250, 0.5);
        this.fieldCentricPowerVectorMovement(0, 1, 0);
        sleep(800);
        this.fieldCentricPowerVectorMovement(0, 0.1, 0);
        sleep(400);
        this.rest(500);
//        this.fieldCentricPowerVectorMovement(0, 0.69, 0);
//        sleep(750);
//        this.rest(500);
        //        this.park();
        this.endAuto(OpModeEndBehaviour.BREAK);
    }

    private void rest(int ms) {
        this.drivetrainHandler.stopMotors();
        sleep(ms);
    }

    private void moveAndPlaceSpecimen() throws Exception {
        this.fieldCentricPowerVectorMovement(-0, -0.5, 0);
        sleep(500);
        this.drivetrainHandler.stopMotors();
//        arm.changePosition(567, 0.4);
        sleep(0);
        super.arm.setExtension(1000, 0.9);
        sleep(0);
        claw.setWristPos(1);
        arm.changePosition(-45, 0.5);
        sleep(500);
        this.fieldCentricPowerVectorMovement(-0, 0.6
                , 0);
        sleep(500);
        this.drivetrainHandler.stopMotors();
        claw.open();
        sleep(300);
        claw.setWristPos(0.8);
        this.drivetrainHandler.stopMotors();
        super.arm.setExtension(-1000, 0.9);
//        arm.changePosition(-200-369-31, 1);
        sleep(0);
        this.fieldCentricPowerVectorMovement(0, 0.3, 0);
        super.retractArm();
        sleep(0);
        this.drivetrainHandler.stopMotors();
        this.claw.open();
        sleep(0);
        this.claw.close();
        arm.changePosition(150, 0.5);
    }


    private void park() {
        this.arm.changePosition(-200, 0.5);
        this.fieldCentricPowerVectorMovement(1, 0.1, 0);
        sleep(1000);
        this.drivetrainHandler.stopMotors();
        this.endAuto();
    }

}
