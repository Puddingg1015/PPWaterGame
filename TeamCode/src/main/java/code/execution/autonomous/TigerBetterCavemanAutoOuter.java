package code.execution.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import code.control.identifiers.OpModeEndBehaviour;
import code.execution.autonomous.base.TigerBetterCavemanAutoBase;

@Autonomous(name = "Tiger BetterCaveman Auto -- CORNER VER", group = "AUTO")
public class TigerBetterCavemanAutoOuter extends TigerBetterCavemanAutoBase {

    @Override
    protected void standardBehaviour() throws Exception {
        telemetry.addData("Event", 3);
        this.moveAndPlaceSpecimen();
        telemetry.addData("Event", 4);
        this.drivetrainHandler.FCOptimizedMoveDisplacementVector(1000, 300, 0.8); //0.8
        telemetry.addData("Event", 5);
        // 1 Sample
        arm.changePosition(-250, 0.5);
        int num_specimens = 3;
        for (int i = 0; i < num_specimens; i++) {
            this.drivetrainHandler.Forward(1400, 0.8); // 0.8
//            this.rest(200);
            if (i != 2) {
                this.drivetrainHandler.Right(430, 0.3); // 0.3
            } else {
                this.drivetrainHandler.Right(300, 0.3);
            }
            //            this.rest(200);
            this.drivetrainHandler.Backward(1000, 0.8); // 0.8
//            if (i != num_specimens-1) {
            this.drivetrainHandler.Backward(600, 0.4); // 0.4
//            }
            this.rest(200);
        }
        // another specimen
//        this.moveAndPlaceSecondSpecimen();
//        this.drivetrainHandler.Forward(450, 0.5);

        park();
        this.endAuto(OpModeEndBehaviour.BREAK);
    }


    private void moveAndPlaceSecondSpecimen() throws Exception {
        this.claw.setWristPos(1);
        this.claw.open();
        this.arm.changePosition(900, 0.8); // 0.5
        this.arm.setExtension(200, 1); // 0.8
        this.rest(1000);
        this.claw.close();
        this.rest(1000);
        this.arm.changePosition(-500, 0.7); // 0.5
        this.claw.setWristPos(0.5);
        this.drivetrainHandler.FCOptimizedMoveDisplacementVector(-1700, 300, 1); // 0.8
        this.drivetrainHandler.Backward(250, 0.6); // 0.6
        this.drivetrainHandler.Forward(200, 0.3); // 0.3
        moveAndPlaceSpecimen();
    }


    private void park() {
        this.arm.changePosition(-200, 0.5);
//        sleep(1000);
        this.drivetrainHandler.stopMotors();
        this.endAuto();
    }

}
