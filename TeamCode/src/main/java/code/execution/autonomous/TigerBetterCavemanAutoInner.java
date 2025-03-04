package code.execution.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import code.control.identifiers.OpModeEndBehaviour;
import code.execution.autonomous.base.TigerBetterCavemanAutoBase;

@Autonomous(name = "Tiger BetterCaveman Auto -- INNER VER")
public class TigerBetterCavemanAutoInner extends TigerBetterCavemanAutoBase {

    @Override
    protected void standardBehaviour() throws Exception {
        telemetry.addData("Event", 3);
        this.moveAndPlaceSpecimen();
        telemetry.addData("Event", 4);
        this.drivetrainHandler.FCOptimizedMoveDisplacementVector(-1000, 300, 0.8); //0.8
        telemetry.addData("Event", 5);
        // 1 Sample
        arm.changePosition(-250, 0.5);
        int num_specimens = 3;
        for (int i = 0; i < num_specimens; i++) {
            this.drivetrainHandler.Forward(1400, 0.8); // 0.8
//            this.rest(200);
            if (i != 2) {
                this.drivetrainHandler.Left(445, 0.5); // 0.3
            } else {
                this.drivetrainHandler.Left(300, 0.3);
            }
//            this.rest(200);
            this.drivetrainHandler.Backward(1100, 0.8); // 0.8
//            if (i != num_specimens-1) {
            this.drivetrainHandler.Backward(300, 0.4); // 0.4
//            }
            this.rest(200);
        }
        // another specimen
//        this.moveAndPlaceSecondSpecimen();
//        this.drivetrainHandler.Forward(450, 0.5);

        park();
        this.endAuto(OpModeEndBehaviour.BREAK);
    }


    private void park() {
        this.arm.changePosition(-200, 0.5);
//        sleep(1000);
        this.drivetrainHandler.stopMotors();
        this.endAuto();
    }

}
