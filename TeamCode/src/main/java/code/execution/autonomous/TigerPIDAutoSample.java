package code.execution.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import code.control.identifiers.OpModeEndBehaviour;
import code.execution.autonomous.base.TigerPIDAutoBase;

@Autonomous(name = "Tiger PID Auto -- SAMPLE VER", group = "AUTO")
public class TigerPIDAutoSample extends TigerPIDAutoBase {

    @Override
    protected void standardBehaviour() throws Exception {
        telemetry.addData("Event", 3);
        this.moveAndPlaceSpecimen();
        this.drivetrainHandler.movePIDDisplacementVector(1000, 0, 0.8); //0.8
        arm.changePosition(-250, 0.5);
        int num_specimens = 3;
        for (int i = 0; i < num_specimens; i++) {
            this.drivetrainHandler.movePIDDisplacementVector(0, 1400, 0.8); // 0.8
//            this.rest(200);
            if (i != 2) {
                this.drivetrainHandler.movePIDDisplacementVector(430, 0, 0.3); // 0.3
            } else {
                this.drivetrainHandler.movePIDDisplacementVector(300, 0, 0.3);
            }
            //            this.rest(200);
            this.drivetrainHandler.movePIDDisplacementVector(0, -1000, 0.8); // 0.8
//            if (i != num_specimens-1) {
            this.drivetrainHandler.movePIDDisplacementVector(0, -600, 0.4); // 0.4
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
//        this.claw.setWristPos(1);
        claw.setUp();
        this.claw.open();
        this.arm.changePosition(900, 0.8); // 0.5
//        this.arm.setExtension(200, 1); // 0.8
        this.rest(1000);
        this.claw.close();
        this.rest(1000);
        this.arm.changePosition(-500, 0.7); // 0.5
//        this.claw.setWristPos(0.5);
        claw.setHorizontal();
        this.drivetrainHandler.movePIDDisplacementVector(-1700, -300, 1); // 0.8
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
