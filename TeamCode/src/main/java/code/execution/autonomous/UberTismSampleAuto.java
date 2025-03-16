package code.execution.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import code.control.identifiers.OpModeEndBehaviour;
import code.execution.autonomous.base.TigerBetterCavemanAutoBase;

@Autonomous(name = "TISM SAMPLE AUTO", group = "AUTO")
public class UberTismSampleAuto extends TigerBetterCavemanAutoBase {

    @Override
    protected void standardBehaviour() throws Exception {
        this.claw.setStraight();
        this.drivetrainHandler.Backward(1000, 0.5);
        this.arm.setExtension(1050, 0.6);
        this.arm.changePosition(350, 0.5);
        this.claw.setUp();
        this.sleep(300);
        this.claw.open();
        this.endAuto(OpModeEndBehaviour.BREAK);
    }
}
