package code.execution.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import code.control.identifiers.OpModeEndBehaviour;
import code.execution.autonomous.base.TigerBetterCavemanAutoBase;
import code.execution.autonomous.base.TigerPIDAutoBase;

@Autonomous(name = "TISM AUTO", group = "AUTO")
public class UberTismAuto extends TigerBetterCavemanAutoBase {

    @Override
    protected void standardBehaviour() throws Exception {
        sleep(4000);
        this.moveAndPlaceSpecimen();
        this.drivetrainHandler.Left(1000, 0.5);
        this.endAuto(OpModeEndBehaviour.BREAK);
    }
}
