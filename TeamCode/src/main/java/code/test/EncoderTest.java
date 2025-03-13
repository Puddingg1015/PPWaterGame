package code.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import code.execution.autonomous.base.TigerCavemanAutoBase;
import code.control.identifiers.OpModeEndBehaviour;

@Autonomous(name = "Tiger Encoder Test", group = "TEST")
public class EncoderTest extends TigerCavemanAutoBase {

    @Override
    protected void standardBehaviour() {
//        this.drivetrainHandler.Forward(1000, 0.5);
//        this.drivetrainHandler.Left(500, 0.5);
//        this.drivetrainHandler.Right(500, 0.5);
//        this.drivetrainHandler.Backward(1000, 0.5);
//        sleep(1000);
        this.drivetrainHandler.OptimizedMovePolarVector(1000, PI/6, 0.2);
        this.park();
        this.endAuto(OpModeEndBehaviour.BREAK);
    }

    private void park() {
        this.drivetrainHandler.stopMotors();
    }
}
