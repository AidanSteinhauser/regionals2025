package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.0019915;
        ThreeWheelConstants.strafeTicksToInches = -0.0019809;
        ThreeWheelConstants.turnTicksToInches = 0.00196927;
        ThreeWheelConstants.leftY = 4.73;
        ThreeWheelConstants.rightY = -2.83;
        ThreeWheelConstants.strafeX = -0.58;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "leftFront";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "leftRear";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "rightFront";
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
    }
}




