package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static OctoQuadConfig localizerConfig = new OctoQuadConfig(c -> {
        c.name.set("octoquad");
        c.xPodPort.set(1);
        c.yPodPort.set(0);
        c.ticksPerUnit.set(505.316944406);
        c.xPodOffset.set(-6.535433070866143);
        c.yPodOffset.set(-2.5);
        c.xPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
        c.yPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
        c.headingScalar.set(1.0136439146076104);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2608564262964484);
                Controller secondaryTranslationalForward = Controller.proportional(0.09637947930445483);
                Controller primaryTranslationalLateral = Controller.proportional(0.3186949891975569);
                Controller secondaryTranslationalLateral = Controller.proportional(0.11774928282155027);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.011624412629352184));
                c.brake.set(Controller.proportionalFeedforward(0.009880750734949355));

                c.headingFeedback.set(Controller.proportional(4.809761573923885));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0597580805597309, 0.004109986295634797));

                c.linearBrakeCoefficients.set(Matrix.diag(0.08836193718531676, 0.06190219023333389));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0013744840122157293, 0.0016171183880403525));

                c.maxAchievableForwardVelocity.set(85.47816438012906);
                c.maxAchievableStrafeVelocity.set(73.59539503003818);
                c.naturalForwardDeceleration.set(38.56659432038697);
                c.naturalStrafeDeceleration.set(64.64074506087198);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new OctoQuadLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}