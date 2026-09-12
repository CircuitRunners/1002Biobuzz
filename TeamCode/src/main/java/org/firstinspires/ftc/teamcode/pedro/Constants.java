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
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(new OctoQuadLocalizer(h, localizerConfig), new Mecanum(h, drivetrainConfig), new Foresight(foresightConfig));
    }
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("fl");
                c.backLeftName.set("bl");
                c.frontRightName.set("fr");
                c.backRightName.set("br");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

                c.manualBrakeMode.set(true);
            }
    );

    public static OctoQuadConfig localizerConfig = new OctoQuadConfig(c -> {
        c.name.set("octoquad");
        c.xPodPort.set(0);
        c.yPodPort.set(1);
        c.ticksPerUnit.set(505.316944406);
        c.xPodOffset.set(-3.799212598425197);
        c.yPodOffset.set(6.476377952755906);
        c.xPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
        c.yPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
        c.headingScalar.set(1.015709653014181);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.13817045357235355);
                Controller secondaryTranslationalForward = Controller.proportional(0.051050290612468976);
                Controller primaryTranslationalLateral = Controller.proportional(0.2578937460788165);
                Controller secondaryTranslationalLateral = Controller.proportional(0.09528484812831323);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013680879526316662));
                c.brake.set(Controller.proportionalFeedforward(0.011628747597369163));

                c.headingFeedback.set(Controller.proportional(2.070976931601414));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.03364917668154549, 0.006485046851924603));

                c.linearBrakeCoefficients.set(Matrix.diag(0.049944648956514175, 0.026808696654856887));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0015932765096998197, 0.002130770804810657));

                c.maxAchievableForwardVelocity.set(77.04514170515807);
                c.maxAchievableStrafeVelocity.set(55.56413798239545);
                c.naturalForwardDeceleration.set(39.323895826863534);
                c.naturalStrafeDeceleration.set(58.56035025720471);
            }
    );


}