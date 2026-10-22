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
                    c.frontRightName.set("fr");
                    c.backLeftName.set("bl");
                    c.backRightName.set("br");
                    c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                    c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                    c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                    c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

               c.manualBrakeMode.set(true);
            }
    );

    public static OctoQuadConfig localizerConfig = new OctoQuadConfig(c -> {
        c.name.set("octoquad");
        c.xPodPort.set(6);
        c.yPodPort.set(7);
        c.ticksPerUnit.set(505.316944406);
        c.xPodOffset.set(-0.07874015748031496);
        c.yPodOffset.set(-0.767716535433071);
        c.xPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
        c.yPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
        c.headingScalar.set(1.230749900167557);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.23741338580219604);
                Controller secondaryTranslationalForward = Controller.proportional(0.08771790225140731);
                Controller primaryTranslationalLateral = Controller.proportional(2.332363579676565);
                Controller secondaryTranslationalLateral = Controller.proportional(0.8617460207878429);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.016383479874995107));
                c.brake.set(Controller.proportionalFeedforward(0.01392595789374584));

                c.headingFeedback.set(Controller.proportional(4.253966061143969));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.029255431017874277, 0.009681128188096738));

                c.linearBrakeCoefficients.set(Matrix.diag(0.04096497332551374, 0.027061111377451142));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0025294775756007646, 0.0026028654678332402));

                c.maxAchievableForwardVelocity.set(63.15469506757788);
                c.maxAchievableStrafeVelocity.set(41.78575036387146);
                c.naturalForwardDeceleration.set(42.10073264920968);
                c.naturalStrafeDeceleration.set(81.92559474164996);
            }
    );


}