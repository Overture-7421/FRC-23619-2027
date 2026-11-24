package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing 3.0 constants. The values below are placeholders: run the
 * AutoTune procedures in
 * {@link Tuning} (see https://pedropathing.com/docs/pathing/tuning) and paste
 * the generated configs here.
 * If you use a different drivetrain or localizer, swap the config classes and
 * {@link #createFollower} to match.
 */
public class Constants {
	public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
		c.frontLeftName.set("lf");
		c.backLeftName.set("lr");
		c.frontRightName.set("rf");
		c.backRightName.set("rr");

		c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
		c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
		c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
		c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

		c.powerThreshold.set(0.01);
		c.manualBrakeMode.set(false);
	});

	public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
		c.name.set("pinpoint");
		c.xPodOffset.set(-70.405553);
		c.yPodOffset.set(59.093552);
		c.offsetUnits.set(DistanceUnit.MM);
		c.globalDistanceUnit.set(DistanceUnit.INCH);
		c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
		c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
		c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
	});

	public static ForesightConfig foresightConfig = new ForesightConfig(
			c -> {
				Controller primaryTranslationalForward = Controller.proportional(0.38961718104217735);
				Controller secondaryTranslationalForward = Controller.proportional(0.1439531376322693);
				Controller primaryTranslationalLateral = Controller.proportional(0.6233402242929085);
				Controller secondaryTranslationalLateral = Controller.proportional(0.23030755691867943);

				c.forwardTranslational
						.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
				c.strafeTranslational
						.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

				c.coast.set(Controller.proportionalFeedforward(0.012383855669454168));
				c.brake.set(Controller.proportionalFeedforward(0.010526277319036043));

				c.headingFeedback.set(Controller.proportional(6.643479689657513));
				c.headingBrakeCoefficients.set(Vector2D.cartesian(0.07214014974305596, 0.006529206454563242));

				c.linearBrakeCoefficients.set(Matrix.diag(0.11921098019215788, 0.07232388391930887));
				c.quadraticBrakeCoefficients.set(Matrix.diag(0.0020215599298710396, 0.0020595935742556888));

				c.maxAchievableForwardVelocity.set(75.985802937674);
				c.maxAchievableStrafeVelocity.set(60.56961875581709);
				c.naturalForwardDeceleration.set(29.970309689265445);
				c.naturalStrafeDeceleration.set(62.232943481229135);
			});

	public static Follower createFollower(HardwareMap hardwareMap) {
		return new Follower(
				new PinpointLocalizer(hardwareMap, localizerConfig),
				new Mecanum(hardwareMap, drivetrainConfig),
				new Foresight(foresightConfig));
	}
}
