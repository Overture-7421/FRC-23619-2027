package org.firstinspires.ftc.teamcode.utils;

import com.seattlesolvers.solverslib.util.InterpLUT;

public class LaunchConstants {
	public static final InterpLUT DistanceToShooter = new InterpLUT();
	public static final InterpLUT DistanceToHood = new InterpLUT();
	public static final InterpLUT DistanceToTimeOfFlight = new InterpLUT();

	static {
		DistanceToShooter.add(36.0, 1690.0);
		DistanceToShooter.add(60.0, 1830.0);
		DistanceToShooter.add(84.0, 1990.0);
		DistanceToShooter.add(108.0, 2150.0);
		DistanceToShooter.add(132.0, 2310.0);
		DistanceToShooter.add(156.0, 2460.0);
		DistanceToShooter.createLUT();

		DistanceToHood.add(36.0, -0.120);
		DistanceToHood.add(60.0, -0.386);
		DistanceToHood.add(84.0, -0.572);
		DistanceToHood.add(108.0, -0.710);
		DistanceToHood.add(132.0, -0.814);
		DistanceToHood.add(156.0, -0.894);
		DistanceToHood.createLUT();

		DistanceToTimeOfFlight.add(36.0, 0.57);
		DistanceToTimeOfFlight.add(60.0, 0.63);
		DistanceToTimeOfFlight.add(84.0, 0.68);
		DistanceToTimeOfFlight.add(108.0, 0.74);
		DistanceToTimeOfFlight.add(132.0, 0.78);
		DistanceToTimeOfFlight.add(156.0, 0.83);
		DistanceToTimeOfFlight.createLUT();
	}
}
