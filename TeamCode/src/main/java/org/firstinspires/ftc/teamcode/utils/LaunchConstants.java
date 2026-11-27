package org.firstinspires.ftc.teamcode.utils;

import com.seattlesolvers.solverslib.util.InterpLUT;

public class LaunchConstants {
	public static final InterpLUT DistanceToShooter = new InterpLUT();
	public static final InterpLUT DistanceToHood = new InterpLUT();
	public static final InterpLUT DistanceToTimeOfFlight = new InterpLUT();

	static {
		DistanceToShooter.add(20.7, 1175.0);
		DistanceToShooter.add(25.4, 1175.0);
		DistanceToShooter.add(30.8, 1225.0);
		DistanceToShooter.add(35.4, 1275.0);
		DistanceToShooter.add(40.2, 1275.0);
		DistanceToShooter.add(45.5, 1275.0);
		DistanceToShooter.add(50.6, 1300.0);
		DistanceToShooter.add(55.5, 1325.0);
		DistanceToShooter.add(60.0, 1350.0);
		DistanceToShooter.add(65.8, 1375.0);
		DistanceToShooter.createLUT();

		DistanceToHood.add(20.7, 0.65);
		DistanceToHood.add(25.4, 0.40);
		DistanceToHood.add(30.8, 0.15);
		DistanceToHood.add(35.4, -0.10);
		DistanceToHood.add(40.2, -0.30);
		DistanceToHood.add(45.5, -0.40);
		DistanceToHood.add(50.6, -0.45);
		DistanceToHood.add(55.5, -0.45);
		DistanceToHood.add(60.0, -0.60);
		DistanceToHood.add(65.8, -0.65);
		DistanceToHood.createLUT();

		DistanceToTimeOfFlight.add(25.4, 0.50);
		DistanceToTimeOfFlight.add(30.8, 0.31);
		DistanceToTimeOfFlight.add(35.4, 0.34);
		DistanceToTimeOfFlight.add(40.2, 0.29);
		DistanceToTimeOfFlight.add(45.5, 0.31);
		DistanceToTimeOfFlight.add(50.6, 0.43);
		DistanceToTimeOfFlight.add(55.5, 0.52);
		DistanceToTimeOfFlight.add(60.0, 0.53);
		DistanceToTimeOfFlight.add(65.8, 0.55);
		DistanceToTimeOfFlight.createLUT();
	}
}
