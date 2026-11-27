package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.MyRobot.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

@Configurable
public class Drive extends CommandBase {
	public static double speedMultiplier = 1.0;
	public static double shootingMultiplier = 0.5;
	public static double turnMultiplier = 0.7;
	public static double shootingTurnMultiplier = 0.25;

	private final Follower follower;
	private final Alliance alliance;
	private final GamepadEx driver;

	private double allianceMultiplier;

	public Drive(Chassis chassis, Alliance alliance, GamepadEx driver) {
		this.follower = chassis.getFollower();
		this.alliance = alliance;
		this.driver = driver;
		addRequirements(chassis);
	}

	@Override
	public void initialize() {
		if (alliance == Alliance.Blue) {
			allianceMultiplier = -1;
		} else {
			allianceMultiplier = 1;
		}
	}

	@Override
	public void execute() {
		boolean isShooting = driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3;
		double shooting = isShooting ? shootingMultiplier : 1.0;
		double turn = isShooting ? shootingTurnMultiplier : turnMultiplier;

		follower.manual(ManualDrive.fieldCentric(
				driver.getLeftY() * allianceMultiplier * speedMultiplier * shooting,
				-driver.getLeftX() * allianceMultiplier * speedMultiplier * shooting,
				-driver.getRightX() * turn,
				follower.pose().heading()));
	}

}
