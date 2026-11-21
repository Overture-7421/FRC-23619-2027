package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.MyRobot.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

public class Drive extends CommandBase {

	private final Follower follower;
	private final Alliance alliance;
	private final GamepadEx driver;

	private double allianceMultiplier;
	private final double speedMultiplier = 1;
	private final double turnMultiplier = 0.7;

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

		follower.manual(ManualDrive.fieldCentric(
				driver.getLeftY() * allianceMultiplier * speedMultiplier,
				-driver.getLeftX() * allianceMultiplier * speedMultiplier,
				-driver.getRightX() * turnMultiplier,
				follower.pose().heading()));
	}

}
