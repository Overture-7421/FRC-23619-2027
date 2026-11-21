package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Robot;

import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.commands.Drive;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.Intakedexer;

public class MyRobot extends Robot {

	public enum Alliance {
		Red,
		Blue
	}

	private final Intakedexer intake;
	private final Chassis chassis;
	private final Alliance alliance;
	private final GamepadEx driver;
	private final Follower follower;

	public MyRobot(Alliance alliance, HardwareMap hardwareMap, GamepadEx driver) {
		this.driver = driver;
		this.alliance = alliance;
		follower = Constants.createFollower(hardwareMap);

		chassis = new Chassis(follower);
		intake = new Intakedexer(hardwareMap);

		setBulkReading(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
	}

	public Intakedexer getIntake() {
		return this.intake;
	}

	public void initTeleop() {
		chassis.initPose(false, new Pose(8, 8));
		chassis.setDefaultCommand(new Drive(chassis, alliance, driver));
	}

	public void onEnd() {
		chassis.setLastPose();
	}

	public void updateLoop() {
		CommandScheduler.getInstance().run();
	}
}