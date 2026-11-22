package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Robot;

import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.Drive;
import org.firstinspires.ftc.teamcode.commands.Launch;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intakedexer;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class MyRobot extends Robot {

	public enum Alliance {
		Red,
		Blue
	}

	private final Intakedexer intake;
	private final Chassis chassis;
	private final Turret turret;
	private final Shooter shooter;
	private final Hood hood;
	private final Arm arm;
	private final Camera camera;

	private final Alliance alliance;
	private final GamepadEx driver;
	private final Follower follower;

	public MyRobot(Alliance alliance, HardwareMap hardwareMap, GamepadEx driver) {
		this.driver = driver;
		this.alliance = alliance;
		follower = Constants.createFollower(hardwareMap);

		chassis = new Chassis(follower);
		turret = new Turret(hardwareMap);
		shooter = new Shooter(hardwareMap);
		hood = new Hood(hardwareMap);
		arm = new Arm(hardwareMap);

		camera = new Camera(hardwareMap, alliance, () -> turret.getTurretPose(follower.pose()));

		intake = new Intakedexer(hardwareMap);

		setBulkReading(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
	}

	public Intakedexer getIntake() {
		return this.intake;
	}

	public void initTeleop() {
		chassis.initPose(false, new Pose(10.68, 72, 0));
		chassis.setDefaultCommand(new Drive(chassis, alliance, driver));
		Launch launch = new Launch(turret, shooter, hood, camera, chassis, alliance,
				() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3);
		turret.setDefaultCommand(launch);

		new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.3)
				.whileActiveContinuous(intake.intake());

		new Trigger(launch::canFeed).whileActiveContinuous(intake.shoot());

		driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
				.whenPressed(arm.open())
				.whenReleased(arm.close());
	}

	public void onEnd() {
		chassis.setLastPose();
	}

	public void updateLoop() {
		CommandScheduler.getInstance().run();
	}
}