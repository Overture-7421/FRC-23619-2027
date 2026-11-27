package org.firstinspires.ftc.teamcode.Autos;

import static com.pedropathing.api.Paths.line;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import org.firstinspires.ftc.teamcode.MyRobot;

import org.firstinspires.ftc.teamcode.subsystems.Chassis;

import org.firstinspires.ftc.teamcode.subsystems.Intakedexer;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;



@Autonomous(name = "AutoTest")
public class AutoIndependent extends CommandOpMode {
    private Follower follower;
    private MyRobot robot;
    private Chassis chassis;
    // private final MyRobot.Alliance alliance;
    private GamepadEx driver;

    public AutoIndependent() {
        // alliance = allianceColor;

        driver = new GamepadEx(gamepad1);
        robot = new MyRobot(MyRobot.Alliance.Red, hardwareMap, driver);
        follower = robot.getFollower();
    }

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(27.1759, 133.8414, 270);
    private final Pose pose1 = poseFactory.of(27.176, 115.5945, 270);
    private final Pose point2 = poseFactory.of(13.9739, 94.4308, -178.4536);
    private final Pose point2Control1 = poseFactory.of(40.5497, 95.1483, 0);

    public Path pathOne = Paths.line(start, pose1).linear(start, pose1);
    public Path pathTwo = Paths.curve(pose1,point2Control1, point2).tangent();


    @Override
    public void initialize() {
        super.reset();


        chassis = robot.getChassis();
        SequentialCommandGroup autoCommand = new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, pathOne, false, 0.7)
                ),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, pathTwo, false, 0.7)
                )
        );
        
        schedule(autoCommand);
    }

    @Override
    public void run() {
        robot.run();
    }

    @Override
    public void end(){
        chassis.setLastPose();
    }

}
