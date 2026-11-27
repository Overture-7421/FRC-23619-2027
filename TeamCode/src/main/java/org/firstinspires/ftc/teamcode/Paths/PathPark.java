package org.firstinspires.ftc.teamcode.Paths;

import static com.pedropathing.api.Paths.line;

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


public class PathPark extends CommandOpMode {
    private Follower follower;
    private MyRobot robot;
    private Chassis chassis;

    private final MyRobot.Alliance alliance;
    private GamepadEx driver;




public PathPark(MyRobot.Alliance allianceColor){
    alliance = allianceColor;
}

    private PoseFactory poseFactory;
    private Pose start;
    private Pose pose1;
    public Path pathOne;

    @Override
public void initialize() {
    super.reset();

    driver = new GamepadEx(gamepad1);
    robot = new MyRobot(alliance, hardwareMap, driver);
    follower = robot.getFollower();
    chassis = robot.getChassis();

    if(alliance == MyRobot.Alliance.Red) {
        poseFactory = PoseFactory.degrees();

    } else if (alliance == MyRobot.Alliance.Blue){
        poseFactory = PoseFactory.degrees().mirrorY(70.75).mirrorX(70.75);

    }
    start = poseFactory.of(9.9251, 133.1863, 270);
    pose1 = poseFactory.of(9.7068, 119.7434, 270);
    pathOne = Paths.line(start, pose1).linear(start, pose1);
    chassis.initPose(true, start);

    SequentialCommandGroup autoCommand = new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                    new FollowPathCommand(follower, pathOne, false, 0.7)
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
