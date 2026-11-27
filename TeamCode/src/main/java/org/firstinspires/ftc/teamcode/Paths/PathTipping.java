package org.firstinspires.ftc.teamcode.Paths;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.MyRobot;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intakedexer;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class PathTipping extends CommandOpMode {
    private Follower follower;
    private MyRobot robot;
    private Chassis chassis;
    private Shooter shooter;
    private Intakedexer intakedexer;
    private Turret turret;
    private Camera camera;
    private Arm arm;
    private Hood hood;


    private final MyRobot.Alliance alliance;
    private GamepadEx driver;
    public PathTipping(MyRobot.Alliance allianceColor){
        alliance = allianceColor;
    }

    private PoseFactory poseFactory;
    private Pose start;
    private Pose pose1;
    private Pose pose2;
    private Pose pose3;
    private Pose point3Control1;
    private Pose pose4;
    private Pose point4Control1;
    private Pose point4Control2;
    private Pose pose5;
    private Pose pose6;

    public Path pathOne;
    public Path pathTwo;
    public Path pathThree;
    public Path pathFour;
    public Path pathFive;
    public Path pathSix;
    @Override
    public void initialize(){
        super.reset();

        driver = new GamepadEx(gamepad1);
        robot = new MyRobot(alliance, hardwareMap, driver);
        follower = robot.getFollower();
        chassis = robot.getChassis();
        hood = robot.getHood();
        turret = robot.getTurret();
        arm = robot.getArm();
        intakedexer = robot.getIntake();
        shooter = robot.getShooter();
        camera = robot.getCamera();

        if(alliance == MyRobot.Alliance.Red) {
            poseFactory = PoseFactory.degrees();

        } else if (alliance == MyRobot.Alliance.Blue){
            poseFactory = PoseFactory.degrees().mirrorY(70.75).mirrorX(70.75);

        }

        start = poseFactory.of(47.7022, 9.0918, 90);
        pose1 = poseFactory.of(10.3618, 8.7045, 180);
        pose2 = poseFactory.of(37.033, 8.6228, 180);
        pose3 = poseFactory.of(12.3611, 46.7156, 180);
        point3Control1 = poseFactory.of(42.4579, 46.3732, 0);
        pose4 = poseFactory.of(46.2757, 123.5189, 90);
        point4Control1 = poseFactory.of(36.1893, 55.2577, 0);
        point4Control2 = poseFactory.of(4.0839, 112.8574, 0);
        pose5 = poseFactory.of(46.702, 129.8528, 90);
        pose6 = poseFactory.of(12.2007, 120.6338, 90);

        pathOne = Paths.line(start, pose1).linear(start, pose1);
        pathTwo = Paths.line(pose1, pose2).linear(pose1, pose2);
        pathThree = Paths.curve(pose2, point3Control1, pose3).linear(pose2, pose3);
        pathFour = Paths.curve(pose3, point4Control1, point4Control2, pose4).linear(pose3, pose4);
        pathFive = Paths.line(pose4, pose5).linear(pose4, pose5);
        pathSix = Paths.line(pose5, pose6).linear(pose5, pose6);

        chassis.initPose(true, start);

        SequentialCommandGroup autoCommand = new SequentialCommandGroup(

                intakedexer.intakeAutoShoot(),
                //shoot
                new WaitCommand(600),
                //stopshoot
                intakedexer.stopIntake(),
                new ParallelDeadlineGroup(
                        intakedexer.intake(),
                        new FollowPathCommand(follower, pathOne, false, 0.7)
                ),
                intakedexer.stopIntake(),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, pathTwo, false, 0.7)
                ),
                intakedexer.intakeAutoShoot(),
                //shoot
                new WaitCommand(600),
                //stopshoot
                arm.open(),
                new ParallelDeadlineGroup(
                        intakedexer.intake(),
                        new FollowPathCommand(follower, pathThree, false, 0.7)
                ),
                arm.close(),
                intakedexer.stopIntake(),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, pathFour, false, 0.7)
                ),
                intakedexer.intakeAutoShoot(),
                //shoot
                new WaitCommand(600),
                //stopshoot
                intakedexer.stopIntake(),
                new ParallelDeadlineGroup(
                        intakedexer.intake(),
                        arm.open(),
                        new FollowPathCommand(follower, pathFive, false, 0.7)
                ),
                arm.close(),
                intakedexer.stopIntake(),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, pathSix, false, 0.7)
                )

        );

        schedule(autoCommand);
    }

    @Override
    public void run(){
        robot.run();
    }

    @Override
    public void end(){
        chassis.setLastPose();
    }
}
