package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MyRobot;
import org.firstinspires.ftc.teamcode.Paths.PathPark;

@Autonomous(name = "AutoPark - Blue", group = "AutoParks")
public class AutoParkBlue extends PathPark {
    public AutoParkBlue(){
        super(MyRobot.Alliance.Blue);
    }
}
