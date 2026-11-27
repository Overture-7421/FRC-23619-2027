package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MyRobot;
import org.firstinspires.ftc.teamcode.Paths.PathPark;

@Autonomous(name = "AutoPark - Red", group = "AutoParks")
public class AutoParkRed extends PathPark {
    public AutoParkRed(){
        super(MyRobot.Alliance.Red);
    }
}
