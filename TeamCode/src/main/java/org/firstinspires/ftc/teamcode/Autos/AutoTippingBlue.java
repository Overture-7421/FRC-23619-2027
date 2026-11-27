package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MyRobot;
import org.firstinspires.ftc.teamcode.Paths.PathTipping;

@Autonomous(name = "AutoTipping - Blue", group = "AutoTippings")
public class AutoTippingBlue extends PathTipping {
    public AutoTippingBlue(){
        super(MyRobot.Alliance.Blue);
    }
}
