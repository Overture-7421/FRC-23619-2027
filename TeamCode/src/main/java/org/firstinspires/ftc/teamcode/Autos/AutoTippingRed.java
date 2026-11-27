package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MyRobot;
import org.firstinspires.ftc.teamcode.Paths.PathTipping;

@Autonomous(name = "AutoTipping - Red", group = "AutoTippings")
public class AutoTippingRed extends PathTipping {
    public AutoTippingRed(){
        super(MyRobot.Alliance.Red);
    }
}
