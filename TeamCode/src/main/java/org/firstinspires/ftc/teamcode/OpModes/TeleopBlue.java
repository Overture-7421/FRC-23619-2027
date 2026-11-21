package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleopMode;
import org.firstinspires.ftc.teamcode.MyRobot;

@TeleOp(name = "Teleop - Blue", group = "Teleop")
public class TeleopBlue extends TeleopMode {
	public TeleopBlue() {
		super(MyRobot.Alliance.Blue);
	}
}
