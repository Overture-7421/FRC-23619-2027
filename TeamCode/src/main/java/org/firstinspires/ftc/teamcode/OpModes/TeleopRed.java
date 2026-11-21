package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleopMode;
import org.firstinspires.ftc.teamcode.MyRobot;

@TeleOp(name = "Teleop - Red", group = "Teleop")
public class TeleopRed extends TeleopMode {
	public TeleopRed() {
		super(MyRobot.Alliance.Red);
	}
}
