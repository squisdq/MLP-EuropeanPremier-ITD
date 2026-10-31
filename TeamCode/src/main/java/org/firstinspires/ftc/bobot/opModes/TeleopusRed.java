package org.firstinspires.ftc.bobot.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.bobot.Teleopus;

@TeleOp(group = "0opMode")
public class TeleopusRed extends Teleopus {

    @Override
    public void runOpMode() throws InterruptedException {
        super.alliance = Alliance.RED;
        super.runOpMode();
    }
}
