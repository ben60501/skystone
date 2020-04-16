package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class deployIntake extends LinearOpMode {

    private Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.outtakeAction(Robot.Outtake.Release);

        waitForStart();

        sleep(200);

        robot.deployIntake();

        sleep(500);

        robot.stopIntaking();

    }
}
