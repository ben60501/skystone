package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class forwardBridge extends LinearOpMode{

    private Robot robot = new Robot();
    //private int skystonePosition = 2;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.outtakeAction(Robot.Outtake.Release);

        robot.releaseFoundation();

        waitForStart();

        robot.deployIntake();

        //sleep(25000);

        robot.forward(8, 1);

        //sleep(500);
    }
}
