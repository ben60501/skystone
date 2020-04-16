package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class blueFoundation extends LinearOpMode {

    private Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.outtakeAction(Robot.Outtake.Release);

        robot.releaseFoundation();

        waitForStart();

        robot.backwardsPID(28);

        robot.turnToZero();

        robot.right(12);

        robot.turnToZero();

        robot.backwards(2, 0.25);

        robot.grabFoundation();

        robot.forwardPID(16);

        robot.turnImuBasic(15, 0.5);

        robot.forwardPID(24);

        robot.turnImuBasic(75, 0.5);

        robot.releaseFoundation();

        robot.backwards(8, 0.5);

        robot.forwardPID(24);

        robot.left(16);

        robot.forwardPID(20);

        robot.deployIntake();

    }
}
