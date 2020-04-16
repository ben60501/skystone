package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SkystoneDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous

public class stoneOnlyBlue extends LinearOpMode{

    private Robot robot = new Robot();
    private int skystonePosition = 2;

    private OpenCvCamera phoneCam;
    private SkystoneDetector skyStoneDetector;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.outtakeAction(Robot.Outtake.Release);

        robot.releaseFoundation();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();

        skyStoneDetector = new SkystoneDetector();
        phoneCam.setPipeline(skyStoneDetector);

        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);

        waitForStart();

        while (!opModeIsActive()) {
            telemetry.addData("Stone Position X", skyStoneDetector.getScreenPosition().x);

            if (skyStoneDetector.getScreenPosition().x < 100) {
                skystonePosition = 0;
            } else if (skyStoneDetector.getScreenPosition().x < 200) {
                skystonePosition = 1;
            } else {
                skystonePosition = 2;
            }

            telemetry.addData("Stone Position: ", skystonePosition);
            telemetry.update();

        }

        robot.deployIntake();

        if(skystonePosition == 1){
            skyStonePosition1();
        }else if(skystonePosition == 2) {
            skyStonePosition2();
        }else{
            skyStonePosition0();
        }
        moveFoundation();

        dropStone();

        park();

    }

    void skyStonePosition0() {
        robot.forwardPID(25);
        sleep(100);
        robot.turnToZero();
        robot.turnImu(-90, 1);

        robot.resetAngle();

        sleep(250);

        robot.resetAngle();
        robot.left(19);

        sleep(250);

        robot.turnToZero();

        robot.intake();

        robot.forward(7, 0.25);
        sleep(1000);
        robot.outtakeAction(Robot.Outtake.Grab);
        robot.stopIntaking();

        robot.right(15);

        sleep(250);

        robot.turnToZero();

        //robot.resetAngle();

        robot.backwardsPID(45);

        robot.turnToZero();
        robot.resetAngle();

        robot.backwardsPID(40);

        sleep(250);

        //robot.turnToZero();

        robot.turnImu(-90, 1);

        robot.backwards(18, 0.35);
    }

    void skyStonePosition1() {
        robot.forwardPID(25);
        sleep(100);
        robot.turnToZero();
        robot.turnImu(-90, 1);

        robot.resetAngle();

        sleep(250);

        robot.resetAngle();

        robot.forwardPID(9);

        robot.turnToZero();

        robot.resetAngle();

        robot.left(19);

        sleep(250);

        robot.turnToZeroFast(0.175);

        robot.intake();

        robot.forward(7, 0.25);
        sleep(1000);
        robot.outtakeAction(Robot.Outtake.Grab);
        robot.stopIntaking();

        robot.right(16);

        sleep(250);

        //robot.turnToZero();

        robot.turnToZeroFast(0.175);

        robot.resetAngle();

        robot.backwardsPID(54);

        robot.turnToZero();
        robot.resetAngle();

        robot.backwardsPID(40);

        sleep(250);

        //robot.turnToZero();

        robot.turnImu(-90, 1);

        robot.backwards(14, 0.35);

        sleep(500);

    }

    void skyStonePosition2() {
        robot.forwardPID(25);
        sleep(100);
        robot.turnToZero();
        robot.turnImu(-90, 1);

        robot.resetAngle();

        sleep(250);

        robot.resetAngle();

        robot.forwardPID(16);

        //robot.turnToZero();

        //robot.resetAngle();

        robot.left(20);

        sleep(250);

        robot.turnToZero();

        robot.intake();

        robot.forward(7, 0.25);
        sleep(1000);
        robot.outtakeAction(Robot.Outtake.Grab);
        robot.stopIntaking();

        robot.right(14);

        sleep(250);

        robot.turnToZero();
        robot.resetAngle();

        robot.backwardsPID(62);

        sleep(250);
        robot.turnToZero();
        sleep(250);
        robot.resetAngle();

        robot.backwardsPID(40);

        sleep(250);

        robot.turnToZeroFast(0.175);

        robot.turnImu(-90, 1);

        robot.backwards(13, 0.35);

        sleep(500);
    }

    void moveFoundation() {
        robot.grabFoundation();

        robot.forward(16, 0.5);
        //robot.forwardPID(16);

        robot.turnImuBasic(15, 0.5);

        robot.forward(24, 0.5);
        //robot.forwardPID(24);

        robot.turnImuBasic(75, 0.5);

        robot.releaseFoundation();

        robot.backwards(10, 0.5);
    }

    void dropStone() {
        robot.outtakeAction(Robot.Outtake.Grab);

        sleep(500);

        robot.outtakeAction(Robot.Outtake.Up);

        robot.outtakeAction(Robot.Outtake.Out);

        sleep(750);

        robot.outtakeAction(Robot.Outtake.Down);

        robot.outtakeAction(Robot.Outtake.Release);

        sleep(500);

        robot.outtakeAction(Robot.Outtake.Up);

        robot.outtakeAction(Robot.Outtake.In);

        sleep(750);

        robot.outtakeAction(Robot.Outtake.Down);
    }

    void park() {
        robot.forwardPID(20);

        robot.left(16);

        robot.forwardPID(24);
    }
}
