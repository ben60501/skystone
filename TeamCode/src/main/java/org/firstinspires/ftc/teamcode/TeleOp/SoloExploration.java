package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp
public class SoloExploration extends OpMode {

    private DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight, motorOuttake, motorIntakeRight, motorIntakeLeft;
    private Servo servoOuttakeRotate, servoOuttakeClaw, servoFoundationLeft, servoFoundationRight;

    private double leftX, rightX, leftY, rightY;
    private double leftBumperTime = 0, aTime = 0, bTime = 0;

    private boolean leftBumper = false;

    private Intake intakeState;

    private enum Intake {
        INTAKE, OUTTAKE, RELEASE, OFF
    }

    @Override
    public void init() {
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorOuttake = hardwareMap.get(DcMotor.class, "motorOuttake");
        motorIntakeLeft = hardwareMap.get(DcMotor.class, "motorIntakeLeft");
        motorIntakeRight = hardwareMap.get(DcMotor.class, "motorIntakeRight");

        servoOuttakeClaw = hardwareMap.get(Servo.class, "servoOuttakeClaw");
        servoOuttakeRotate = hardwareMap.get(Servo.class, "servoOuttakeRotate");
        servoFoundationLeft = hardwareMap.get(Servo.class, "servoFoundationLeft");
        servoFoundationRight = hardwareMap.get(Servo.class, "servoFoundationRight");

        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorOuttake.setDirection(DcMotor.Direction.REVERSE);
        motorIntakeRight.setDirection(DcMotor.Direction.REVERSE);
        motorIntakeLeft.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorOuttake.setPower(0);
        motorIntakeRight.setPower(0);
        motorIntakeLeft.setPower(0);

        motorOuttake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorOuttake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorOuttake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        resetStartTime();
    }

    @Override
    public void loop() {
        leftX = gamepad1.left_stick_x;
        rightX = gamepad1.right_stick_x;
        leftY = -gamepad1.left_stick_y;
        rightY = -gamepad1.right_stick_y;

        if (gamepad1.right_bumper) {
            leftX *= 0.5;
            rightX *= 0.5;
            leftY *= 0.5;
            rightY *= 0.5;
        }

        //Drivetrain
        if (Math.abs(rightX) >= 0.1) {
            //Turning
            motorFrontRight.setPower(-rightX);
            motorFrontLeft.setPower(rightX);
            motorBackRight.setPower(-rightX);
            motorBackLeft.setPower(rightX);
        } else if (Math.abs(leftY) >= 0.1 || Math.abs(leftX) >= 0.1) {
            if (Math.abs(leftY) >= Math.abs(leftX)) {
                //Move Y
                if (Math.abs(leftY) < 0.65) {
                    leftY /= 2;
                }
                motorFrontRight.setPower(leftY);
                motorFrontLeft.setPower(leftY);
                motorBackRight.setPower(leftY);
                motorBackLeft.setPower(leftY);
            } else {
                //Move X
                if (Math.abs(leftX) < 0.65) {
                    leftX /= 2;
                }
                if (gamepad1.right_bumper) {
                    leftX += (0.15 * (leftX / Math.abs(leftX)));
                }
                motorFrontRight.setPower(-leftX);
                motorFrontLeft.setPower(leftX);
                motorBackRight.setPower(leftX);
                motorBackLeft.setPower(-leftX);
            }
        } else {
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorBackRight.setPower(0);
            motorBackLeft.setPower(0);
        }

        //Outtake; left joystick up and down; >0.7 to swing servo x axis; grabby servo: x to grab y to release
        if (gamepad1.left_trigger > 0.05) {
            motorOuttake.setPower(gamepad1.left_trigger);
        } else if (gamepad1.right_trigger > 0.05) {
            motorOuttake.setPower(-gamepad1.right_trigger);
        } else {
            if (motorOuttake.getCurrentPosition() < -100) {
                motorOuttake.setPower(0.1);
            } else {
                motorOuttake.setPower(0);
            }
        }

        //Check if bumper was actually pressed and not held
        if(gamepad1.left_bumper) {
            if(leftBumperTime == 0 || getRuntime() - leftBumperTime > 0.5) {
                leftBumper = !leftBumper;
            }
            leftBumperTime = getRuntime();
        }

        //Action depending on state of left bumper
        if (leftBumper) {
            servoOuttakeRotate.setPosition(0.06);

        } else {
            servoOuttakeRotate.setPosition(0.78);
        }

        if (gamepad1.x) {
            servoOuttakeClaw.setPosition(0.45);
        } else if (gamepad1.y) {
            servoOuttakeClaw.setPosition(0);
        }

        if (gamepad1.a && gamepad1.b) {
            intakeState = Intake.RELEASE;
        } else if(gamepad1.a) {
            if(aTime == 0 || getRuntime() - aTime > 0.25)
                if (intakeState == Intake.INTAKE) {
                    intakeState = Intake.OFF;
                } else {
                    intakeState = Intake.INTAKE;
                }

            aTime = getRuntime();
        } else if (gamepad1.b) {
            if(bTime == 0 || getRuntime() - bTime > 0.25)
                if (intakeState == Intake.OUTTAKE) {
                    intakeState = Intake.OFF;
                } else {
                    intakeState = Intake.OUTTAKE;
                }

            bTime = getRuntime();
        }

        //Intake (a intake (and stop) b outtake) d pad down is down; up is up, left or right to make horizontal
        if (intakeState == Intake.INTAKE) {
            motorIntakeRight.setPower(0.35);
            motorIntakeLeft.setPower(-0.35);
        } else if (intakeState == Intake.OUTTAKE) {
            motorIntakeRight.setPower(-0.35);
            motorIntakeLeft.setPower(0.35);
        } else if (intakeState == Intake.RELEASE) {
            motorIntakeLeft.setPower(1);
            motorIntakeRight.setPower(1);
        } else if (intakeState == Intake.OFF) {
            motorIntakeRight.setPower(0);
            motorIntakeLeft.setPower(0);
        }

        if (gamepad1.dpad_up) {
            //Release Foundation
            servoFoundationRight.setPosition(0.75);
            servoFoundationLeft.setPosition(0.25);
        } else if (gamepad1.dpad_down) {
            //Grab Foundation
            servoFoundationRight.setPosition(0);
            servoFoundationLeft.setPosition(1);
        }
    }
}
