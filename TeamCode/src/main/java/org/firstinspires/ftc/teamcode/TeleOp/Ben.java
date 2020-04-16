package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class Ben extends OpMode {

    private DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight, motorOuttake, motorIntakeRight, motorIntakeLeft, motorIntakeMiddle;
    private Servo servoOuttakeRotate, servoOuttakeClaw, servoFoundationLeft, servoFoundationRight, servoCapstone;
    private double powerFR, powerFL, powerBR, powerBL;
    private double leftX, rightX, leftY, rightY, leftX2, leftY2;

    double motor0Raw, motor1Raw, motor2Raw, motor3Raw, rawMax;
    double motor0Scaled, motor1Scaled, motor2Scaled, motor3Scaled;

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
        motorIntakeMiddle = hardwareMap.get(DcMotor.class, "motorIntakeMiddle");

        servoCapstone = hardwareMap.get(Servo.class, "servoCapstone");
        servoOuttakeClaw = hardwareMap.get(Servo.class, "servoOuttakeClaw");
        servoOuttakeRotate = hardwareMap.get(Servo.class, "servoOuttakeRotate");
        servoFoundationLeft = hardwareMap.get(Servo.class, "servoFoundationLeft");
        servoFoundationRight = hardwareMap.get(Servo.class, "servoFoundationRight");

        motorFrontRight.setDirection(DcMotor.Direction.REVERSE); //Forward
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE); // Forward
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        motorOuttake.setDirection(DcMotor.Direction.REVERSE);
        motorIntakeRight.setDirection(DcMotor.Direction.REVERSE);
        motorIntakeLeft.setDirection(DcMotor.Direction.REVERSE);
        motorIntakeMiddle.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorOuttake.setPower(0);
        motorIntakeRight.setPower(0);
        motorIntakeLeft.setPower(0);
        motorIntakeMiddle.setPower(0);

        servoCapstone.setPosition(0.52);

        motorOuttake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorOuttake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorOuttake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        resetStartTime();
    }

    @Override
    public void loop() {
        leftX = gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y;
        rightX = gamepad1.right_stick_x;
        rightY = gamepad1.right_stick_y;

        motor0Raw = -leftY + leftX + (rightX * 0.75);
        motor1Raw = leftY + leftX + (rightX * 0.75);
        motor2Raw = leftY - leftX + (rightX * 0.75);
        motor3Raw = -leftY - leftX + (rightX * 0.75);

        rawMax = Math.max(Math.abs(motor0Raw), Math.max(Math.abs(motor1Raw), Math.max(Math.abs(motor2Raw), Math.abs(motor3Raw))));

        if (rawMax > 1) {
            motor0Scaled = motor0Raw / rawMax;
            motor1Scaled = motor1Raw / rawMax;
            motor2Scaled = motor2Raw / rawMax;
            motor3Scaled = motor3Raw / rawMax;
        } else {
            motor0Scaled = motor0Raw;
            motor1Scaled = motor1Raw;
            motor2Scaled = motor2Raw;
            motor3Scaled = motor3Raw;
        }

        motorFrontLeft.setPower(motor0Scaled);
        motorFrontRight.setPower(motor1Scaled);
        motorBackRight.setPower(motor2Scaled);
        motorBackLeft.setPower(motor3Scaled);

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
            motorIntakeMiddle.setPower(1);
        } else if (intakeState == Intake.OUTTAKE) {
            motorIntakeRight.setPower(-0.35);
            motorIntakeLeft.setPower(0.35);
            motorIntakeMiddle.setPower(-1);
        } else if (intakeState == Intake.RELEASE) {
            motorIntakeLeft.setPower(1);
            motorIntakeRight.setPower(1);
        } else if (intakeState == Intake.OFF) {
            motorIntakeRight.setPower(0);
            motorIntakeLeft.setPower(0);
            motorIntakeMiddle.setPower(0);
        }

    }
}
