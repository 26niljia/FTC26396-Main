package org.firstinspires.ftc.team26396.opmodes.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.team26396.opmodes.Subsystems.HangCode;
import org.firstinspires.ftc.team26396.opmodes.Subsystems.IntakeCode;
import org.firstinspires.ftc.team26396.opmodes.Subsystems.PresetArmCode;
import org.firstinspires.ftc.team26396.opmodes.Subsystems.WristCode;

@TeleOp(name = "TeleOpRobotCentric", group = "TeleOpFINAL")
public class RobotCentricDrive extends LinearOpMode {

    // Subsystems
    private PresetArmCode armControl;
    private IntakeCode intakeControl;
    private WristCode wristControl;
    private HangCode hangControl;
    DcMotorEx linearSlideMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare the motors and servos
//        DcMotorEx linearSlideMotor = hardwareMap.get(DCMotorEx,"armMotor");
         linearSlideMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "armMotor");

        DcMotor armMotor = hardwareMap.dcMotor.get("liftMotor");
        CRServo intakeServo = hardwareMap.get(CRServo.class, "intake");
        Servo wristServo = hardwareMap.get(Servo.class, "wrist");
        DcMotor HangMotor1 = hardwareMap.dcMotor.get("HM1");
        DcMotor HangMotor2 = hardwareMap.dcMotor.get("HM2");


        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        if (isStopRequested()) return;

        // Initialize and configure IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(imuParameters);

        // Set zero power behavior
        linearSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        linearSlideMotor.setPositionPIDFCoefficients(10.0);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        HangMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        HangMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize subsystems
        armControl = new PresetArmCode(linearSlideMotor, armMotor);
        intakeControl = new IntakeCode(intakeServo);
        wristControl = new WristCode(wristServo);
        hangControl = new HangCode(HangMotor1, HangMotor2);
        waitForStart();

        while (opModeIsActive()) {


            double y = Math.pow(-gamepad1.left_stick_y,3); // Remember, Y stick value is reversed
            double x = Math.pow(gamepad1.left_stick_x * 1.1,3); // Counteract imperfect strafing
            double rx = Math.pow(gamepad1.right_stick_x,3);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 2);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            // ARM CONTROL
            armControl.controlArmAndSlide(gamepad2);
            /*
            LINEAR SLIDE
            Gamepad2 Trigger:
            a) Using Left Trigger - move linear slide backward
            b) Using Right Trigger - move linear slide forward

            ARM
            Gamepad2 Dpad:
            c) Using Dpad UP - Lift to highest position, for hanging
            d) Using Dpad DOWN - Lift to Lowest position, pick up blocks from submersible
            e) Using Dpad LEFT - Lift to low position, i.e Low Basket
            f) Using Dpad RIGHT - Lift to high position, i.e Upper Basket
             */

            // INTAKE CONTROL
            intakeControl.controlIntake(gamepad1.right_bumper, gamepad1.left_bumper);
            /*
            Gamepad1 Trigger:
            a) Using Left Trigger - pushes block OUTWARD
            b) Using Right Trigger - take block IN
             */

            // WRIST CONTROL
            wristControl.controlWrist(gamepad2.square, gamepad2.circle);
            /*
            Gamepad2 Buttons (PS4 Controller):
            a) Using Square Button - Bends Wrist to the Left
            b) Using Circle Trigger - Makes Wrist straight
             */

            // HANG CONTROL
            hangControl.controlHang(gamepad1);
            /*
            Gamepad1 Buttons (Logitech Controller):
            a) Using A button - Extends the linearSlides
            b) Using B button - Retracts the linearSlides
            Note: HangMotor1 Encoder Value should ALWAYS be equal to HangMotor2 Encoder Value
             */

            // TELEMETRY
            telemetry.addData("Front Left Power", frontLeftMotor.getPower());
            telemetry.addData("Back Left Power", backLeftMotor.getPower());
            telemetry.addData("Front Right Power", frontRightMotor.getPower());
            telemetry.addData("Back Right Power", backRightMotor.getPower());
            telemetry.addData("LinearSlide Motor Power", linearSlideMotor.getPower());
            telemetry.addData("Arm Motor Power", armMotor.getPower());
            telemetry.addData("Intake Power", intakeServo.getPower());
            telemetry.addData("Wrist Position", wristServo.getPosition());
            telemetry.addData("Linear Slide Encoder", linearSlideMotor.getCurrentPosition());
            telemetry.addData("Arm Motor Encoder", armMotor.getCurrentPosition());
            telemetry.addData("Hang Motor1 Encoder", HangMotor1.getCurrentPosition());
            telemetry.addData("Hang Motor2 Encoder", HangMotor2.getCurrentPosition());
            telemetry.update();
        }
    }
}
