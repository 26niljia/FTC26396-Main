package org.firstinspires.ftc.team26396.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue Left / Red Right Auto",group="LinearOpMode")
@Disabled
public class EncoderBlueLeft_RedRight extends LinearOpMode{
    // Initialize the hardware variables
    private DcMotor frontLeftMotor;
    private  DcMotor backLeftMotor;
    private  DcMotor frontRightMotor;
    private  DcMotor backRightMotor;
    private  DcMotor armMotor;
    private  DcMotor liftMotor;
    private CRServo intake;
    private ElapsedTime runtime = new ElapsedTime();
    private Servo wrist;


    // Initialize all drive constants for drive motors
    private static final double COUNTS_PER_MOTOR_REV = 537.7; // Adjust for your motor
    private static final double DRIVE_GEAR_REDUCTION = 19.2; // No gear reduction(External gear ratio is 30:30 = 1:1)
    private static final double WHEEL_DIAMETER_INCHES = 3.7796; // Wheel diameter
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    private static final double WHEELBASE_WIDTH = 14.0; // Example width in inches
    private static final double TURNING_RADIUS = WHEELBASE_WIDTH / 2;
    private static final double COUNTS_PER_RADIAN = (COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_INCHES)) * (TURNING_RADIUS);




    // Motor power settings
    private static final double ARM_POWER = 0.8;
    private static final double LIFT_POWER = 0.8;

    final double ARM_COUNTS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation

    final int ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_CLEAR_BARRIER         = 15 * ARM_COUNTS_PER_DEGREE;
    //final double ARM_SCORE_SPECIMEN        = 90 * ARM_COUNTS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 57.61 * ARM_COUNTS_PER_DEGREE;
    final double  ARM_SCORE_SAMPLE_IN_HIGH = 70.97 * ARM_COUNTS_PER_DEGREE;

    private static final double MAX_LINEAR_SLIDE_EXTENSION = 38.42521;//In inches
    private static final double LINEAR_SLIDE_COUNTS_PER_MOTOR_REV = 384.5;//Linear Slide uses the 435 RPM Motor
    private static final double ROTATIONS_FOR_MAX = 8.13;
    private static final double INCHES_PER_ROTATION = MAX_LINEAR_SLIDE_EXTENSION/ROTATIONS_FOR_MAX;
    private static final double ARM_COUNTS_PER_INCH = LINEAR_SLIDE_COUNTS_PER_MOTOR_REV/INCHES_PER_ROTATION;

    final double SLIDE_HIGH = 18.5 * ARM_COUNTS_PER_INCH;//18.5 inches for the high basket(Theoretical)
    final double SLIDE_LOW = 7.9375 * ARM_COUNTS_PER_INCH;//7.15/16 inches for the low basket(Theoretical)
    final double SLIDE_ZERO = 0 * ARM_COUNTS_PER_INCH;//0 inches for base position

    private static final double WRIST_COLLECT = 1.0;  // Position for collecting
    private static final double WRIST_DEPOSIT = 0.0; // Position for depositing
    private static final double WRIST_HOME = 0.5;






    @Override
    public void runOpMode() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");
        Direction();
        Telemetry();
        // Reset encoders
/*        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run using encoders
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wrist.setPosition(WRIST_COLLECT);
*/

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //Put all code
            wrist.setPosition(WRIST_DEPOSIT);
            setLiftPosition(ARM_COLLAPSED_INTO_ROBOT);
            driveInches("forward",0.5,17.5);
            sleep(1000);
            setLiftPosition((int) Math.round(ARM_SCORE_SAMPLE_IN_HIGH));//Linear slide rotates to 71 degrees for high basket dropping
            setArmPosition((int) Math.round(SLIDE_HIGH));//Linear slide extends to 18.5 inches for high basket dropping
            Intake(-1.0,1000);
            setArmPosition((int) Math.round(SLIDE_ZERO));
            setLiftPosition((ARM_COLLAPSED_INTO_ROBOT));
            driveInches("backward",0.5,22.5);
            driveInches("right",0.5,10);
            Intake(1.0,1000);
            driveInches("left",0.5,10);
            driveInches("forward",0.5,22.5);
            setLiftPosition((int) Math.round(ARM_SCORE_SAMPLE_IN_HIGH));//Linear slide rotates to 71 degrees for high basket dropping
            setArmPosition((int) Math.round(SLIDE_HIGH));//Linear slide extends to 18.5 inches for high basket dropping
            driveInches("turn",0.5, Math.toRadians(180));
            driveInches("left",0.5,22);
            setLiftPosition((int) Math.round(ARM_CLEAR_BARRIER));
        }


    }

    // Set Lift position using encoder ticks - Rotational
    public void setLiftPosition(int targetPosition) {
        liftMotor.setTargetPosition((int) targetPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(LIFT_POWER);
    }

    // Set Arm position using encoder ticks - Horizontal
    private void setArmPosition(int targetPosition) {
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ARM_POWER);
    }

    private void MotorPower(double speed) {
        //set MotorPower for the OpMode
        frontLeftMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(speed);
    }

    private void driveInches(String direction, double speed, double value) {
        int targetPosition;
        // Determine the target position or radians based on the direction
        if (direction.equalsIgnoreCase("turn")) {
            targetPosition = (int) (value * COUNTS_PER_RADIAN); // 'value' is radians
        } else {
            targetPosition = (int) (value * COUNTS_PER_INCH); // 'value' is inches
        }

        switch (direction.toLowerCase()) {
            case "forward":  // Move forward
                frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + targetPosition);
                backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() + targetPosition);
                frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + targetPosition);
                backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + targetPosition);
                MotorPower(speed);
                telemetry.addData("Status", "Moving forward");
                telemetry.update();

                break;

            case "backward": // Move backward
                frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() - targetPosition);
                backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() - targetPosition);
                frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() - targetPosition);
                backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() - targetPosition);
                MotorPower(speed);
                telemetry.addData("Status", "Moving backward");
                telemetry.update();

                break;

            case "right": // Strafing right
                frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + targetPosition);
                backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() - targetPosition);
                frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() - targetPosition);
                backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + targetPosition);
                MotorPower(speed);
                telemetry.addData("Status", "Strafing right");
                telemetry.update();

                break;

            case "left": // Strafing left
                frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() - targetPosition);
                backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() + targetPosition);
                frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + targetPosition);
                backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() - targetPosition);
                MotorPower(speed);
                telemetry.addData("Status", "Strafing left");
                telemetry.update();

                break;
            case "turn": // Changing the orientation of the robot [For the OpMode code, do Math.toRadians]
                frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + targetPosition); // Move left side forward
                backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() + targetPosition); // Move left side forward
                frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() - targetPosition); // Move right side backward
                backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() - targetPosition);// Move right side backward
                MotorPower(speed);
                telemetry.addData("Status", "Turning " + value + " degrees");
                telemetry.update();
                break;
        }
    }

    private void Telemetry() {
        //Initializes telemetry on driver station
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Front Left Power", frontLeftMotor.getPower());
        telemetry.addData("Front Right Power", frontRightMotor.getPower());
        telemetry.addData("Back Left Power", backLeftMotor.getPower());
        telemetry.addData("Back Right Power", backRightMotor.getPower());
        telemetry.addData("Linear Arm Horizontal Extension Power", armMotor.getPower());
        telemetry.addData("Linear Arm Rotation Power", liftMotor.getPower());
        //If else statements for displaying telemetry for Intake
        if (intake.getPower() == 1) {
            telemetry.addData("Status:", "Intake");
        } else if (intake.getPower() == -1) {
            telemetry.addData("Status:", "Outtake");
        } else if (intake.getPower() == 0) {
            telemetry.addData("Status:", "Rest");
        }
        telemetry.update();
    }

    private void Direction() {
        // Set motor directions: Reverse the motors on one side to ensure correct movement
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);

    }

    public void ExtendArm(double power, long time) {
        //set armMotor(Horizontal) Power
        armMotor.setPower(power);
    }

    public void Intake(double power, long time) {
    }

    public void Outtake(double power, long time) {
        //set Outtake Power
        intake.setPower(-1.0);
    }

    public void WristCode(Servo wrist) {
        // Ensure the wrist starts at the WRIST_COLLECT position
        wrist.setPosition(WRIST_COLLECT);
    }
}
