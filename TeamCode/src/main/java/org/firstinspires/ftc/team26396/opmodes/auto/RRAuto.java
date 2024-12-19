package org.firstinspires.ftc.team26396.opmodes.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.team26396.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team26396.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.team26396.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import java.util.Locale;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Autonomous(name="Basic Auto Test", group="LinearOpMode")
@Disabled
public class RRAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotor armMotor = null;
    private DcMotor liftMotor = null;
    private CRServo intake = null;


    @Override
    public void runOpMode(){
        // Initialize the hardware variables for all motors
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        intake = hardwareMap.get(CRServo.class, "intake");


        // Set motor directions: Reverse the motors on one side to ensure correct movement
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Status", "Initialized");
        telemetry.addData("Front Left Power", leftFrontDrive.getPower());
        telemetry.addData("Front Right Power",rightFrontDrive.getPower());
        telemetry.addData("Back Left Power", leftBackDrive.getPower());
        telemetry.addData("Back Right Power", rightBackDrive.getPower());
        telemetry.update();


        waitForStart();
        runtime.reset();



        while (opModeIsActive()) {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            drive.setPoseEstimate(new Pose2d(0.22, -71.00, Math.toRadians(90.00)));
            MotorPower(0.5);
            TrajectorySequence traj0 = drive.trajectorySequenceBuilder(new Pose2d(0.22, -71.00, Math.toRadians(90.00)))
                    .lineTo(new Vector2d(0.37, -28.35))
                    .lineTo(new Vector2d(-47.68, -29.52))
                    .lineToSplineHeading(new Pose2d(-59.99, -57.79, Math.toRadians(247.46)))
                    .lineToSplineHeading(new Pose2d(51.20, -67.46, Math.toRadians(-4.97)))
                    .build();
            telemetry.addData("Status", "Built Trajectory");
            telemetry.update();
            waitForStart();
            telemetry.addData("Status", "Following Trajectory");
            telemetry.update();
            if(isStopRequested()) return;


            drive.followTrajectorySequence(traj0);
            telemetry.addData("Status", "Trajectory Complete");
            telemetry.update();
            sleep(100);
            MotorPower(0);


        }
    }
    public void ExtendArm(double power, long time){
        //set armMotor(Horizontal) Power
        armMotor.setPower(power);
    }
    public void RotateArm(double power, long time){
        //set liftMotor(Rotation) Power
        liftMotor.setPower(power);
    }
    public void Intake(double power, long time){
        //set intake Power
        intake.setPower(1.0);
    }
    public void Outtake(double power, long time){
        //set Outtake Power
        intake.setPower(1.0);
    }
    public void MotorPower(double power){
        //set MotorPower for the OpMode
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
    }






}






