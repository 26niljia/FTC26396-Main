package org.firstinspires.ftc.team26396.opmodes.Subsystems.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team26396.constants.Constants;

public class LinearSlide {

    private final DcMotorEx linearSlideMotor;
    private static final double LINEAR_SLIDE_POWER = 0.5;
    // Positions in degrees (as doubles)
    private static final double INIT_DEGREES = 14.0;
    private static final double GROUND_DEGREES = 10.0;   // Default position (0 degrees)
    private static final double MIN_DEGREES_FOR_WRIST = 28.0;
    private static final double LOW_DEGREES = 12.0;     // Position to pick up from the ground (15 degrees)
    private static final double HIGH_DEGREES = 71.0;    // Position to place into low basket (45 degrees)
    private static final double MAX_DEGREES = 95.0;     // Position to place into an high basket (70 degrees)

    // Formula to calculate ticks per degree
    final double LINEAR_SLIDE_TICKS_PER_DEGREE = 19.2032086;
//            145.1 // encoder ticks per rotation of the bare RS-555 motor
//                    * 5.2 // gear ratio of the 5.2:1 Yellow Jacket gearbox
//                    * 5.0 // external gear reduction, a 20T pinion gear driving a 100T hub-mount gear (5:1 reduction)
//                    * 1 / 360.0 * 2; // we want ticks per degree, not per rotation


    // Pre-calculated arm positions in encoder ticks based on degrees
    private final double INIT_POSITION_TICKS = INIT_DEGREES* LINEAR_SLIDE_TICKS_PER_DEGREE;
    private final double GROUND_POSITION_TICKS = GROUND_DEGREES * LINEAR_SLIDE_TICKS_PER_DEGREE;
    private final double MIN_POS_FOR_WRIST_TICKS = MIN_DEGREES_FOR_WRIST * LINEAR_SLIDE_TICKS_PER_DEGREE;
    private final double LOW_POSITION_TICKS = LOW_DEGREES * LINEAR_SLIDE_TICKS_PER_DEGREE;
    private final double HIGH_POSITION_TICKS = HIGH_DEGREES * LINEAR_SLIDE_TICKS_PER_DEGREE;
    private final double MAX_POSITION_TICKS = MAX_DEGREES * LINEAR_SLIDE_TICKS_PER_DEGREE;

    // Fudge factor for fine control of arm adjustments
    //Larger FudgeFactor = More Jerky Movements
    private static final double FUDGE_FACTOR = 5.0;


    public LinearSlide(HardwareMap hardwareMap) {

        linearSlideMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, Constants.HardwareConstants.LINEAR_SLIDE_MOTOR_NAME);

        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public Action initLinearSlide() {
        return new InitLinearSlide();
    }

    public Action extendArmForward() {
        return new ExtendLinearSlide();
    }

    public Action retractArmBackward() {
        return new RetractArmBackward();
    }

    public Action extendSlideForPickFromPool() {
        return new ExtendSlideForPickFromPool();
    }

    public class InitLinearSlide implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                linearSlideMotor.setPower(LINEAR_SLIDE_POWER);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                initialized = true;
            }

            // checks lift's current position
            return setLinearSlidePosition(packet, INIT_POSITION_TICKS, LINEAR_SLIDE_POWER);

        }
    }

    public class ExtendLinearSlide implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                linearSlideMotor.setPower(LINEAR_SLIDE_POWER);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                initialized = true;
            }

            // checks lift's current position

            return setLinearSlidePosition(packet, MIN_POS_FOR_WRIST_TICKS, LINEAR_SLIDE_POWER);

        }
    }

    public class RetractArmBackward implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                linearSlideMotor.setPower(LINEAR_SLIDE_POWER);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                initialized = true;
            }

            // checks lift's current position
            return setLinearSlidePosition(packet, INIT_POSITION_TICKS, LINEAR_SLIDE_POWER);

        }
    }

    public class ExtendSlideForPickFromPool implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                linearSlideMotor.setPower(LINEAR_SLIDE_POWER);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                initialized = true;
            }

            // checks lift's current position
            return setLinearSlidePosition(packet, MAX_POSITION_TICKS, LINEAR_SLIDE_POWER);

        }
    }


    private boolean setLinearSlidePosition(TelemetryPacket packet, double targetPosition, double linearSlidePower) {

        if (targetPosition < GROUND_POSITION_TICKS || targetPosition > (MAX_POSITION_TICKS+5.0)) {
            targetPosition = LOW_POSITION_TICKS; // Set to low/ground position if out of range
        }

        // Convert target position in ticks (double) and set motor
        double currentPosition = linearSlideMotor.getCurrentPosition();
        packet.put("Linear Slide Position", currentPosition);

        linearSlideMotor.setTargetPosition((int) targetPosition);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideMotor.setPower(linearSlidePower);

        if (currentPosition < targetPosition) {
            // true causes the action to rerun
            return true;
        } else {
            // false stops action rerun
            linearSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            linearSlideMotor.setPower(0);
            return false;
        }
        // overall, the action powers the lift until it surpasses
        // 3000 encoder ticks, then powers it off
    }
}
