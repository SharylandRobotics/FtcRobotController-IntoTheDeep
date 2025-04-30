package org.firstinspires.ftc.team12397.v2;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.*;

public class RoadRunnerActions {
    RobotHardware robotHW;
    public RoadRunnerActions(RobotHardware robot){
        robotHW = robot;
    }

    // elbow object class
    public class VerticalSlides {
        // class vars
        private DcMotorEx verticalSlideL;
        private DcMotorEx verticalSlideR;

        // class constructor & hardware mapper
        public VerticalSlides(HardwareMap hardwareMap){
            verticalSlideL = hardwareMap.get(DcMotorEx.class, "slideMotorL");
            verticalSlideR = hardwareMap.get(DcMotorEx.class, "slideMotorR");

            verticalSlideL.setDirection(DcMotorSimple.Direction.FORWARD);
            verticalSlideR.setDirection(DcMotorSimple.Direction.REVERSE);

            verticalSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            verticalSlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // actual action class/ do-er
        public class slideToPos implements Action {
            private double Tpos; // in counts

            /**
             *
             * @param position give pure ticks/counts, use the robot. variables
             */
            public slideToPos(double position){
                Tpos = position;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robotHW.setSlidePosition(Tpos);
                return false;
            }

        }

        // usable method/ action class shortcut
        public Action verticalSlidesToPos(double position){
            return new slideToPos(position);
        }
    }

    public class HorizontalExtender {
        // class vars
        private Servo extenderL;
        private Servo extenderR;

        // class constructor & hardware mapper
        public HorizontalExtender(HardwareMap hardwareMap){
            extenderL = hardwareMap.get(Servo.class, "lextend");
            extenderR = hardwareMap.get(Servo.class, "rextend");
        }

        // actual action class/ do-er
        public class extenderToPosition implements Action {
            private double Tpos; // in percentage (0 to 1)
            public extenderToPosition(double position){
                // return the smaller of the two (effectively limit to maximum count)
                Tpos = position;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robotHW.setExtensionPos(Tpos);
                return false;
            }

        }

        // usable method/ action class shortcut
        public Action extenderToPos(double pos){
            return new extenderToPosition(pos);
        }
    }

    public class InClawPincher {
        // class vars
        private Servo pincher;

        // class constructor & hardware mapper
        public InClawPincher(HardwareMap hardwareMap){
            pincher = hardwareMap.get(Servo.class, "claw_pinch");
        }

        // actual action class/ do-er(s) -->
        public class CloseInPincher implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robotHW.setInClawPinch(1);
                return false;
            }
        }
        public class OpenInPincher implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robotHW.setInClawPinch(0);
                return false;
            }
        }
        // <--

        // usable method/ action class shortcut
        public Action closeInClaw() {
            return new CloseInPincher();
        }
        public Action openInClaw() {
            return new OpenInPincher();
        }
    }

    public class InClawYaw {
        // class vars
        private Servo yaw;

        // class constructor & hardware mapper
        public InClawYaw(HardwareMap hardwareMap){
            yaw = hardwareMap.get(Servo.class, "claw_yaw");
        }

        // actual action class/ do-er(s) -->
        public class RotateInYaw implements Action {
            private double Tpos;
            public RotateInYaw(double Tpos){
                this.Tpos = Tpos;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robotHW.setInClawYaw(Tpos);
                return false;
            }
        }

        // usable method/ action class shortcut
        public Action rotateInClawYaw(double pos) {
            return new RotateInYaw(pos);
        }
    }

    public class InClawPitch {
        // class vars
        private Servo pitch;

        // class constructor & hardware mapper
        public InClawPitch(HardwareMap hardwareMap){
            pitch = hardwareMap.get(Servo.class, "claw_pitch");
        }

        // actual action class/ do-er(s) -->
        public class RotateInPitch implements Action {
            private double Tpos;
            public RotateInPitch(double Tpos){
                this.Tpos = Tpos;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robotHW.setInClawPitchPos(Tpos);
                return false;
            }
        }

        /**
         *
         * @param pos 0 is facing the floor, 1 is to transfer, -1 is tucked in
         * @return
         */
        public Action rotateInClawPitch(double pos){
            return new RotateInPitch(pos);
        }


    }
}
