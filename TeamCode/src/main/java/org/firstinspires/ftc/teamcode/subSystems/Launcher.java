package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MyRobot;

public class Launcher {
    private final Servo hoodServo;
    private final Motor leftWheel;
    private final Motor rightWheel;

    private static boolean buttonPress;

    public Launcher(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        leftWheel = hardwareMap.get(Motor.class, "leftWheel");
        rightWheel = hardwareMap.get(Motor.class, "rightWheel");

        leftWheel.setInverted(true); //change the motor affected based on physical config of the launching mechanism

    }
    /**
     * Creates a new Task with the provided RobotContext.
     */
    public class SetHoodPosTask extends Task {
        private final double desiredPos;     // Target position from user
        private final double waitTime;       // How long to wait after setting position

        private long startTime;

        public SetHoodPosTask(RobotContext robotContext, double desiredPos) {
            super(robotContext);
            this.desiredPos = Math.max(0.0, Math.min(1.0, desiredPos)); // clamp 0–1
            waitTime = 0.1 * Math.abs(desiredPos - hoodServo.getPosition());
        }

        @Override
        public void initialize(RobotContext robotContext) {
            // Set immediately
            hoodServo.setPosition(desiredPos);
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            // After waiting the required time, finish
            return ELAPSED_TIME.seconds() < waitTime;
        }
    }








    public class runOuttakeTask extends Task {
        private final double POWER;

        public runOuttakeTask(MyRobot robotContext, double power) {
            super(robotContext);
            this.POWER = power;
        }


        @Override
        protected void initialize(RobotContext robotContext) {
            leftWheel.set(POWER);
            rightWheel.set(POWER);
        }

        @Override
        protected boolean run(RobotContext robotContext) { //to be cont.
            return false;
        }
    }



}



