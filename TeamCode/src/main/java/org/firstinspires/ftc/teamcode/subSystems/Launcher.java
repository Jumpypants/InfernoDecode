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

    }


    /**
     * Creates a new Task with the provided RobotContext.
     */
    public class SetHoodPosTask extends Task {
        public int currentPos;
        public int maxPos;
        public int step;
        public int minPos;


        public SetHoodPosTask(RobotContext robotContext, double targetPos) {
            super(robotContext);

        }

        @Override
        public void initialize(RobotContext robotContext) {
            hoodServo.setPosition(0);
            currentPos++;
            // Move the hood servo to the desired position
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            // If up button is pressed, increase position gradually
            if (currentPos < maxPos) {
                currentPos += step;
                if (currentPos > maxPos) currentPos = maxPos;
                hoodServo.setPosition(currentPos);
            }

            // If down button is pressed, decrease position gradually
            if (currentPos > minPos) {
                currentPos -= step;
                if (currentPos < minPos) currentPos = minPos;
                hoodServo.setPosition(currentPos);
            }

            return false; // keep task running indefinitely
        }
    }


    public class RunOuttakeTask extends Task {

        private final double targetPower;  // power to run outtake at
        private final double rampStep = 0.05;
        private double currentPower = 0.0;

        public RunOuttakeTask(RobotContext robotContext, Motor leftWheel, Motor rightWheel, double targetPower) {
            super(robotContext);
            this.leftWheel = leftWheel;
            this.rightWheel = rightWheel;
            this.targetPower = targetPower;
        }

        @Override
        public void initialize(RobotContext robotContext) {
            leftWheel.set(0);
            rightWheel.set(0);
            currentPower = 0.0;
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            // Gradually ramp up to targetPower
            if (currentPower < targetPower) {
                currentPower += rampStep;
                if (currentPower > targetPower) currentPower = targetPower;
            }

            // Apply power to both wheels
            leftWheel.set(currentPower);
            rightWheel.set(currentPower);

            // Keep running until manually stopped or task is removed
            return false;
        }
    }


}



