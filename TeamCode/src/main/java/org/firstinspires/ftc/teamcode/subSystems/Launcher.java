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
    private final Servo hoodServo1;
    private final Motor leftWheel;
    private final Motor rightWheel;

    private static boolean buttonPress;

    public Launcher(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        hoodServo1 = hardwareMap.get(Servo.class, "hoodServo1");
        leftWheel = hardwareMap.get(Motor.class, "leftWheel");
        rightWheel = hardwareMap.get(Motor.class, "rightWheel");

        leftWheel.setInverted(true); //change the motor affected based on physical config of the launching mechanism

    }
    /**
     * Creates a new Task with the provided RobotContext.
     */
    public class SetHoodPosTask extends Task {
        private final double minPos = 0.0;  // hood down
        private final double maxPos = 1.0;  // hood up
        private final double step = 0.1;    // how much to move each button press

        private double currentPos;
        private final boolean moveUp, moveDown;

        public SetHoodPosTask(RobotContext robotContext, boolean moveUp) {
            super(robotContext);
            this.moveUp = moveUp;
            this.moveDown = !moveUp;
        }
    //while u hold a button, it moves in one direction otherwise it moves in a different direction
//once up button is pressed, the current pos will move up 0.1 if down button, the current pos will move down
        @Override
        public void initialize(RobotContext robotContext) {
            currentPos = hoodServo.getPosition(); // get current servo position

        }

        @Override
        protected boolean run(RobotContext robotContext) {

            if (moveUp) {
                currentPos += step;
            } else if (moveDown) {
                currentPos -= step;
            }
            if (currentPos > maxPos) currentPos = maxPos;
            if (currentPos < minPos) currentPos = minPos;

            if (robotContext.gamepad1.a){
                return false;
            }

            // Apply position to both servos immediately
            hoodServo.setPosition(currentPos);
            hoodServo1.setPosition(currentPos);

            return true;
        }
    }




    public class RunOuttakeTask extends Task {
        private final Motor leftWheel;
        private final Motor rightWheel;
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
            leftWheel.set(currentPower);
            rightWheel.set(currentPower);
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



