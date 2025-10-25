package org.firstinspires.ftc.teamcode.subSystems;

import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer {
    private final Servo rightServo;
    private final Servo leftServo;


    public Transfer(HardwareMap hardwareMap){
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
    }


    public class RaiseLeftTask extends Task {
        private final double leftUpPos;
        public RaiseLeftTask(RobotContext robotContext) {
            super(robotContext);
            leftUpPos = 1;
            leftServo.setPosition(leftUpPos);
        }

        public void initialize(RobotContext robotContext){
            leftServo.setPosition(0);
        }

        protected boolean run(RobotContext robotContext){
            return leftServo.getPosition() >= leftUpPos;
        }

    }

    public class RaiseRightTask extends Task {
        private final double rightUpPos;
        public RaiseRightTask(RobotContext robotContext) {
            super(robotContext);
            rightUpPos = 1;
            rightServo.setPosition(rightUpPos);
        }

        public void initialize(RobotContext robotContext){
            rightServo.setPosition(0);
        }

        protected boolean run(RobotContext robotContext){
            return rightServo.getPosition() >= rightUpPos;
        }

    }
}
