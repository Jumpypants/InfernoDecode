package org.firstinspires.ftc.teamcode.subSystems;

import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer {
    private final Servo rightServo;
    private final Servo leftServo;
    public static double LEFT_UP_POS;
    public static double RIGHT_UP_POS;
    public static double LEFT_DOWN_POS;
    public static double RIGHT_DOWN_POS;


    public Transfer(HardwareMap hardwareMap){
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        LEFT_UP_POS = 1;
        RIGHT_UP_POS = 1;
        LEFT_DOWN_POS = 0;
        RIGHT_DOWN_POS = 0;

    }


    public class MoveLeftTask extends Task {
        public MoveLeftTask(RobotContext robotContext, double leftPos) {
            super(robotContext);
            leftPos = 1;
            leftServo.setPosition(leftPos);
        }

        public void initialize(RobotContext robotContext){
            leftServo.setPosition(0);
        }

        protected boolean run(RobotContext robotContext){
            return leftServo.getPosition() >= LEFT_UP_POS;
        }

    }

    public class MoveRightTask extends Task {
        public MoveRightTask(RobotContext robotContext, double rightPos) {
            super(robotContext);
            rightServo.setPosition(rightPos);
        }

        public void initialize(RobotContext robotContext){
            rightServo.setPosition(0);
        }

        protected boolean run(RobotContext robotContext){
            return rightServo.getPosition() >= RIGHT_UP_POS;
        }

    }
}
