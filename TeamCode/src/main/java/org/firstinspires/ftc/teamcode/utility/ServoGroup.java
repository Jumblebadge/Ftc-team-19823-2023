package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ServoGroup {

    public final ServoImplEx[] servos;
    public ServoGroup(ServoImplEx... servos){
        this.servos = servos;
    }

    public void setPosition(double... positions){
        for (int i = 0; i < servos.length; i++){
            servos[i].setPosition(positions[i]);
        }
    }

    public void setPositions(double position, int servo){
        servos[servo].setPosition(position);
    }

    public double getPosition(int servo){
        return servos[servo].getPosition();
    }

    public void PWMrelease() {
        for (ServoImplEx servo : servos) {
            servo.setPwmDisable();
        }
    }


}
