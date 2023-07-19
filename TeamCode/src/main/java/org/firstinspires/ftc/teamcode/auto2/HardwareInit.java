package org.firstinspires.ftc.teamcode.auto2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class HardwareInit {
     DcMotor motorEsquerda   = null;
     DcMotor motorDireita  = null;
     DcMotor motorEsquerdaTras = null;
     DcMotor motorDireitaTras = null;
    DcMotor[] motores;
    OpMode opMode;
    public HardwareInit(OpMode opMode) {
        this.opMode = opMode;
    }

    public void iniciarHardware() {
        // Inicialização dos motores
        motorEsquerda  = opMode.hardwareMap.get(DcMotor.class, "motor_esquerda");
        motorDireita = opMode.hardwareMap.get(DcMotor.class, "motor_direita");
        motorEsquerdaTras  = opMode.hardwareMap.get(DcMotor.class, "motor_esquerdaTras");
        motorDireitaTras = opMode.hardwareMap.get(DcMotor.class, "motor_direitaTras");

        // Array de motores (opcinal)
        motores = new DcMotor[]{motorEsquerda, motorDireita, motorEsquerdaTras, motorDireitaTras};

        // Um dos lados do robô sempre tera motores invertidos, portanto revertar os motores aqui
        // vale dizer que uma transmissão por engranagens inverte a direção do movimento
        motorEsquerda.setDirection(DcMotor.Direction.REVERSE);
        motorEsquerdaTras.setDirection(DcMotor.Direction.REVERSE);
        motorDireita.setDirection(DcMotor.Direction.FORWARD);
        motorDireitaTras.setDirection(DcMotor.Direction.FORWARD);

    }

    // getter para objeto do opMode, útil quando criar subsistemas
    public OpMode getOpMode() {
        return opMode;
    }
}
