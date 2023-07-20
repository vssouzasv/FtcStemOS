package org.firstinspires.ftc.teamcode.auto2;

// Classe de constantes para desenvolver o robô e não espelhar números pelo código
public class Constants {

    /*
     * Aqui setamos as variáveis para configurar nossa tração no período autônomo
     * precisamos desses valores para calcular a contagem necessária dos encoders para
     * a distância dada por nós, por exemplo, eu quero que o robô se mova 5 metros, entretanto
     * meu encoder somente faz a leitura por ticks, então, quanto seria 5 metros em ticks
     * Para isso que temos esses valores.
     * VALORES DE COMPRIMENTO EM POLEGADAS
     */
    static final double COUNTS_PER_MOTOR_REV = 1440; // CPR do motor, entre no site da fabricante
    static final double DRIVE_GEAR_REDUCTION = 1.0;  // Redução entre motor e roda
    static final double WHEEL_DIAMETER_INCHES = 4.0;  // Diâmetro da roda
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415); // Fator de conversão
    static final double DRIVE_SPEED = 0.6; // Com que tensão os motores se moverão para frente
    static final double TURN_SPEED = 0.5; // Com que tensão os motores farão o robô girar

}
