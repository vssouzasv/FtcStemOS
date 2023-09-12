/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.comandos.motor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="Motor to sP", group="Robot")
public class ControleMotor extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor motor   = null;

    // Constantes para conversão
    static final double     COUNTS_PER_MOTOR_REV    = 560; // CPR do motor, entre no site da fabricante
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;  // Redução entre motor e roda
    
    static final double     WHEEL_DIAMETER_INCHES   = 1.96;
    double fatorDeConversao = (COUNTS_PER_MOTOR_REV  * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Iniciando");
        telemetry.update();
        // Inicia o mtoor
        motor  = hardwareMap.get(DcMotor.class, "motor");
        // Seta o motor como FORWARD
        motor.setDirection(DcMotor.Direction.FORWARD);
        // Para e reseta os encoder
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Espera o start
        waitForStart();

        // Para adicionar objetivos apenas chame a função addSetpoint(valor em polegadas)
        // Como não resetamos o encoder a cada movimento, a rotação será relativa ao motor
        // Portanto se eu avançar para 10, e quiser voltar a 0, é necessário colocar -10
        addSetpoint(10);
        sleep(1000);
        addSetpoint(-10);
    }

    public void addSetpoint(int setPoint) {
        // Define um objetivo
        int newSetPoint = motor.getCurrentPosition() + (int)(setPoint * fatorDeConversao);
        // Define uma posição alvo
        motor.setTargetPosition(newSetPoint);
        // Seta o motor para ir até a posição
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Coloca o motor para se mover
        motor.setPower(0.5);

        // Distância percorrida pelo motor
        while(motor.isBusy()) {
            telemetry.addData("Distância percorrida: ", (motor.getCurrentPosition() / fatorDeConversao));
            telemetry.update();
        }
        // Para os motores
        motor.setPower(0);

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(500);
    }
}