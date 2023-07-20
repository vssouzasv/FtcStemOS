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

package org.firstinspires.ftc.teamcode.teleoperados.mecanum;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

// Classe usada para inicializar todos os sensores e acionadores usados no robô
public class HardwareClassTracao {
    OpMode opMode;
    //Declaração dos motores
    DcMotor motorEsquerda = null;
    DcMotor motorDireita = null;
    DcMotor motorEsquerdaTras = null;
    DcMotor motorDireitaTras = null;

    // Objeto da interface de giroscópio
    IMU imu;

    // Método construtor da classe, usado para obter uma variável importante na inicialização
    public HardwareClassTracao(OpMode opMode) {
        this.opMode = opMode;
    }

    // Método que usamos para inicializar o hardware do sistema
    public void hardwareGeral() {

        /*
         * =============================================================================
         *                                  GYRO
         * =============================================================================
         */
        // Objeto IMU (declarado com nome imu na Ds)
        imu = opMode.hardwareMap.get(IMU.class, "imu");

        // Orientação do Control Hub/Expansion Hub
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Inicializa o giroscópio
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        /*
         * =============================================================================
         *                                  ACIONADORES
         * =============================================================================
         */

        // Motores que usaremos para mechanum
        motorEsquerda = opMode.hardwareMap.get(DcMotor.class, "motor_esquerda");
        motorEsquerdaTras = opMode.hardwareMap.get(DcMotor.class, "motor_esquerdatras");
        motorDireita = opMode.hardwareMap.get(DcMotor.class, "motor_direita");
        motorDireitaTras = opMode.hardwareMap.get(DcMotor.class, "motor_direitatras");
    }

}

