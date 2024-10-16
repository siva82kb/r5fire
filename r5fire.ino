#include <cmath>
#include "variables.h"

void setup() {
    Serial.begin(115200);
    // Initialize DIO pins
    // Calibration swtiches
    pinMode(SwitchM1_PIN, INPUT_PULLUP);
    pinMode(SwitchM2_PIN, INPUT_PULLUP);
    // Motor 1
    pinMode(M1PIN, OUTPUT);
    pinMode(E1PIN, OUTPUT);
    pinMode(D1PIN, OUTPUT);
    // Motor 2
    pinMode(M2PIN, OUTPUT);
    pinMode(E2PIN, OUTPUT);
    pinMode(D2PIN, OUTPUT);
    // Set start time
    startTime = millis();
    // Reset encoders
    encLeft.write(0);
    encRight.write(0);
}

// int i = 0;
void loop() {
    // Read all sensor data
    readAllSensorData();

    // Process raw sensor data


    // Read incoming serial data
    Serial.print("Left: ");
    Serial.print(angleLeft.val(0));
    Serial.print(" Right: ");
    Serial.print(angleRight.val(0));
    Serial.print(" \n");
    //     Serial.print(',');
    //     Serial.print(Yp);
    //     Serial.print('\t');
    //     Serial.print(Theta1);
    //     Serial.print(',');
    //     Serial.print(Theta2);
    // Update control

    // Write outgoing serial data

    // if (i == 0) {
    //     X0 = 15.0;
    //     Y0 = 18.0;
    // } else {
    //     X0 = EEXp[i - 1];
    //     Y0 = EEYp[i - 1];
    // }

    // Xf = EEXp[i];
    // Yf = EEYp[i];

    // T0 = 10 * (millis() - startTime) / 1000;

    // if (T0 <= Tf) {

    //     DesMJTpos = minimumJerkTrajectory(X0, Xf, Y0, Yf, T0, Tf);
    //     Xp = DesMJTpos.Xd;
    //     Yp = DesMJTpos.Yd;

    //     DesAng = InverseKinematics(Xp, Yp);
    //     Theta1 = 180 - DesAng.Ang1;
    //     Theta2 = -DesAng.Ang2;
    //     if (Theta1 > 117) {
    //     Theta1 = 117;
    //     }
    //     if (Theta2 < -117) {
    //     Theta2 = -117;
    //     }

    //     if (DesAng.Ang1 > 180) {
    //     Theta1 = 0;
    //     }
    //     if (DesAng.Ang2 < 0) {
    //     Theta2 = 0;
    //     }

    //     if (digitalRead(SwitchM1_PIN) == LOW) {
    //     Enc1In.write(0);
    //     } else {
    //     Enc1Count = Enc1In.read();
    //     }
    //     posM1 = (Enc1Count * 360) / (4 * 4096 * 43);  //PPR =4096, quad=4, gear ratio = 43:1

    //     if (digitalRead(SwitchM2_PIN) == LOW) {
    //     Enc2In.write(0);
    //     } else {
    //     Enc2Count = Enc2In.read();
    //     }
    //     posM2 = (Enc2Count * 360) / (4 * 4096 * 43);


    //     present_error1 = Theta1 - posM1;
    //     derivative1 = previous_error1 - present_error1;
    //     ConOutM1 = 37 + fabs((KpM1 * present_error1) + (KdM1 * derivative1));
    //     previous_error1 = present_error1;

    //     present_error2 = Theta2 - posM2;
    //     derivative2 = previous_error2 - present_error2;
    //     ConOutM2 = 37 + fabs((KpM2 * present_error2) + (KdM2 * derivative2));
    //     previous_error2 = present_error2;

    //     pwm1Val = (int)(ConOutM1);
    //     if (pwm1Val > 229) {
    //     pwm1Val = 229;
    //     } else if (pwm1Val < 26) {
    //     pwm1Val = 26;
    //     }

    //     pwm2Val = (int)(ConOutM2);
    //     if (pwm2Val > 229) {
    //     pwm2Val = 229;
    //     } else if (pwm2Val < 26) {
    //     pwm2Val = 26;
    //     }

    //     Serial.print(Xp);
    //     Serial.print(',');
    //     Serial.print(Yp);
    //     Serial.print('\t');
    //     Serial.print(Theta1);
    //     Serial.print(',');
    //     Serial.print(Theta2);
    //     Serial.print('\t');
    //     Serial.print(posM1);
    //     Serial.print(',');
    //     Serial.print(posM2);
    //     Serial.print('\t');
    //     Serial.print(digitalRead(SwitchM1_PIN));
    //     Serial.print(',');
    //     Serial.print(digitalRead(SwitchM2_PIN));
    //     Serial.print('\t');
    //     Serial.print(pwm1Val);
    //     Serial.print(',');
    //     Serial.print(pwm2Val);
    //     Serial.print('\t');
    //     Serial.println(T0);

    //     if (present_error1 > 2) {
    //     analogWrite(M1PIN, pwm1Val);
    //     digitalWrite(E1PIN, HIGH);
    //     digitalWrite(D1PIN, HIGH);
    //     } else if (present_error1 < -2) {
    //     analogWrite(M1PIN, pwm1Val);
    //     digitalWrite(E1PIN, HIGH);
    //     digitalWrite(D1PIN, LOW);
    //     } else {
    //     pwm1Val = 0;
    //     analogWrite(M1PIN, pwm1Val);
    //     digitalWrite(E1PIN, LOW);
    //     digitalWrite(D1PIN, LOW);
    //     }

    //     if (present_error2 > 2) {
    //     analogWrite(M2PIN, pwm2Val);
    //     digitalWrite(E2PIN, HIGH);
    //     digitalWrite(D2PIN, HIGH);
    //     } else if (present_error2 < -2) {
    //     analogWrite(M2PIN, pwm2Val);
    //     digitalWrite(E2PIN, HIGH);
    //     digitalWrite(D2PIN, LOW);
    //     } else {
    //     pwm2Val = 0;
    //     analogWrite(M2PIN, pwm1Val);
    //     digitalWrite(E2PIN, LOW);
    //     digitalWrite(D2PIN, LOW);
    //     }
    // }

    // if (T0 >= Tf) {
    //     pwm1Val = 26;
    //     pwm2Val = 26;
    //     if (i < 6 && (unsigned long)(millis() - previousMillis) >= (((Tf * 1000) / 10) + interval)) {
    //     previousMillis = millis();
    //     i = i + 1;
    //     startTime = millis();
    //     T0 = 0;
    //     }
    // }
    // if (i == 6) {
    //     i = 0;
    // }
}

// DesiredAngle InverseKinematics(float, float) {
//   DesiredAngle angle;
//   angle.Ang1 = (2 * (atan(((-(-2 * l * Yp) + sqrt(sq(-2 * l * Xp) + sq(-2 * l * Yp) - sq(sq(Xp) + sq(Yp) + sq(l) - sq(r)))) / (sq(Xp) + sq(Yp) + sq(l) - sq(r) - (-2 * l * Xp))))) * 57.2958);
//   angle.Ang2 = (2 * (atan(((-(-2 * l * Yp) + sqrt(sq(-2 * l * (Xp - l0)) + sq(-2 * l * Yp) - sq((sq(Xp) + sq(Yp) + sq(l0) + sq(l) - sq(r) - 2 * Xp * l0))))) / (sq(Xp) + sq(Yp) + sq(l0) + sq(l) - sq(r) - 2 * Xp * l0 - (2 * l * (Xp - l0))))) * 57.2958);
//   if (angle.Ang2 < 0) {
//     angle.Ang2 = -(180 + angle.Ang2);
//   } else {
//     angle.Ang2 = 180 - angle.Ang2;
//   }
//   return angle;
// }

// // Function to calculate minimum jerk trajectory
// DesiredPos_MJT minimumJerkTrajectory(float, float, float, float, float, float) {
//   DesiredPos_MJT POS;
//   float Tau = T0 / Tf;
//   if (Tau <= 1) {
//     POS.Xd = X0 + ((Xf - X0) * (10 * pow(Tau, 3) - 15 * pow(Tau, 4) + 6 * pow(Tau, 5)));
//     POS.Yd = Y0 + ((Yf - Y0) * (10 * pow(Tau, 3) - 15 * pow(Tau, 4) + 6 * pow(Tau, 5)));
//     return POS;
//   }
//   //  else{
//   //    Tau = 1;
//   //    POS.Xd = X0 + ((Xf - X0) * (10 * pow(Tau,3) - 15 * pow(Tau,4) + 6 * pow(Tau,5)));
//   //    POS.Yd = Y0 + ((Yf - Y0) * (10 * pow(Tau,3) - 15 * pow(Tau,4) + 6 * pow(Tau,5)));
//   //    return POS;
//   //  }
// }
