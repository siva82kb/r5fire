/*
 * File containing functions for handling the kinematics and dynamics
 * of the R5 robot.
 * 
 * Ritwika Chouduri & Sivakumar Balasubramanian
 */

// Forward kinematics
// void r5ForwardKinematics(float rAngle, float lAngle, robotEndpointPosition_t &epPos) {

// }

//  ActualPosition ForwardKinematics(float, float){
//       ActualPosition posCord;

//       float E = -2*r*(l0 + l*cos(posM2*0.01745) - l*cos(posM1*0.01745));
//       float F = 2*r*l*(sin(posM1*0.01745) - sin(posM2*0.01745));
//       float G = sq(l0) + 2*sq(l) + 2*l*l0*(cos(posM2*0.01745) - cos(posM1*0.01745)) - 2*sq(l)*cos(posM2*0.01745 - posM1*0.01745);
      
//       float Phi1 = 2*(atan((-F + sqrt(sq(E) + sq(F) - sq(G)))/(G-E)))*57.2958;
//       posCord.Xp_actual = l*cos(posM1*0.01745)+ r*cos(Phi1*0.01745);
//       posCord.Yp_actual = l*sin(posM1*0.01745)+ r*sin(Phi1*0.01745);

// //      float E = 2*r*(l0 + l*cos(posM2*0.01745) - l*cos(posM1*0.01745));
// //      float F = 2*r*l*(sin(posM2*0.01745) - sin(posM1*0.01745));
// //      float G = sq(l0) + 2*sq(l) + 2*l*l0*(cos(posM2*0.01745) - cos(posM1*0.01745)) - 2*sq(l)*cos(posM2*0.01745 - posM1*0.01745);
// //      
// //      float Phi2 = 2*(atan((-F - sqrt(sq(E) + sq(F) - sq(G)))/(G-E)))*57.2958;
// //      posCord.Xp_actual = l0 + l*cos(posM2*0.01745)+ r*cos(Phi2*0.01745);
// //      posCord.Yp_actual = l*sin(posM2*0.01745)+ r*sin(Phi2*0.01745);
      
//       return posCord;
// }