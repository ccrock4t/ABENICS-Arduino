// Author: Christian Hahm
// Date: February 10, 2024
// ABENICS driver code

#include <math.h>
#include <MultiStepper.h>
#include <AccelStepper.h>

/// ========== ABENICS
// there are 2 driving modules: A and B, each driving one of the 2 MP gears, which in turn drive the CS gear
// SET HERE the digial output pins
AccelStepper stepper_motor_A1(AccelStepper::FULL4WIRE, 22, 23, 24, 25); // driver gear A
AccelStepper stepper_motor_A2(AccelStepper::FULL4WIRE, 28, 29, 30, 31); // worm gear A
AccelStepper stepper_motor_B1(AccelStepper::FULL4WIRE, 44, 45, 46, 47); // driver gear B
AccelStepper stepper_motor_B2(AccelStepper::FULL4WIRE, 50, 51, 52, 53); // worm gear B

MultiStepper steppers; // allows for coordinated control of the steppers, so they all reach their target angle at the same time.

float target_CS_Gear_angle[] = {0, 0, 0}; // what euler angles, in radians, that the user is trying to make the CS gear reach
float target_MP_Gear_angles[] = { {0, 0}, {0, 0}}; // the roll and pitch angles, in radians, to set the MP gears (r.A, p.A, r.B, p.B) 
/// ==========


// put your setup code here, to run once:
void setup() {
  // allow the user to use keyboard
  Serial.begin(9600); 

  // SET HERE the maximum speed the stepper motor can go
  int maxSpeed = 15; 

  // set maximum motor speed
  stepper_motor_A1.setMaxSpeed(maxSpeed);
  stepper_motor_A2.setMaxSpeed(maxSpeed);
  stepper_motor_B1.setMaxSpeed(maxSpeed);
  stepper_motor_B2.setMaxSpeed(maxSpeed);

  // add to multistepper
  steppers.addStepper(stepper_motor_A1);
  steppers.addStepper(stepper_motor_A2);
  steppers.addStepper(stepper_motor_B1);
  steppers.addStepper(stepper_motor_B2);
}


void loop() {
  // put your main code here, to run repeatedly:
  Send_User_Input_To_Arduino();
  Calculate_Kinematics();
  Rotate_CS_Gear_To_Target_Angle();
}


void Calculate_Kinematics(){
  float beta = DegToRad(90); 

  float Cr = cos(r);
  float Cy = cos(y);
  float Cp = cos(p);
  float Cb = cos(beta);
  float Sr = sin(r);
  float Sy = sin(y);
  float Sp = sin(p);
  float Sb = sin(beta);

  
  float theta_A1 = atan((Cr*Sy + Cy*Sp*Sr)/(Cr*Cy*Sp - Sr*Sy)); // (Equation 54)
  float theta_A2 = acos(Cp*Cy); // (Equation 55)
  float theta_B1 = -atan((Cy*Cb*Cr + Sy*(Sb*Cp - Cb*Sp*Sr))/(Cr*Sp*Sy + Cy*Sr)); // (Equation 57)
  float theta_B2 = acos(Cy*Cr*Sb - Sy*(Cb*Cp + Sn*Sp*Sr)); // (Equation 58)

  // (Equation 17), MP gear roll
  float theta_rA = theta_A1; 
  float theta_rB = theta_B1;

  // (Equation 18), MP gear pitch
  float theta_pA = -2*theta_A2; 
  float theta_pB = -2*theta_B2; 

  output_motor_angles[0] = {theta_rA, theta_pA};
  output_motor_angles[1] = {theta_rB, theta_pB};
}

void Rotate_CS_Gear_To_Target_Angle(){
  long positions[4]; // the positions of the steppers

  float screw_gear_to_drive_train_factor = (20/2*pi); // 20 teeth on each drive train gear (worm/driver)
  float driver_gear_to_MP = 1;
  float worm_gear_to_MP = (16/2*pi); // 16 teeth on the MP gear

  float theta_rA = output_motor_angles[0][0];
  float theta_pA = output_motor_angles[0][1];
  float theta_rB = output_motor_angles[1][0];
  float theta_pB = output_motor_angles[1][1];

  positions[0] = screw_gear_to_drive_train_factor*driver_gear_to_MP*theta_rA; // driver gear A
  positions[1] = screw_gear_to_drive_train_factor*worm_gear_to_MP*theta_pA; // worm gear A

  positions[2] = screw_gear_to_drive_train_factor*driver_gear_to_MP*theta_rB; // driver gear B
  positions[3] = screw_gear_to_drive_train_factor*worm_gear_to_MP*theta_pB; // worm gear B

  // now, set the positions
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // runs the stepper motors. Blocks the program until finished
  delay(1000); // let things settle down
}

void Send_User_Input_To_Arduino(){
  if (Serial.available() > 0)
    {
      int inByte = Serial.read();
      
      switch (inByte)
      {
        case 'test':  { 
          delay(10);
          break;
        }
         default:   {
          break;
        }
      }
    }
}

float DegToRad(float degrees){
  return degrees * pi / 180;
}