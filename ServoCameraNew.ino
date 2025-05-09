#include <Servo.h> // Include the Servo library to control servo motors
 
Servo Hservo;  // Create a servo object to control the horizontal servo motor
Servo Vservo;  // Create a servo object to control the vertical servo motor

int angleX, angleY, cpt, val; // Variables for angles, counter, and serial input value

void setup()
{
   Vservo.attach(3);  // Attach the vertical servo to pin 3
   Hservo.attach(5);  // Attach the horizontal servo to pin 5
   
   // Initialize servo positions
   angleX = 90;  // Set the initial angle for the horizontal servo (mid-position)
   angleY = 35;  // Set the initial angle for the vertical servo
   
   // Move servos to their initial positions
   Hservo.write(angleX); // Set horizontal servo to initial angle
   Vservo.write(angleY); // Set vertical servo to initial angle
   delay(150); // Allow time for the servos to reach their positions
   
   // Initialize serial communication
   Serial.begin(9600);         // Begin communication with a baud rate of 9600
   Serial.setTimeout(1);       // Set the timeout for serial communication to 1ms
   delay(150); // Allow a short delay for stabilization
   
   cpt = 1; // Initialize counter to 1, used to alternate between horizontal and vertical servo control
}
 
void loop()
{
   // Check if there is data available on the serial port
   if (Serial.available() > 0)
   {
      char val;          // Variable to store the received character
      val = Serial.read(); // Read the character from the serial port
      
      // Print debug information to the serial monitor
      Serial.println("cpt"); // Print the current value of the counter
      Serial.println(cpt);
      Serial.println(val, DEC); // Print the received character as a decimal value
      
      // Control the vertical servo when counter (cpt) is 1
      if (cpt == 1)
      {
         cpt = 2; // Switch the counter to 2 for the next iteration
         
         // Check if the received value is within the valid range for the servo
         if (val >= 0 && val <= 179)
         {
            Vservo.write(val); // Move the vertical servo to the specified angle
            delay(10);         // Short delay to ensure smooth servo movement
         }
      }
      
      // Control the horizontal servo when counter (cpt) is 2
      else if (cpt == 2)
      {
         cpt = 1; // Reset the counter to 1 for the next iteration
         
         // Check if the received value is within the valid range for the servo
         if (val >= 0 && val <= 179)
         {
            Hservo.write(val); // Move the horizontal servo to the specified angle
            delay(10);         // Short delay to ensure smooth servo movement
         }
      }
   }
}
