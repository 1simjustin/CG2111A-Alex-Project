#define S0 4
#define S1 7  
#define S2 8
#define S3 9
#define sensorOut 12

/*
      Alex's State Variables
*/

// Stores frequency read by the photodiodes
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

float redColour = 0;
float greenColour = 0;
float blueColour = 0;

void setup() {
  // put your setup code here, to run once:
  setup_colour_sensor();
}



void setup_colour_sensor() {
  //setting DDRB and DDRD as output
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  //setting frequency scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  Serial.begin(9600);
}

void sendColour() {
  //set red photodiodes to be read
  //[code]
    digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  //read the output frequency
  redFrequency = pulseIn(sensorOut, LOW);
  //remap the value of the red frequency from 0 to 255
  redColour = map(redFrequency, 400, 1500, 255, 0);

  //setting green photodiodes to be read
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  greenFrequency = pulseIn(sensorOut, LOW);
  greenColour = map(greenFrequency, 415, 2400, 255, 0);

  //setting blue photodiodes to be read
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  blueFrequency = pulseIn(sensorOut, LOW);
  blueColour = map(blueFrequency, 200, 1260, 255, 0);



  float hue = calchue(redColour, greenColour, blueColour) * 60;
  Serial.println(hue);
  delay(1000);

  /*

  //Serial.println("loop");
  //converting RGB values to ranges between 0 & 1
  double redDegree = (float)redFrequency/255.0;
  double greenDegree = (float)greenFrequency/255.0;
  double blueDegree =(float) blueFrequency/255.0;

  double length = sqrt(redDegree*redDegree + greenDegree*greenDegree + blueDegree*blueDegree);
  float dotProduct = redDegree * 1.0 + greenDegree * 0.0 + blueDegree * 0.0;
  float angle = acos(dotProduct/length)*180.0 /M_PI;

  //Serial.println(angle);
     // Printing the RED (R) value
  Serial.print("R = ");
  Serial.println(redFrequency);
  delay(100);

     // Printing the RED (R) value
  Serial.print("G = ");
  Serial.println(greenFrequency);
  delay(100);

     // Printing the RED (R) value
  Serial.print("B = ");
  Serial.println(blueFrequency);
  delay(100);

     // Printing the RED (R) value
  Serial.print("Angle = ");
  Serial.println(angle);
  delay(100);
  
  if(angle>=0 && angle<=30) { //red detected
    //statusPacket.params[0] = 1;
    //Serial.println("Red");
  } else if (angle>=90 && angle<=150){ //green detected
    //statusPacket.params[0] = 2;  
    //Serial.println("Green");
  } else { //others
    //Serial.println("Others"); 
  }*/

  //sending data packet to the rpi
  /*TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_COLOUR;
  
  if(angle>=0 && angle<=30) { //red detected
    statusPacket.params[0] = 1;
  } else if (angle>=90 && angle<=150){ //green detected
    statusPacket.params[0] = 2;  
  } else { //others
    statusPacket.params[0] = 0;  
  }
  //statusPacket.params[0] = redColour;
  //statusPacket.params[1] = greenColour;
  //statusPacket.params[2] = blueColour;
  sendResponse(&statusPacket);*/
}

/*Calculates the color hue of detected color*/
float calchue(float red, float green, float blue) {
  float color_array[3]= {red, green, blue};
  float R = color_array[0] / 255;
  float G = color_array[1] / 255;
  float B = color_array[2] / 255;
  float max = R;
  float min = R;
  for (int i = 0; i <= 2; i += 1) {
    if (max < color_array[i] / 255) {
      max = color_array[i] / 255;
    }
    if (min > color_array[i] / 255) {
      min = color_array[i] / 255;
    }
  }
  /*Returns calculated color hue according to formula in section x of report*/
  if (max == R) {
    return (G - B) / (max - min); 
  }
  if (max == G) {
    return 2.0 + (B - R) / (max - min);
  }
  if (max == B) {
    return 4.0 + (R - G) / (max - min);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  sendColour();

}