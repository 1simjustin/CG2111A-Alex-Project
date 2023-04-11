#define s0 4
#define s1 7  
#define s2 8
#define s3 9
#define out 12

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
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(out, INPUT);

  //setting frequency scaling to 20%
  digitalWrite(s0, HIGH);
  digitalWrite(s1, LOW);
  Serial.begin(9600);
}

void sendColour() {
    digitalWrite(s2, LOW);
    digitalWrite(s3, LOW);
    //read the output frequency
    redFrequency = pulseIn(out, LOW);
    //remap the value of the red frequency from 0 to 255
    redColour = map(redFrequency, 400, 1500, 255, 0);

    //setting green photodiodes to be read
    digitalWrite(s2, HIGH);
    digitalWrite(s3, HIGH);
    greenFrequency = pulseIn(out, LOW);
    greenColour = map(greenFrequency, 415, 2400, 255, 0);

    //setting blue photodiodes to be read
    digitalWrite(s2, LOW);
    digitalWrite(s3, HIGH);
    blueFrequency = pulseIn(out, LOW);
    blueColour = map(blueFrequency, 200, 1260, 255, 0);

    float hue = calchue(redColour, greenColour, blueColour) * 60;
    Serial.println(hue);
    delay(1000);
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