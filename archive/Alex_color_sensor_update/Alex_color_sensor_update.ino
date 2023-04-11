#define s0 A0       
#define s1 A1
#define out A2
#define s2 A3
#define s3 A4
#define trig 12 
#define echo 13

int redColor =0;        
int blueColor =0; 
int greenColor =0; 
int color;
float distance;
float soundSpeed = 0.0345;

// Red 1
// Green 2
// White 3

//Green 104 79 86, 115 86 95
//Red 60 74 111, 71 88 121

void setup() 
{ 
   pinMode(s0,OUTPUT);   
   pinMode(s1,OUTPUT);
   pinMode(s2,OUTPUT);
   pinMode(s3,OUTPUT);
   pinMode(out,INPUT);
   pinMode(trig, OUTPUT);
   pinMode(echo, INPUT);
   Serial.begin(9600);   
   digitalWrite(s0,HIGH); //Putting S0/S1 on HIGH/HIGH levels means the output frequency scalling is at 100% (recommended)
   digitalWrite(s1,HIGH);    
}

void loop()                  //Every 0.2s we select a photodiodes set and read its data
{
   distance_check();
   color_check();
}

void color_check(){
digitalWrite(s2,LOW);        //S2/S3 levels define which set of photodiodes we are using LOW/LOW is for RED LOW/HIGH is for Blue and HIGH/HIGH is for green
   digitalWrite(s3,LOW);
   Serial.print("Red value= "); 
   redColor =pulseIn(out,LOW);  //here we wait until "out" go LOW, we start measuring the duration      and stops when "out" is HIGH again
   Serial.print(redColor);     
   Serial.print("\t");          
   delay(20);
                      
   digitalWrite(s2,LOW);
   digitalWrite(s3,HIGH);
   Serial.print("Blue value= ");
   blueColor=pulseIn(out,LOW);  //here we wait until "out" go LOW, we start measuring the duration and stops when "out" is HIGH again
   Serial.print(blueColor);       
   Serial.print("\t");          
   delay(20);

   digitalWrite(s2,HIGH);
   digitalWrite(s3,HIGH);
   Serial.print("Green value= ");
   greenColor = pulseIn(out,LOW);  //here we wait until "out" go LOW, we start measuring the duration and stops when "out" is HIGH again
   Serial.print(greenColor);    
   Serial.print("\t");          
   delay(20);
   
   if(redColor > 57 && redColor < 73 &&  greenColor > 109 && greenColor < 126 ){
    color = 1;
   } else if(redColor > 91 && redColor < 109  && greenColor > 77 && greenColor < 95){
    color = 2;
   } else{
    color = 0; 
   }
   Serial.print("Color = "); 
   Serial.print(color);
   Serial.println();
   delay(200);  
}

void distance_check(){
   digitalWrite(trig, HIGH);
   delayMicroseconds(10);
   digitalWrite(trig, LOW);
   delayMicroseconds(2);
   distance = pulseIn(echo, HIGH);
   distance = distance*soundSpeed/2;
   Serial.print("Distance = "); 
   Serial.print((int)distance);
   Serial.println();
}
