#define s0 A0       
#define s1 A1
#define out A2
#define s2 A3
#define s3 A4

int redColor =0;        
int blueColor =0; 
int greenColor =0; 
int data = 0;
int color = 0;

// Red 1
// Green 2
// White 3

void setup() 
{
  
   pinMode(s0,OUTPUT);   
   pinMode(s1,OUTPUT);
   pinMode(s2,OUTPUT);
   pinMode(s3,OUTPUT);
   pinMode(out,INPUT);
   //Serial.begin(9600);   
   
   digitalWrite(s0,HIGH); //Putting S0/S1 on HIGH/HIGH levels means the output frequency scalling is at 100% (recommended)
   digitalWrite(s1,HIGH); 
   
}

void loop()                  //Every 0.2s we select a photodiodes set and read its data
{
   color_check();
}

void color_check(){
digitalWrite(s2,LOW);        //S2/S3 levels define which set of photodiodes we are using LOW/LOW is for RED LOW/HIGH is for Blue and HIGH/HIGH is for green
   digitalWrite(s3,LOW);
   redColor =pulseIn(out,LOW);  //here we wait until "out" go LOW, we start measuring the duration      and stops when "out" is HIGH again       
   delay(20);
                      
   digitalWrite(s2,LOW);
   digitalWrite(s3,HIGH);
   blueColor=pulseIn(out,LOW);  //here we wait until "out" go LOW, we start measuring the duration and stops when "out" is HIGH again       
   delay(20);

   digitalWrite(s2,HIGH);
   digitalWrite(s3,HIGH);
   greenColor = pulseIn(out,LOW);  //here we wait until "out" go LOW, we start measuring the duration and stops when "out" is HIGH again  
   delay(20);
   
   
   if(redColor > 29 && redColor < 63 && blueColor > 39 && blueColor < 70 && greenColor > 57 && greenColor < 98 ){
    color = 1;
   } else if(redColor > 47 && redColor < 77 && blueColor > 37 && blueColor < 67 && greenColor > 40 && greenColor < 70){
    color = 2;
   }else if(redColor > 21 && redColor < 60 && blueColor > 14 && blueColor < 60 && greenColor > 17 && greenColor < 60){
    color = 3;
   } else{
    color = 0; 
   }

   delay(200);  
}
