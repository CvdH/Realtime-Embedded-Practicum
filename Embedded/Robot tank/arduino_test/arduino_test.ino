#define APLUS 36
#define AMIN 38
#define AEN 44

#define BPLUS 40
#define BMIN 42
#define BEN 46



void setup() 
{
  pinMode(APLUS, OUTPUT);
  pinMode(AMIN, OUTPUT);
  pinMode(AEN,OUTPUT);

  pinMode(BPLUS, OUTPUT);
  pinMode(BMIN, OUTPUT);
  pinMode(BEN,OUTPUT);
}

void loop() 
{
  digitalWrite(AEN, HIGH);
   digitalWrite(BEN, HIGH);
  
  digitalWrite(APLUS, HIGH);
  digitalWrite(AMIN, LOW);

  digitalWrite(BPLUS, LOW);
  digitalWrite(BMIN, HIGH);

}
