const int line_pin[5]={8,9,10,11,12};
const int near=7;
const int clp=6;
int line_val[5]={0};
int near_val=0;
int clp_val=0;
int i;

void setup() 
{
    Serial.begin(9600);
    for (i=0;i<5;i++)
    {
      pinMode(line_pin[i],INPUT);
    }
    pinMode(near,INPUT);
    pinMode(clp,INPUT);
    Serial.println("success");
}

void loop() 
{
  Serial.print("Infrared:");
    read_sensor();
    display_serial();
}

void display_serial()
{
  for (i=0;i<5;i++)
  {
    Serial.print(line_val[i]);
    Serial.print(" ");
  }
  Serial.print(near_val);
  Serial.print(" ");
  Serial.println(clp_val);
  delay(1000);
}

void read_sensor()
{
  for (i=0;i<5;i++)
  {
    line_val[i]=digitalRead(line_pin[i]);
  }
  near_val=digitalRead(near);
  clp_val=digitalRead(clp);
}
