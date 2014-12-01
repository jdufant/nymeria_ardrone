const int pingPin = 11;
long duration;
int cm;
int tmp_cm;
uint8_t first, second, third, last;

void setup() {
  // initialize serial communication:
  Serial.begin(115200);
}

void loop()
{
  // establish variables for duration of the ping, 
  // and the distance result in inches and centimeters:

  // The PING))) is triggered intby a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delay(2);
  digitalWrite(pingPin, HIGH);
  delay(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);
  Serial.flush();
  if(cm != tmp_cm){
    String dist = String(cm);
    
   
    if(cm < 10)
      Serial.print('0');
    if(cm < 100)
      Serial.print('0');
  
    Serial.print(dist); // always print 3 characters
    Serial.print('x'); // print 'x'
  }
  tmp_cm = cm;
  delay(10);
}


int microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

