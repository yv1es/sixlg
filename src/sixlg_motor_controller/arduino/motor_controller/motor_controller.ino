#include <Servo.h>


const int servoCount = 18;
const int firstServoPin = 22;

int servoAngles[servoCount];
Servo servos[servoCount];

const int numChars = servoCount * 4 + 2; 
char receivedChars[numChars];
boolean newData = false;

void setup()
{
  Serial.begin(115200);

  for (int i = 0; i < servoCount; i++)
  {
    Servo s;
    s.write(90); 
    s.attach(firstServoPin + i); 
    servos[i] = s;
  }
}

void loop()
{
  recvWithStartEndMarkers();
  if (newData == true)
  {
    parseData();
    for (int i = 0; i < servoCount; i++) {
      servos[i].write(servoAngles[i]); 
    }
    newData = false;
  }
}

void recvWithStartEndMarkers()
{
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false)
  {
    rc = Serial.read();

    if (recvInProgress == true)
    {
      if (rc != endMarker)
      {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars)
        {
          ndx = numChars - 1;
        }
      }
      else
      {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker)
    {
      recvInProgress = true;
    }
  }
}

void parseData()
{ 
  char *token;
  int i = 0; 

  token = strtok(receivedChars, ",");
  servoAngles[i++] = atoi(token); 

  while (token != NULL)
  {
    token = strtok(NULL, ",");
    servoAngles[i++] = atoi(token); 
  }
}
