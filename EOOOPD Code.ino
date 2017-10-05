const int TRIG1 = 1; //first trigger sensor (second will be TRIG1 + 1, third TRIG+2, etc)
const int SENSOR1 = 5; //first input from the sensor
const int L_VIBRATION = 10; //vibration motor pin on the left
const int R_VIBRATION = 11; //vibration motor pin on the right
const int C_VIBRATION = 9; //vibration motor pin on the center
const int NUM_SENS = 4; //how many sensors
const int TO_CM = 58;  //convert pulse length to CM distance
const int TO_IN = 148;  //convert pulse length to IN distance
const int DELAY_TRIES = 10;  //how much to delay between tries on the same sensor
const int DELAY_SENSORS = 10; //how much to delay between two sensors
const int NUM_TRIES = 1; //how many tries to do each sensor for
const int ACTIVATE_BTN = 12; //button to start vibrations
const int MAX_DIST = 150; //maximum distance read in cm
const int MIN_DIST = 50; //minimum distance read in cm
const int MAX_SENSOR_CHANGE = 23200; //maximum value the sensor can read
const int MAX_DIFF_SENS = 250; //sensors should not read beyond a difference of this
const int MAX_SENS_WRITE = 255; //maximum amount sent to the motor
const double C_MOTOR_COEFF = 1; //how much above 255 the readings can be mapped to. leftover goes to center motor
const boolean DO_OUTPUT = false;

//last readings to calculate the difference between this reading and the last readings
unsigned long lastDiffs[NUM_SENS] = {0, 0, 0, 0};
//new reading from each sensor
unsigned long aveDiffs[NUM_SENS] = {0, 0, 0, 0};
//last iteration button state
boolean lastBtn = LOW;

void setup()
{
  //set up the trigger pins as outputs
  for (int i = TRIG1; i < TRIG1 + NUM_SENS; i++)
  {
    pinMode(i, OUTPUT);
  }
  //set up the sensor pins as inputs
  for (int i = SENSOR1; i < SENSOR1 + NUM_SENS; i++)
  {
    pinMode(i, INPUT);
  }

  //begin serial communciations
  if(DO_OUTPUT)
    Serial.begin(9600);

  //vibration motors are outputs
  pinMode(L_VIBRATION, OUTPUT);
  pinMode(R_VIBRATION, OUTPUT);
  pinMode(C_VIBRATION, OUTPUT);
  
  //button is an input
  pinMode(ACTIVATE_BTN, INPUT);
}

void loop()
{
  //read the sensors
  readSensors();
  
  //see if button is pressed
  if (digitalRead(ACTIVATE_BTN))
  {
    //vibrate motors
    vibration();
    lastBtn = HIGH;
  }
  else
  {
    if (!lastBtn)
    {
      //button isn't pressed so turn off vibration
      analogWrite(L_VIBRATION , 0);
      analogWrite(R_VIBRATION, 0);
      analogWrite(C_VIBRATION, 0);
    }
    lastBtn = LOW;
  }
  
  //push back current data to last
  setToLast();
}

void readSensors()
{
  //for each sensor
  for (int i = 0; i < NUM_SENS; i++)
  {
    //lengths of each pulse for each try
    unsigned long diffs[NUM_TRIES];

    //for each try
    for (int t = 0; t < NUM_TRIES; t++)
    {
      //before the pulse and after the pulse
      unsigned long beforeTime, afterTime;

      //current pins of the sensors
      int currSens = SENSOR1 + i;
      int currTrig = TRIG1 + i;

      //send pulse to the sensor
      digitalWrite(currTrig, HIGH);
      delayMicroseconds(10);
      digitalWrite(currTrig, LOW);

      //wait for pulse back
      while (digitalRead(currSens) == 0);

      //calculate the length of time of the pulse
      beforeTime = micros();
      while (digitalRead(currSens) == 1);
      afterTime = micros();
      diffs[t] = afterTime - beforeTime;

      //wait to do the next try
      delay(DELAY_TRIES);

      //reread if it's unreasonable
      if (abs((long)diffs[t] - (long)lastDiffs[t]) > MAX_SENSOR_CHANGE)
      {
        if(DO_OUTPUT)
          Serial.println("Error in sensor readings. Using last.");
        diffs[t] = lastDiffs[t];
      }
    }

    //calculate the average of the data set and convert to in and cm
    aveDiffs[i] = calcAve(diffs);
    double in = aveDiffs[i] / (double)TO_IN, cm = aveDiffs[i] / (double)TO_CM;

    if (DO_OUTPUT)
    {
      //output information:
      //which sensor
      Serial.print("Sensor: ");
      Serial.println(i + 1);
      Serial.println("Tries: ");
      //all of the tries
      for (int t = 0; t < NUM_TRIES; t++)
      {
        Serial.print(diffs[t]);
        Serial.print('\t');
      }
      Serial.println();

      //average of the data and distance away
      Serial.print("Average (removing outliers): ");
      Serial.println(aveDiffs[i]);
      Serial.print("Ave IN: ");
      Serial.println(in);
      Serial.print("Ave CM: ");
      Serial.println(cm);
      Serial.print("Difference from last read: ");

      //change from the last set of data
      if ((long)aveDiffs[i] - (long)lastDiffs[i] >= 0)
      {
        Serial.print('+');
        Serial.println((long)aveDiffs[i] - (long)lastDiffs[i]);
      }
      else
      {
        Serial.print('-');
        Serial.println((long)lastDiffs[i] - (long)aveDiffs[i]);
      }
      Serial.println();
    }
    //time to wait between sensors
    delay(DELAY_SENSORS);
  }
}

//push back data currently in current to last
void setToLast()
{
  for (int i = 0; i < NUM_SENS; i++)
  {
    lastDiffs[i] = aveDiffs[i];
  }
}

//convert the average of two current datas at index and index + 1 to centimeter distance
//also make sure both data sensors are reasonable; if not just use one sensor's data.
double toCM(int index)
{
  long d1 = (long)aveDiffs[index];
  long d2 = (long)aveDiffs[index + 1];
  if(abs(d1 - d2) > MAX_DIFF_SENS)
  {
   double aveLast = (lastDiffs[index] + lastDiffs[index + 1] ) / 2.0; 
   
   if((abs(d1 - aveLast) > MAX_SENSOR_CHANGE) && abs(d1 - aveLast) < MAX_SENSOR_CHANGE)
     return d2;
   else if(abs(d2 - aveLast) > MAX_SENSOR_CHANGE && abs(d1 - aveLast) < MAX_SENSOR_CHANGE)
     return d1;
  }
  double ave = (d1 + d2) / 2.0;
  return ave / (double)TO_CM;
}

//calculate intensity of PWM vibration and send it to the motors
void vibration()
{
  //get distances
  double cmL = toCM(0);
  double cmR = toCM(2);
  
  //make sure it isn't too big or too small of a distance
  if(cmL < MIN_DIST)
    cmL = MIN_DIST;
  if(cmR < MIN_DIST)
    cmR = MIN_DIST;
  if(cmR > MAX_DIST)
    cmR = MAX_DIST;
  if(cmL > MAX_DIST)
    cmL = MAX_DIST;
   
  //map the distance to an intensity, linearly
  int lMotor = map(cmL, MAX_DIST, MIN_DIST, 0, MAX_SENS_WRITE * C_MOTOR_COEFF);
  int rMotor = map(cmR, MAX_DIST, MIN_DIST, 0, MAX_SENS_WRITE * C_MOTOR_COEFF);

  //set up center motor to take the overflow over 255
  int cMotor = 0;
  if (lMotor > MAX_SENS_WRITE)
  {
    cMotor += lMotor - MAX_SENS_WRITE;
    lMotor = MAX_SENS_WRITE;
  }
  if (rMotor > MAX_SENS_WRITE)
  {
    cMotor += rMotor - MAX_SENS_WRITE;
    rMotor = MAX_SENS_WRITE;
  }
  if (cMotor > MAX_SENS_WRITE)
    cMotor = MAX_SENS_WRITE;

  //write the motor intensities
  analogWrite(L_VIBRATION , lMotor);
  analogWrite(R_VIBRATION, rMotor);
  analogWrite(C_VIBRATION, cMotor);
  
  //output the intensities
  if(DO_OUTPUT)
  {
  Serial.print(lMotor);
  Serial.print('\t');
  Serial.print(rMotor);
  Serial.print('\t');
  Serial.println(cMotor);
  }
}

//calculates the average of the data set
unsigned long calcAve(unsigned long diffs[])
{
  unsigned long sum;
  sum = diffs[0];

  //find the greatest and least in the data set.
  //add all to get the sum.
  for (int t = 1; t < NUM_TRIES; t++)
  {
    sum += diffs[t];
  }

  //return average
  return sum / (double)(NUM_TRIES);
}
