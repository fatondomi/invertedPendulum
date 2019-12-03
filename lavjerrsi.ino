
// Pinat e butonave
#define mBtnPin 3
#define sBtnPin A1
#define minusBtnPin A2
#define plusBtnPin A3
#define xBtnPin A4

// Pinat e motor driverit
#define ccwDrivePin 10
#define cwDrivePin 11
#define pwmDrivePin 9

// Pinat enkoderit te motorit
#define pulseAPin 4
#define pulseBPin 2

// Pini i enkoderit analog
#define analogAnglePin A5

/* graphs for finding direction and steps
byte graphState[2][2] = {{2,1},
			             {3,0}};

signed char graphStep[4][4] = {{ 0, 1, 0,-1},
			                   {-1, 0, 1, 0},
			                   { 0,-1, 0, 1},
			                   { 1, 0,-1, 0}};
*/

/* encoder variables
byte prevState;
byte currentState;
signed char stepCounter = 0;
float rpm = 0;
*/

/* measurment vars
*/
int currentMeasurement = 0;
int prevMeasurement = 0;
int measurements[4] = {0,0,0,0};
byte mIndex = 0;

String textReceived = "";

enum workModes {play, drive, calibrate, paused} mode;

float kp = 5,
      ki = 0.12,
      kd = 14;

byte fromHere = 0;
byte toHere = 0;

float currentE = 0, prevE = 0;

float derivative = 0,
      integral = 0,
      setPoint = 771.8,
      pidResult = 0;

double timeRef;

void setup()
{
    Serial.begin(115200);
    // Pinat e butonave
    pinMode(mBtnPin,INPUT);
    pinMode(sBtnPin,INPUT);
    pinMode(minusBtnPin,INPUT);
    pinMode(plusBtnPin,INPUT);
    pinMode(xBtnPin,INPUT);
    // Pinat e driverit te motorit
    pinMode(ccwDrivePin,OUTPUT);
    pinMode(cwDrivePin,OUTPUT);
    pinMode(pwmDrivePin,OUTPUT);
    // Pinat e enkoderit te motorit
    pinMode(pulseAPin,INPUT);
    pinMode(pulseBPin,INPUT);
    // Pini i potenciometrit
    pinMode(analogAnglePin,INPUT);
    //
    mode = paused;
}

void loop()
{
    if(Serial.available())
    {
        while(Serial.available())
        {
            textReceived += Serial.readString();
        }
        textReceived.trim();
        parseText();
    }
    if(mode == play)
    {
        pidDrive();
    }
    else if(mode == calibrate)
    {
        measurements[mIndex] = analogRead(analogAnglePin);
        mIndex = (mIndex > 2)? 0 : (mIndex + 1);
        Serial.println(((float)(measurements[0] + measurements[1] + measurements[2] + measurements[3])) / 4);
    }
}

void pidDrive()
{
    if(millis() > timeRef)
    {
        timeRef = millis() + 4;

        currentE = ((float)analogRead(analogAnglePin)) - setPoint;
        derivative = currentE - prevE;
        integral += currentE;

        pidResult = (kp * currentE) + (kd * derivative) + (ki * integral);
        pidResult = (pidResult >  255)?  255 : 
                    (pidResult < -255)? -255 : pidResult;

        if(currentE > -200 && currentE < 200)
        {// is angle between +- 45 degrees from top normal
            if(pidResult >= 0)
            {
                digitalWrite(ccwDrivePin, LOW);
                digitalWrite(cwDrivePin, HIGH);
                analogWrite(pwmDrivePin, pidResult);
            }
            else
            {
                digitalWrite(cwDrivePin, LOW);
                digitalWrite(ccwDrivePin, HIGH);
                analogWrite(pwmDrivePin, -pidResult);
            }
        }
        else
        {
            digitalWrite(cwDrivePin, LOW);
            digitalWrite(ccwDrivePin, LOW);
            analogWrite(pwmDrivePin, 0);
            mode = paused;
        }

        prevE = currentE;
        //Serial.println(currentE);
    }
}

void parseText()
{
    Serial.println(textReceived);

    if(textReceived.indexOf("stop()") == 0)
    {
        Serial.println("OK");
        
        analogWrite(pwmDrivePin, 0);
        mode = paused;
    }
    else if(textReceived.indexOf("drive(") == 0)
    {
        Serial.println("OK");

        fromHere = textReceived.indexOf("(") + 1;
        toHere = textReceived.indexOf(")");
        int result = textReceived.substring(fromHere, toHere).toInt();
        result = (result >  255)?  255 : result;
        result = (result < -255)? -255 : result;

        if(result >= 0)
        {
            digitalWrite(ccwDrivePin, LOW);
            digitalWrite(cwDrivePin, HIGH);
            analogWrite(pwmDrivePin, result);
        }
        else
        {
            digitalWrite(cwDrivePin, LOW);
            digitalWrite(ccwDrivePin, HIGH);
            analogWrite(pwmDrivePin, -result);
        }

        mode = drive;
    }
    else if(textReceived.indexOf("pidDrive()") == 0)
    {
        Serial.println("OK");

        currentE = ((float)analogRead(analogAnglePin)) - setPoint;
        prevE = currentE;
        integral = 0;

        mode = play;
        timeRef = millis() + 4;
    }
    else if(textReceived.indexOf("calibrate()") == 0)
    {
        Serial.println("OK");
        mIndex = 0;
        mode = calibrate;
    }
    else if(textReceived.indexOf("getPid()") == 0)
    {
        Serial.println("OK");
        
        Serial.print("Kp = ");
        Serial.print(kp);
        Serial.print(" Ki = ");
        Serial.print(ki);
        Serial.print(" Kd = ");
        Serial.println(kd);
    }
    else if(textReceived.indexOf("setPid(") == 0)
    {
        Serial.println("OK");
        
        fromHere = textReceived.indexOf("(") + 1;
        toHere = textReceived.indexOf(",");

        kp = textReceived.substring(fromHere, toHere).toFloat();

        fromHere = toHere + 1;
        toHere = textReceived.indexOf(",", fromHere);

        ki = textReceived.substring(fromHere,toHere).toFloat();

        fromHere = toHere + 1;
        toHere = textReceived.indexOf(")");

        kd = textReceived.substring(fromHere,toHere).toFloat();

        Serial.print("Kp = ");
        Serial.print(kp);
        Serial.print(" Ki = ");
        Serial.print(ki);
        Serial.print(" Kd = ");
        Serial.println(kd);
    }
    else if(textReceived.indexOf("getSetpoint()") == 0)
    {
        Serial.println("OK");
        Serial.print("setPoint = ");
        Serial.println(setPoint);
    }
    else if(textReceived.indexOf("setSetpoint(") == 0)
    {
        Serial.println("OK");
        fromHere = textReceived.indexOf("(") + 1;
        toHere = textReceived.indexOf(")");
        setPoint = textReceived.substring(fromHere, toHere).toFloat();
        Serial.print("setPoint = ");
        Serial.println(setPoint);
    }
    else
    {
        Serial.println("No internal function matches command");
    }
    
    textReceived = "";
}

/*

void encoderCounting()
{
    prevState = currentState;
    currentState = graphState[digitalRead(pulseAPin)][digitalRead(pulseBPin)];
    stepCounter += graphStep[prevState][currentState];
}

void measureAngle()
{
    measurements[mIndex] = (float)analogRead(analogAnglePin)/10;
    mIndex = (mIndex > 3)? 0 : (mIndex + 1);
    Serial.println((float)((measurements[0] + measurements[1] + measurements[2] + measurements[3]) / 4));
}


void btnDriving()
{
    if(!digitalRead(sBtn))
    {
        digitalWrite(cwDrive,HIGH);
    }
    else
    {
        digitalWrite(cwDrive,LOW);

        if(!digitalRead(xBtn))
        {
            digitalWrite(ccwDrive,HIGH);
        }
        else
        {
            digitalWrite(ccwDrive,LOW);
        }
    }
}



//set ppd code textReceived parsing


        Serial.println("OK");
        
        fromHere = textReceived.indexOf("(") + 1;
        toHere = textReceived.indexOf(",");

        kpa = textReceived.substring(fromHere, toHere).toFloat();

        fromHere = toHere + 1;
        toHere = textReceived.indexOf(",", fromHere);

        kpv = textReceived.substring(fromHere,toHere).toFloat();

        fromHere = toHere + 1;
        toHere = textReceived.indexOf(")");

        kd = textReceived.substring(fromHere,toHere).toFloat();

        Serial.print("Kpa = ");
        Serial.print(kpa);
        Serial.print(" Kpv = ");
        Serial.print(kpv);
        Serial.print(" Kd = ");
        Serial.println(kd);
*/
