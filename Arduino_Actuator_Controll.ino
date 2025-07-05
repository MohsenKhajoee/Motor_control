#define leftEncoderPinA 2
#define leftEncoderPinB 3

class Motor {
 private:
    int ENA;
    int IN1;
    int IN2;

 public:
    //Constructor
    Motor(int ena, int in1, int in2) {
        this->ENA = ena;
        this->IN1 = in1;
        this->IN2 = in2;
    }
    // Initialization function
    void init() {
        pinMode(ENA, OUTPUT);
        pinMode(IN1, OUTPUT);
        pinMode(IN2, OUTPUT);
    }
    
     // Set motor direction (1 = forward, 0 = backward)
    void setDirection(bool forward) {
        if (forward) {
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
        } else {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
        }
    }
    
     // Set motor speed (0-255 PWM)
    void setVoltage(int cmd_voltage) {
        analogWrite(ENA, constrain(cmd_voltage, 0, 255));
    }

};

class Sensor {
private:
    int pin_A;
    int pin_B;
    volatile int position;
    int positionCopy;
    int lastPosition;
    int revolusion, lastRevolusion;
    float angle, lastAngle;
    unsigned long currentTime;
    unsigned long lastTime;
    float deltaTime;
    float angularSpeed;
    int speedPinVal;
    int counter;

public:
    // Constructor
    Sensor(int Pin_A, int Pin_B) {
        this->pin_A = Pin_A;
        this->pin_B = Pin_B;
        position = 0;
        lastPosition = 0;
        revolusion = lastRevolusion = 0;
        angle = lastAngle = 0;
        lastTime = 0;
        deltaTime = 0;
        angularSpeed = 0;
        speedPinVal = 0;
        counter = 0;
    }

    // Initialization function
    void init() {     
        pinMode(pin_A, INPUT_PULLUP);
        pinMode(pin_B, INPUT_PULLUP);
    }


    // Interrupt service for encoder reading
    static void readEncoder(Sensor* encoder) {
        if (digitalRead(encoder->pin_A) == digitalRead(encoder->pin_B)) {
            encoder->position++;  // Clockwise rotation
        } else {
            encoder->position--;  // Counterclockwise rotation
        } 
        //convert to revolusions so avoid overflow of position variable
        if (encoder->position >= 982.8){
            encoder->revolusion++;
            encoder->position -= 982.8;
        }
        if (encoder->position <= -982.8){
            encoder->revolusion--;
            encoder->position += 982.8;
        }
        
    }

    
    int getRevolusion() {
        return revolusion;
    }

    
    int getRadian() {
        return (revolusion + position / 982.8)*2*PI;
    }

    float getSpeed(){
        currentTime = millis();
        deltaTime = (currentTime - lastTime)/ 1000.0;  // milliseconds
        if (deltaTime <= 0.001) {  // Prevent division by zero or very small dt
        return angularSpeed;  // Keep the last speed to avoid spikes
    }
        //angle = position / 2.73; // Convert encoder ticks to degrees
        angularSpeed = ( (revolusion + position / 982.8f) - (lastRevolusion + lastPosition / 982.8f) )*2*PI / deltaTime; // Radians per second
        
        lastTime = currentTime;
        lastPosition = position;
        lastRevolusion = revolusion;
        
        return angularSpeed;  
    }
};


class Controller {
private:
    // PID Constants
    float Kp = 0.7*180/PI;   // Proportional gain
    float Ki = 0.1*180/PI; // Integral gain
    float Kd = 0.07*180/PI; // Derivative gain

    float error = 0;
    float lastError = 0;   // Previous error
    float integral = 0;    // Integral term
    float derivative = 0;
    float pidOutput = 0;
    float filteredSpeed = 0;
    float alpha = 0.6;  // Adjust between 0-1 for filtering

    Motor& motor; // Motor and Sensor References
    Sensor& encoder;

public:
    // Constructor to initialize Motor and Sensor references
    Controller(Motor& motor, Sensor& encoder) : motor(motor), encoder(encoder) {}
    float desiredSpeed = 3;
    
    void PID() {
       float rawSpeed = encoder.getSpeed();
       filteredSpeed = lowPassFilter(rawSpeed);
        // PID calculations
        error = desiredSpeed - filteredSpeed;
        integral += error;
        integral = constrain(integral, -500*180/PI, 50*180/PI); // Prevent excessive accumulation
        derivative = error - lastError;

        pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);

        setDirection();
        pidOutput = constrain(pidOutput, 0, 255);  // Keep PWM in valid range
        lastError = error;
        
        motor.setVoltage(pidOutput);  // Apply PID output to motor
    }

    void setDirection(){
        if (pidOutput>0){
          motor.setDirection(true);
        }
        else if(pidOutput<0){
          motor.setDirection(false);
          pidOutput = abs(pidOutput);
        }
    }
    
    float lowPassFilter(float rawspeed){
        filteredSpeed = (alpha * rawspeed) + ((1 - alpha) * filteredSpeed);  // Low-pass filter
        return filteredSpeed;
    }
    float getFilteredSpeed(){return filteredSpeed;}
};


Motor leftMotor(9,7,8);
Sensor leftEncoder(leftEncoderPinA,leftEncoderPinB);
Controller leftMotorController(leftMotor, leftEncoder);
int counter = 0;
float targetSpeed = 0;

void setup() {
    leftMotor.init();
    leftMotor.setDirection(true);
    leftEncoder.init();
    attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), []() { Sensor::readEncoder(&leftEncoder); }, CHANGE);
    Serial.begin(9600);
}



void loop() {
    readSerial();
    leftMotorController.PID();
    //rightMotorController.PID();
    //if (counter%70 == 0){Serial.println(leftMotorController.getFilteredSpeed());}
    counter++;
    delay(3);
}

void readSerial(){
    if (Serial.available()) { 
        String input = Serial.readStringUntil('\n');  // Read full input until newline
        input.trim();  // Remove any trailing spaces or newlines

        if (input.length() > 1) {  // Ensure input is valid
            char direction = input[0];  // First character should be 'L' or 'R'
            targetSpeed = input.substring(1).toFloat();  // Convert remaining part to float
            

            if (direction == 'L') {
                leftMotorController.desiredSpeed = targetSpeed;
//                Serial.print("Targer Speed: ");
//                Serial.println(leftMotorController.desiredSpeed);
            } else if (direction == 'R') {
                //rightMotorController.desiredSpeed = targetSpeed;
            }

        }
    }
}
