#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>  // For angle (radians)
#include <std_msgs/msg/int32.h>    // For motor command
#include <rmw/qos_profiles.h>      // Include for QoS profiles
#include <rcl/types.h>        // For rcl_subscription_options_t

#define rightEncoderPinA 22
#define rightEncoderPinB 23
#define leftEncoderPinA 32
#define leftEncoderPinB 33




// Motor class
class Motor {
private:
    int ENA;
    int IN1;
    int IN2;

public:
    Motor(int ena, int in1, int in2) : ENA(ena), IN1(in1), IN2(in2) {}
    
    void init() {
        pinMode(ENA, OUTPUT);
        pinMode(IN1, OUTPUT);
        pinMode(IN2, OUTPUT);
    }
    
    void setDirection(bool forward) {
        if (forward) {
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
        } else {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
        }
    }
    
    void setVoltage(int cmd_voltage) {
        analogWrite(ENA, constrain(cmd_voltage, 0, 255));
    }
};

// Encoder class
class Encoder {
private:
    int pin_A;
    int pin_B;
    volatile int position;
    int revolusion;
    unsigned long lastTime;
    double currentAngle;
    float angularSpeed;
    double lastAngle;

public:
Encoder(int Pin_A, int Pin_B) : pin_A(Pin_A), pin_B(Pin_B), position(0), revolusion(0), lastTime(0), angularSpeed(0), currentAngle(0), lastAngle(0) {}

    void init() {
        pinMode(pin_A, INPUT_PULLUP);
        pinMode(pin_B, INPUT_PULLUP);
    }

    static void IRAM_ATTR readEncoder(Encoder* encoder) {
        if (digitalRead(encoder->pin_A) == digitalRead(encoder->pin_B)) {
            encoder->position++;
        } else {
            encoder->position--;
        }
        if (encoder->position >= 982.8) {
            encoder->revolusion++;
            encoder->position -= 982.8;
        } else if (encoder->position <= -982.8) {
            encoder->revolusion--;
            encoder->position += 982.8;
        }
    }

    float getRadian() {
        return (revolusion + position / 982.8f) * 2 * PI;
    }

    float getSpeed() {
        unsigned long currentTime = millis();
        float deltaTime = (currentTime - lastTime) / 1000.0f;
        if (deltaTime <= 0.0001f) return angularSpeed;
        currentAngle = getRadian();
        angularSpeed = (currentAngle - lastAngle) / deltaTime;
        lastAngle = currentAngle;
        lastTime = currentTime;
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
        float alpha = 0.7;  // Adjust between 0-1 for filtering
    
        Motor& motor; // Motor and Sensor References
        Encoder& encoder;
    
    public:
        // Constructor to initialize Motor and Sensor references
        Controller(Motor& motor, Encoder& encoder) : motor(motor), encoder(encoder) {}
        float desiredSpeed = 0;
        
        void PID() {
           float rawSpeed = encoder.getSpeed();
           //filteredSpeed = lowPassFilter(rawSpeed);
            // PID calculations
            error = desiredSpeed - rawSpeed;
            //error = desiredSpeed - filteredSpeed;
            integral += error;
            integral = constrain(integral, -500*180/PI, 50*180/PI); // Prevent excessive accumulation
            derivative = error - lastError;
    
            pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);
            
            pidOutput = constrain(pidOutput, -255, 255);  // Keep PWM in valid range
            setDirection();
            motor.setVoltage(abs(pidOutput));  // Apply PID output to motor
            lastError = error;
        }
    
        void setDirection(){
            if (pidOutput>0){
              motor.setDirection(true); //Forward
            }
            else if(pidOutput<0){
              motor.setDirection(false); //Reverse
            }
        }
        
        float lowPassFilter(float rawspeed){
            filteredSpeed = (alpha * rawspeed) + ((1 - alpha) * filteredSpeed);  // Low-pass filter
            return filteredSpeed;
        }
        float getFilteredSpeed(){return filteredSpeed;}
};


// ROS2 objects
rcl_publisher_t right_angle_publisher;
std_msgs__msg__Float32 right_angle_msg;
rcl_publisher_t left_angle_publisher;
std_msgs__msg__Float32 left_angle_msg;

// speed publisher
rcl_publisher_t right_vel_publisher;
std_msgs__msg__Float32 right_vel_msg;
rcl_publisher_t left_vel_publisher;  // New publisher for speed
std_msgs__msg__Float32 left_vel_msg;  // New message for speed

rcl_subscription_t r_cmd_subscriber;
std_msgs__msg__Int32 r_cmd_msg;
rcl_subscription_t cmd_subscriber;
std_msgs__msg__Int32 cmd_msg;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Instances
Motor rightMotor(5,18,19);
Encoder rightEncoder(rightEncoderPinA, rightEncoderPinB);
Controller rightMotorController(rightMotor, rightEncoder);
Motor leftMotor(13, 26, 27);
Encoder leftEncoder(leftEncoderPinA, leftEncoderPinB);
Controller leftMotorController(leftMotor, leftEncoder);
// FreeRTOS task handle for PID
TaskHandle_t pidTaskHandle;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
    while (1) delay(100);
}

// PID task function
void pidTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);  // Run PID every 10 ms

    for (;;) {
        leftMotorController.PID();  // Run PID in the background
        vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Precise timing
    }
}

void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        // Right wheel
        right_angle_msg.data = rightEncoder.getRadian();
        RCSOFTCHECK(rcl_publish(&right_angle_publisher, &right_angle_msg, NULL));
        right_vel_msg.data = rightEncoder.getSpeed();
        RCSOFTCHECK(rcl_publish(&right_vel_publisher, &right_vel_msg, NULL));

        // Left Wheel
        left_angle_msg.data = leftEncoder.getRadian();
        RCSOFTCHECK(rcl_publish(&left_angle_publisher, &left_angle_msg, NULL));
        left_vel_msg.data = leftEncoder.getSpeed();
        RCSOFTCHECK(rcl_publish(&left_vel_publisher, &left_vel_msg, NULL));
    }
}

void r_subscription_callback(const void* msgin) {
    const std_msgs__msg__Int32* msg = (const std_msgs__msg__Int32*)msgin;
    float r_cmd_vel = msg->data;
    rightMotorController.desiredSpeed = r_cmd_vel;
  }
  
void l_subscription_callback(const void* msgin) {
  const std_msgs__msg__Int32* msg = (const std_msgs__msg__Int32*)msgin;
  float l_cmd_vel = msg->data;
  leftMotorController.desiredSpeed = l_cmd_vel;
}

void setup() {
    Serial.begin(921600);
    Serial.flush();  // Clear any buffer
    set_microros_serial_transports(Serial);
    delay(200);

    rightMotor.init();
    rightEncoder.init();
    attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), []() { Encoder::readEncoder(&rightEncoder); }, CHANGE);

    leftMotor.init();
    leftEncoder.init();
    attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), []() { Encoder::readEncoder(&leftEncoder); }, CHANGE);
    // Create PID task with lower priority
    xTaskCreate(
        pidTask,           // Task function
        "PID Task",        // Task name
        2048,              // Stack size (bytes)
        NULL,              // Parameter
        3,                 // Priority (lower than main loop)
        &pidTaskHandle     // Task handle
    );

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_motor_node", "", &support));

    // Initialize publisher
    RCCHECK(rclc_publisher_init_default(
        &right_angle_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "right_angle"));
    
    RCCHECK(rclc_publisher_init_default(
        &right_vel_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "right_vel"));

    RCCHECK(rclc_publisher_init_default(
        &left_angle_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "left_angle"));

    RCCHECK(rclc_publisher_init_default(
        &left_vel_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "left_vel"));

    // Define custom QoS profile for low latency
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    qos_profile.depth = 1;  // Small queue size to reduce buffering
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;  // Best effort for low latency
    qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;  // Keep only the latest message

    // Initialize subscription options with the QoS profile
    rcl_subscription_options_t subscription_options = rcl_subscription_get_default_options();
    subscription_options.qos = qos_profile;

    // Initialize subscriber with custom QoS
    RCCHECK(rcl_subscription_init(
        &r_cmd_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "r_cmd_vel",
        &subscription_options));

    RCCHECK(rcl_subscription_init(
        &cmd_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "l_cmd_vel",
        &subscription_options));

    // Initialize timer
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(100),
        timer_callback));

    // Initialize executor and add components
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &r_cmd_subscriber,
        &r_cmd_msg,
        &r_subscription_callback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &cmd_subscriber,
        &cmd_msg,
        &l_subscription_callback,
        ON_NEW_DATA));
}

void loop() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(12)));
}
