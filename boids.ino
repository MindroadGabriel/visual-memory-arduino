#include <Vector_datatype.h>
#include <quaternion_type.h>
#include <vector_type.h>

#include <Bounce2.h>

#include <Adafruit_SSD1306.h>
#include <splash.h>

#include "FastIMU.h"
#include <Wire.h>

#define IMU_ADDRESS 0x69    //Change to the address of the IMU
//#define PERFORM_CALIBRATION //Comment to disable startup calibration
BMI160 IMU;               //Change to the name of any supported IMU! 

// Currently supported IMUS: MPU9255 MPU9250 MPU6886 MPU6500 MPU6050 ICM20689 ICM20690 BMI055 BMX055 BMI160 LSM6DS3 LSM6DSL QMI8658

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;
MagData magData;

Bounce button1 = Bounce();
Bounce button2 = Bounce();

float accX = 0.0f;
float accY = 0.0f;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define CIRCLE_RADIUS 5

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pq  §in # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16

class Boid {
public:
    vec3_t position;
    static float velocity;
    float x;
    float y;
    float angle;
    Boid() {
      this->x = 0.0f;
      this->x = 0.0f;
      this->angle = 2.0f;
    }
    void randomize() {
        this->x = random(0, SCREEN_WIDTH - 1);
        this->y = random(0, SCREEN_HEIGHT - 1);
        this->angle = random(0, 2*PI);
    }
    void update(float deltaTime) {
        this->x += cos(this->angle) * velocity * deltaTime;
        this->y += sin(this->angle) * velocity * deltaTime;
        while (this->x < 0.0f) {
            this->x += SCREEN_WIDTH;
        }
        while (this->y < 0.0f) {
            this->y += SCREEN_HEIGHT;
        }
        while (this->x > SCREEN_WIDTH) {
            this->x -= SCREEN_WIDTH;
        }
        while (this->y > SCREEN_HEIGHT) {
            this->y -= SCREEN_HEIGHT;
        }
    }
    void render() {
        display.drawPixel(this->x, this->y, WHITE);
    }
};
float Boid::velocity = 50.0;
void writeString(char* text) {
  char* c = text;
  while(*c) {
    display.write(*c);
    c++;
  }
}


void writeFloatString(float n) {
  char buffer[128];
  dtostrf(n, 4, 2, buffer);
  int len = strlen(buffer);
  Serial.print("Len:");
  int desired_length = 6;
  for (int i = 0; i < len - desired_length; ++i) {
    writeString(" ");
  }
  writeString(buffer);
}
void setup() {
  Wire.begin();
  Wire.setClock(10000); //400khz clock
  Serial.begin(9600);
  while (!Serial) {
    ;
  }

    pinMode(5,INPUT_PULLUP);
    button1.attach(5);
    button1.interval(10);// interval in ms
    pinMode(4,INPUT_PULLUP);
    button2.attach(4);
    button2.interval(10); // interval in ms

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }// */
  display.clearDisplay();
  display.display();
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.display();

  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  
#ifdef PERFORM_CALIBRATION
  Serial.println("FastIMU calibration & data example");
  if (IMU.hasMagnetometer()) {
    delay(1000);
    Serial.println("Move IMU in figure 8 pattern until done.");
    delay(3000);
    IMU.calibrateMag(&calib);
    Serial.println("Magnetic calibration done!");
  }
  else {
    delay(5000);
  }

  delay(5000);
  Serial.println("Keep IMU level.");
  delay(5000);
  IMU.calibrateAccelGyro(&calib);
  delay(5000);
  IMU.init(calib, IMU_ADDRESS);
#endif

  //err = IMU.setGyroRange(500);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
  //err = IMU.setAccelRange(2);       //THESE TWO SET THE GYRO RANGE TO ±500 DPS AND THE ACCELEROMETER RANGE TO ±2g

  if (err != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
}

#define NUM_BOIDS 30
Boid boids[NUM_BOIDS];
void loop() {
    for (int i = 0; i < NUM_BOIDS; ++i) {
        boids[i].randomize();
        Serial.print("Creating boid with x: ");
        Serial.print(boids[i].x);
        Serial.print(", y: ");
        Serial.print(boids[i].y);
        Serial.print(", angle: ");
        Serial.print(boids[i].angle);
        Serial.print("\n");
    }
    while (true) {
        for (auto boid : boids) {
            // Separation
            static float separation_dist_squared = 5 * 5;
            float separation_x = 0.0f;
            float separation_y = 0.0f;
            uint8_t separation_count = 0;
            for (auto other : boids) {
//                float x_distance = (boid->x - other->x);
//                float y_distance = (boid->y - other->y);
//                float distance_squared = x_distance * x_distance + y_distance * y_distance;
//                if (distance_squared > 0 && distance_squared < separation_dist_squared) {
//                    separation_x += distance_squared;
//                    separation_count += 1;
//                }

            }
            // Aligment
            // Cohesion
        }
        button1.update();
        button2.update();
        if (button1.fell()) {
            display.clearDisplay();
            display.setCursor(0, 0);
            writeString("Calibrating.\nKeep level.");
            display.display();
            delay(1000);

            IMU.calibrateAccelGyro(&calib);
            IMU.init(calib, IMU_ADDRESS);

            display.clearDisplay();
            display.setCursor(0, 0);
            writeString("Calibration done!");
            display.display();
            delay(800);
        }

        IMU.update();
        IMU.getAccel(&accelData);
        display.clearDisplay();
        display.setCursor(0, 0);

        accX += accelData.accelX;
        accY += accelData.accelY;
        float factor = 2.5;
        int16_t x = CIRCLE_RADIUS + ((accelData.accelY*factor + 1) / 2) * (SCREEN_WIDTH - CIRCLE_RADIUS);
        int16_t y = CIRCLE_RADIUS + ((accelData.accelX*factor + 1) / 2) * (SCREEN_HEIGHT - CIRCLE_RADIUS);
//  writeFloatString(accelData.accelX);
//  writeString(", ");
//  writeFloatString(accelData.accelY);
//  writeString(", ");
//  writeFloatString((float)x);
//  writeString(", ");
//  writeFloatString((float)y);
        display.drawCircle(x, y, CIRCLE_RADIUS, SSD1306_WHITE);
        //writeString("\nTEMPERATURE = ");
        //writeString("\nTEMPERATURE = ");
        //writeFloatString(temperature);
        //writeString(", HUMIDITY = ");
        //writeFloatString(humidity);
        //writeString("\n");
        for (int i = 0; i < NUM_BOIDS; ++i) {
            boids[i].update(1.0f/50.0f);
        }
        for (int i = 0; i < NUM_BOIDS; ++i) {
            boids[i].render();
        }
        display.display();
        delay(50);
    }
}