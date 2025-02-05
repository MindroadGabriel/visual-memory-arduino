#include <Vector.h>

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

float distance(vec3_t from, vec3_t to);
void limit(vec3_t &vec, float max_magnitude);

class Boid {
public:
    vec3_t position;
    vec3_t velocity;
    vec3_t acceleration;
    float r;
    float maxforce; // Maximum steering force
    float maxspeed; // Maximum speed
    Boid() {}
    void initialize(float x, float y) {
        float angle = random(0.0f, 2.0f*PI);
        velocity = vec3_t(cos(angle), sin(angle));
        position = vec3_t(x, y);
        r = 2.0f;
        maxspeed = 2.0f;
        maxforce = 0.03f;
    }
    void run(Vector<Boid> &boids, bool log) {
        flock(boids, log);
        update(log);
        borders();
        render();
    }
    void applyForce(vec3_t force) {
        acceleration += force;
    }
    void flock(Vector<Boid> &boids, bool log) {
        vec3_t sep = separate(boids);
        vec3_t ali = align(boids);
        vec3_t coh = cohesion(boids);
        sep *= 1.5;
        ali *= 1.0;
        coh *= 1.0;
        if (log) {
            Serial.print("flock: ");
            Serial.print(sep.x);
            Serial.print(":");
            Serial.print(sep.y);
            Serial.print(", ");
            Serial.print(ali.x);
            Serial.print(":");
            Serial.print(ali.y);
            Serial.print(", ");
            Serial.print(coh.x);
            Serial.print(":");
            Serial.print(coh.y);
            Serial.print("  ");
        }

        applyForce(sep);
        applyForce(ali);
        applyForce(coh);
    }
    void update(bool log) {
        if (log) {
            Serial.print("update: ");
            Serial.print(acceleration.x);
            Serial.print(":");
            Serial.print(acceleration.y);
            Serial.print(", ");
            Serial.print(velocity.x);
            Serial.print(":");
            Serial.print(velocity.y);
            Serial.print(", ");
            Serial.print(position.x);
            Serial.print(":");
            Serial.print(position.y);
            Serial.print("  ");
        }
        // Update velocity
        velocity += acceleration;
        // Limit speed
        limit(velocity, maxspeed);
        position += velocity;
        // Reset acceleration to 0 each cycle
        acceleration *= 0;
    }

    vec3_t seek(vec3_t target) {
        // A vector pointing from the position to the target
        vec3_t desired = target - position;
        // Scale to maximum speed
        desired.norm();
        desired *= maxspeed;

        vec3_t steer = desired - velocity;
        limit(steer, maxforce);
        return steer;
    }

    void render() {
        display.drawPixel(position.x, position.y, WHITE);
    }

    void borders() {
        float width = SCREEN_WIDTH;
        float height = SCREEN_HEIGHT;
        if (position.x < -r) position.x = width + r;
        if (position.y < -r) position.y = height + r;
        if (position.x > width + r) position.x = -r;
        if (position.y > height + r) position.y = -r;
    }

    // Separation
    // Method checks for nearby boids and steers away
    vec3_t separate(Vector<Boid> &boids) {
        float desiredseparation = 25.0f;
        vec3_t steer = vec3_t(0, 0, 0);
        int count = 0;
        for (Boid &other : boids) {
            float d = distance(position, other.position);
            if ((d > 0) && (d < desiredseparation)) {
                vec3_t diff = position - other.position;
                diff.norm();
                diff /= d;
                steer += diff;
                count++;
            }
        }
        if (count > 0) {
            steer /= (float)count;
        }

        if (steer.mag() > 0) {
            // Implement Reynolds: Steering = Desired - Velocity
            steer.norm();
            steer *= maxspeed;
            steer -= velocity;
            limit(steer, maxforce);
        }
        return steer;
    }

    // Alignment
    // For every nearby boid in the system, calculate the average velocity
    vec3_t align(Vector<Boid> &boids) {
        float neighbourdist = 50;
        vec3_t sum = vec3_t(0, 0);
        int count = 0;
        for (Boid &other : boids) {
            float d = distance(position, other.position);
            if ((d > 0) && (d < neighbourdist)) {
                sum += other.velocity;
                count++;
            }
        }
        if (count > 0) {
            sum /= (float)count;
            sum.norm();
            sum *= maxspeed;
            vec3_t steer = sum - velocity;
            limit(steer, maxforce);
            return steer;
        } else {
            return vec3_t(0, 0);
        }
    }

    // Cohesion
    // For the average position (i.e. center) of all nearby boids,
    // calculate steering vector towards that position
    vec3_t cohesion(Vector<Boid> &boids) {
        float neighbourdist = 50;
        vec3_t sum = vec3_t(0, 0);
        int count = 0;
        for (Boid other : boids) {
            float d = distance(position, other.position);
            if ((d > 0) && (d < neighbourdist)) {
                sum += other.position;
                count++;
            }
        }
        if (count > 0) {
            sum /= count;
            return seek(sum);
        } else {
            return vec3_t(0, 0);
        }
    }

//    static float velocity;
//    float x;
//    float y;
//    float angle;
//    Boid() {
//      this->x = 0.0f;
//      this->x = 0.0f;
//      this->angle = 2.0f;
//    }
//    void randomize() {
//        this->x = random(0, SCREEN_WIDTH - 1);
//        this->y = random(0, SCREEN_HEIGHT - 1);
//        this->angle = random(0, 2*PI);
//    }
//    void update(float deltaTime) {
//        this->x += cos(this->angle) * velocity * deltaTime;
//        this->y += sin(this->angle) * velocity * deltaTime;
//        while (this->x < 0.0f) {
//            this->x += SCREEN_WIDTH;
//        }
//        while (this->y < 0.0f) {
//            this->y += SCREEN_HEIGHT;
//        }
//        while (this->x > SCREEN_WIDTH) {
//            this->x -= SCREEN_WIDTH;
//        }
//        while (this->y > SCREEN_HEIGHT) {
//            this->y -= SCREEN_HEIGHT;
//        }
//    }
//    void render() {
//        display.drawPixel(this->x, this->y, WHITE);
//    }
};
//float Boid::velocity = 50.0;

float distance(vec3_t from, vec3_t to) {
    return (from - to).mag();
}
void limit(vec3_t &vec, float max_magnitude) {
    float mag = vec.mag();
    if (mag > 0 && mag > max_magnitude) {
        vec /= mag;
        vec *= max_magnitude;
    }
}

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

void loop() {
    const int NUM_MAX_BOIDS = 5;
    Boid storage_array[NUM_MAX_BOIDS];
    Vector<Boid> boids(storage_array);
    for (int i = 0; i < NUM_MAX_BOIDS; ++i) {
        Boid boid = Boid();
        boid.initialize(10, 10);
        boids.push_back(boid);
    }
    while (true) {
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
//        for (int i = 0; i < NUM_BOIDS; ++i) {
//            boids[i].update(1.0f/50.0f);
//        }
//        for (int i = 0; i < NUM_BOIDS; ++i) {
//            boids[i].render();
//        }
        bool first = true;
        for (Boid &boid : boids) {
            boid.run(boids, first);
            first = false;
        }
        Serial.println();
        display.display();
        delay(50);
    }
}