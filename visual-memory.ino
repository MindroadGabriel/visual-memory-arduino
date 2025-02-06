#include <Vector.h>

#include <Vector_datatype.h>
#include <quaternion_type.h>
#include <vector_type.h>

#include <Bounce2.h>

#include <Adafruit_SSD1306.h>
#include <splash.h>

#include <Wire.h>

Bounce button1 = Bounce();
Bounce button2 = Bounce();

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define CIRCLE_RADIUS 3

#define OLED_RESET     4 // Reset pq  §in # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void writeString(char* text) {
    char* c = text;
    while (*c) {
        display.write(*c);
        c++;
    }
}

void writeFloatString(float n) {
    char buffer[128];
    dtostrf(n, 4, 1, buffer);
    int len = strlen(buffer);
    Serial.print("Len:");
    int desired_length = 6;
    for (int i = 0; i < len - desired_length; ++i) {
        writeString(" ");
    }
    writeString(buffer);
}
void displayTemporaryMessage(const char* message, int delayTime) {
    display.clearDisplay();
    display.setCursor(0, 0);
    writeString(message);
    display.display();
    delay(delayTime);
    display.clearDisplay();
    display.setCursor(0, 0);
}
void setup() {
    Wire.begin();
    Wire.setClock(10000); //400khz clock
    Serial.begin(9600);
    while (!Serial) { ; }
    randomSeed(analogRead(0));

    pinMode(5, INPUT_PULLUP);
    button1.attach(5);
    button1.interval(10);// interval in ms
    pinMode(4, INPUT_PULLUP);
    button2.attach(4);
    button2.interval(10); // interval in ms

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
        Serial.println(F("SSD1306 allocation failed"));
        for (;;); // Don't proceed, loop forever
    }// */
    display.clearDisplay();
    display.display();
    display.clearDisplay();

    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(WHITE); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner
    display.cp437(true);         // Use full 256 char 'Code Page 437' font
    display.display();
}

void generateSequence(Vector<bool>& sequence, int length) {
    sequence.clear();
    for (int i = 0; i < length; ++i) {
        int random_int = random(0, 2);
        Serial.print(", ");
        Serial.print(random_int);
        sequence.push_back(random_int > 0);
    }
    Serial.println("");
}

void renderSequence(Vector<bool>& sequence, int subsection) {
    for (int i = 0; i < subsection; ++i) {
        bool box = sequence[i];
        if (box) {
            display.write(0xDB);
        }
        else {
            display.write(0x5F);
        }
    }
}

enum class GameState {
    Menu,
    Displaying,
    Inputting,
    Failure,
    Score,
};
void loop() {
    const int maxSequence = 128;
    bool sequenceStorage[maxSequence];
    Vector<bool> sequence(sequenceStorage);
    sequence.clear();
    GameState gameState = GameState::Menu;
    int nextGuessIndex = 0;
    int highestCleared = 0;
    bool first = true;

    while (true) {
        button1.update();
        button2.update();
        display.clearDisplay();
        display.setCursor(0, 0);
        switch (gameState) {
            case GameState::Menu: {
                writeString("Sequence memory!");
                writeString("\nPush both buttons.\n");
                if (button1.read() == LOW && button2.read() == LOW) {
                    gameState = GameState::Displaying;
                    nextGuessIndex = 0;
                    highestCleared = 0;
                    generateSequence(sequence, 3);
                    first = true;
                } else {
                    if (button1.fell()) {
                        sequence.push_back(false);
                    } else if (button2.fell()) {
                        sequence.push_back(true);
                    }
                    renderSequence(sequence, sequence.size());
                }
                break;
            }
            case GameState::Displaying: {
                if (first) {
                    displayTemporaryMessage("Remember!", 1000);
                }
                writeString(": ");
                renderSequence(sequence, sequence.size());
                display.display();
                delay(2000);
                gameState = GameState::Inputting;
                if (first) {
                    displayTemporaryMessage("Repeat!", 1000);
                }
                break;
            }
            case GameState::Inputting: {
                writeString(": ");
                if (button1.fell()) {
                    if (sequence[nextGuessIndex] == false) {
                        nextGuessIndex++;
                    } else {
                        gameState = GameState::Failure;
                    }
                } else if (button2.fell()) {
                    if (sequence[nextGuessIndex] == true) {
                        nextGuessIndex++;
                    } else {
                        gameState = GameState::Failure;
                    }
                }
                renderSequence(sequence, nextGuessIndex);
                if (nextGuessIndex == sequence.size()) {
                    if (first) {
                        displayTemporaryMessage("Good!", 1000);
                        displayTemporaryMessage("Next sequence:", 1000);
                    } else {
                        displayTemporaryMessage("Good! Next:", 400);
                    }
                    generateSequence(sequence, sequence.size() + 1);
                    nextGuessIndex = 0;
                    highestCleared = sequence.size();
                    gameState = GameState::Displaying;
                    first = false;
                }
                break;
            }
            case GameState::Failure: {
                displayTemporaryMessage("No!", 200);
                renderSequence(sequence, sequence.size());
                display.display();
                delay(1500);
                gameState = GameState::Score;
                break;
            }
            case GameState::Score: {
                writeString("You cleared ");
                float score = highestCleared + (float)nextGuessIndex / (float)sequence.size();
                writeFloatString(score);
                writeString("\nsequences!");
                if (button1.fell() || button2.fell()) {
                    gameState = GameState::Menu;
                    sequence.clear();
                }
                break;
            }
        }


        display.display();
        delay(10);
    }
}