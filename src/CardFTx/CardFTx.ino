#include "M5Cardputer.h"
#include "src/Ft8ArduinoApp.h"

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println();
    Serial.println("CardFTx boot");

    auto cfg = M5.config();
    M5Cardputer.begin(cfg);
    M5Cardputer.Speaker.setVolume(255);
    M5Cardputer.Speaker.end();

    if (!ft8AppBegin()) {
        Serial.println("CardFTx init failed");
    }
}

void loop()
{
    M5Cardputer.update();
    ft8AppLoop();
}
