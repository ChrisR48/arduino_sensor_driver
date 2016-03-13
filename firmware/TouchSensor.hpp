#ifndef touchsensor_h
#define touchsensor_h

#if defined(ARDUINO) && (ARDUINO >= 100)
#    include <Arduino.h>
#else
#    include <WProgram.h>
#endif

class TouchSensor{

private:
    // The pin over which we communicate with the sensor
    uint8_t pin;

public:
    TouchSensor(uint8_t newPin) : pin(newPin) {
        pinMode(newPin, INPUT);
    }

    bool isPressed(){
        return digitalRead(this->pin);
    }

};


#endif
