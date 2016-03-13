#ifndef ir_motion_detection_sensor_h
#define ir_motion_detection_sensor_h

#if defined(ARDUINO) && (ARDUINO >= 100)
#    include <Arduino.h>
#else
#    include <WProgram.h>
#endif

class IRMotionDetectionSensor{

private:
    // The pin over which we communicate with the sensor
    uint8_t pin;

public:
    IRMotionDetectionSensor(uint8_t newPin) : pin(newPin) {
        pinMode(newPin, INPUT);
    }

    bool motionDetected(){
        return digitalRead(this->pin);
    }

};


#endif
