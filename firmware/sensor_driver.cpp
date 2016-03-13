/*
 * NOTE: Type of imu has to be defined in inv_mpu.h
 */

#include <ros.h>
#include <ros/time.h>

#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/RelativeHumidity.h>

#include <std_msgs/Bool.h>

#include "Dht11.h"
#include "TouchSensor.hpp"
#include "IRMotionDetectionSensor.hpp"

#define DHT_DATA_PIN 13
#define DHT_COOL_DOWN 500

Dht11 sensor(DHT_DATA_PIN);

TouchSensor t1(12);
TouchSensor t2(11);

IRMotionDetectionSensor motionDetection(10);

ros::NodeHandle nh;

sensor_msgs::Temperature temp_msg;
ros::Publisher temp_pub("temperature", &temp_msg);

sensor_msgs::RelativeHumidity humid_msg;
ros::Publisher humid_pub("humidity", &humid_msg);

std_msgs::Bool touchsensor_msg;
ros::Publisher t1_pub("touchsensor/1", &touchsensor_msg);
ros::Publisher t2_pub("touchsensor/2", &touchsensor_msg);

std_msgs::Bool motion_detection_msg;
ros::Publisher motion_detection_pub("motion_detection", &motion_detection_msg);

long dht11_publisher_timer;

void processDHT11(){
    if (millis() < dht11_publisher_timer) {
        return;
    }

    ros::Time now = nh.now();

    switch (sensor.read()) {
    case Dht11::OK:

        humid_msg.header.seq++;
        humid_msg.header.stamp = now;
        humid_msg.relative_humidity = sensor.getHumidity() / 100.0;
        humid_pub.publish(&humid_msg);


        temp_msg.header.seq++;
        temp_msg.header.stamp = now;
        temp_msg.temperature = sensor.getTemperature();
        temp_pub.publish(&temp_msg);

        dht11_publisher_timer = millis() + DHT_COOL_DOWN; //publish once a second

        break;

    case Dht11::ERROR_CHECKSUM:
        //Serial.println("Checksum error");
        //break;

    case Dht11::ERROR_TIMEOUT:
        //Serial.println("Timeout error");
        //break;

    default:
        //Serial.println("Unknown error");
        dht11_publisher_timer = millis() + 50;
        break;
    }

}

long touchsensor_publisher_timer;

void processTouchSensor(){
    if (millis() < touchsensor_publisher_timer) {
        return;
    }

    touchsensor_msg.data = t1.isPressed();
    t1_pub.publish(&touchsensor_msg);


    touchsensor_msg.data = t2.isPressed();
    t2_pub.publish(&touchsensor_msg);

    touchsensor_publisher_timer = millis() + 500;
}

long motion_detection_publisher_timer;

void processMotionDetection(){
    if (millis() < motion_detection_publisher_timer) {
        return;
    }

    motion_detection_msg.data = motionDetection.motionDetected();
    motion_detection_pub.publish(&motion_detection_msg);

    motion_detection_publisher_timer = millis() + 500;
}



int ret;
void setup() {

    humid_msg.header.frame_id = "arduino";

    temp_msg.header.frame_id = "arduino";

    nh.initNode();

    nh.advertise(temp_pub);

    nh.advertise(humid_pub);

    nh.advertise(t1_pub);

    nh.advertise(t2_pub);

    nh.advertise(motion_detection_pub);

}

void loop() {

    nh.spinOnce();

    processDHT11();

    processTouchSensor();

    processMotionDetection();

}

