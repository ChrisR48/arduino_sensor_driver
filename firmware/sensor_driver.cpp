/*
 * NOTE: Type of imu has to be defined in inv_mpu.h
 */

#include <ros.h>
#include <ros/time.h>

#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/RelativeHumidity.h>

#include "Dht11.h"

#define DHT_DATA_PIN 13
#define DHT_COOL_DOWN 500

Dht11 sensor(DHT_DATA_PIN);

ros::NodeHandle nh;

sensor_msgs::Temperature temp_msg;
ros::Publisher temp_pub("temperature", &temp_msg);

sensor_msgs::RelativeHumidity humid_msg;
ros::Publisher humid_pub("humidity", &humid_msg);

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

int ret;
void setup() {

    humid_msg.header.frame_id = "arduino";

    temp_msg.header.frame_id = "arduino";

    nh.initNode();

    nh.advertise(temp_pub);

    nh.advertise(humid_pub);

}

void loop() {

    nh.spinOnce();

    processDHT11();

}

