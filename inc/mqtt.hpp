#ifndef MQTT_HPP
#define MQTT_HPP

#include <iostream>
#include <mosquitto.h>
#include <string.h>
#include <unistd.h>

class Publisher
{
public:
    Publisher();
    bool sendMessage(const std::string& topic, const std::string& message);
    ~Publisher() 
    {
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
    }
private:
    struct mosquitto *mosq;
    int rc;

};

#endif