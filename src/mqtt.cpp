#include "mqtt.hpp"

void Publisher::Publisher()
{
    mosquitto_lib_init();

    mosq = mosquitto_new(NULL, true, NULL);
    if(!mosq){
        std::cerr << "Failed to create client instance." << std::endl;
        return;
    }

    rc = mosquitto_connect(mosq, "localhost", 1883, 60);
    if(rc != MOSQ_ERR_SUCCESS){
        std::cerr << "Connect failed: " << mosquitto_strerror(rc) << std::endl;
        return;
    }

    std::cout << "Connected! Ready to publish messages..." << std::endl;
}

bool Publisher::sendMessage(const std::string& topic, const std::string& message)
{
    rc = mosquitto_publish(mosq, NULL, topic.c_str(), message.length(), message.c_str(), 0, false);
    
    if(rc != MOSQ_ERR_SUCCESS){
        std::cerr << "Error publishing: " << mosquitto_strerror(rc) << std::endl;
        return false;
    }

    mosquitto_loop(mosq, 0, 1);
    return true;
}