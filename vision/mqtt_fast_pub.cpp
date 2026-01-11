#include <iostream>
#include <mosquitto.h>
#include <string.h>
#include <unistd.h>

int main() {
    struct mosquitto *mosq;
    int rc;

    // Initialize the mosquitto library
    mosquitto_lib_init();

    // Create a new client. NULL means a random ID will be generated.
    mosq = mosquitto_new(NULL, true, NULL);
    if(!mosq){
        std::cerr << "Failed to create client instance." << std::endl;
        return 1;
    }

    // Connect to your Pi's broker (localhost)
    // 1883 is the default port, 60 is the keepalive timer
    rc = mosquitto_connect(mosq, "localhost", 1883, 60);
    if(rc != MOSQ_ERR_SUCCESS){
        std::cerr << "Connect failed: " << mosquitto_strerror(rc) << std::endl;
        return 1;
    }

    std::cout << "Connected! Continuously publishing to 'test/topic'..." << std::endl;

    int count = 0;
    while(true) {
        std::string payload = "Message #" + std::to_string(count++);
        
        // Publish the message
        // NULL: we don't need the message ID
        // 0: Quality of Service (QoS) 0 (Fastest)
        // false: Don't "retain" the message on the broker
        rc = mosquitto_publish(mosq, NULL, "test/topic", payload.length(), payload.c_str(), 0, false);
        
        if(rc != MOSQ_ERR_SUCCESS){
            std::cerr << "Error publishing: " << mosquitto_strerror(rc) << std::endl;
        }

        // Must call loop or loop_write to actually push the data to the network
        mosquitto_loop(mosq, 0, 1);

        usleep(1000000); // Wait 1 second (1,000,000 microseconds)
    }

    // Cleanup (though we won't reach here in this infinite loop)
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return 0;
}
