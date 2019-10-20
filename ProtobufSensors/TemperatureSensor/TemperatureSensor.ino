// Our code
#include "sensor.pb.h"

// Protocol Buffer
#include "pb_common.h"
#include "pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

// Wifi connection
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// Defines
#define send_time_ms 20000
#define sensor_type  Sensor_SensorType_TEMPERATURE
#define sensor_id    2

unsigned long count_time;

WiFiUDP Udp;

char led_pin = 12;
char temperature_pin = A0;

const char* ssid     = "LUCAS-ESTEVES";
const char* password = "8zA008^1";

const char* server_ip     = "192.168.137.1";
const uint16_t server_port  = 7777;

const uint16_t local_port  = 8888;

uint8_t receive_buffer[CommandMessage_size];
char transmit_buffer[CommandMessage_size];

Sensor sensor = Sensor_init_zero;
CommandMessage msg = CommandMessage_init_zero;

pb_ostream_t stream_o = pb_ostream_from_buffer((uint8_t*)transmit_buffer, sizeof(transmit_buffer));
pb_istream_t stream_i = pb_istream_from_buffer(receive_buffer, sizeof(receive_buffer));

void set_state(float state){
  if(sensor.type == Sensor_SensorType_LIGHT){
    
    if(state == 1.0){
      digitalWrite(led_pin, HIGH);
    }else if (state == 0.0){
      digitalWrite(led_pin, LOW);
    }
    
    sensor.state = state;
  }
}

float get_state(){
  return (float(analogRead(temperature_pin))*3.3/(1023))/0.01;
}

void send_state(){
  // Update sensor state
  sensor.state = get_state();

  // Build reply message
  msg.command = CommandMessage_CommandType_SENSOR_STATE;
  msg.has_parameter =  true;
  msg.parameter = sensor;
  
  // Reset stream
  stream_o = pb_ostream_from_buffer((uint8_t*)transmit_buffer, sizeof(transmit_buffer));

  // Encode the message
  pb_encode(&stream_o, CommandMessage_fields, &msg); 

  // Send a reply, to the IP address and port that sent us the packet we received
  Udp.beginPacket(server_ip, server_port);
  Udp.write(transmit_buffer, stream_o.bytes_written);
  Udp.endPacket();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  sensor.type = sensor_type;
  sensor.id = sensor_id;
  sensor.state = 0;

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Udp.begin(local_port);

  Serial.println("UDP Started");

  // Sends a hello message to gateway
  send_state();

  Serial.println("Hello sent");
    
}

void loop() {
  // put your main code here, to run repeatedly:
  int packetSize = Udp.parsePacket();

  if (packetSize) {
    Serial.println("Got message!");

    // Read the packet
    int n = Udp.read(receive_buffer, UDP_TX_PACKET_MAX_SIZE);
    
    receive_buffer[n] = 0;

    // Decode the packet
    stream_i = pb_istream_from_buffer(receive_buffer, sizeof(receive_buffer));
    
    bool decoded = pb_decode(&stream_i, CommandMessage_fields, &msg);

    if(decoded){
    
      if(msg.command == CommandMessage_CommandType_GET_STATE){
        Serial.println("Sending state...");
        send_state();
                
      }else if(msg.command == CommandMessage_CommandType_SET_STATE){

        if(msg.has_parameter){
          Serial.println("Setting state...");
          set_state(msg.parameter.state);
          
        }
      }
    }
  }else if(millis() - count_time > send_time_ms){
    count_time = millis();
    
    Serial.println("Sending state...");
    send_state();
  }
}
