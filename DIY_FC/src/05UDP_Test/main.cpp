#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

const int ledPin = 13;                                            // ####################################

void parseHexArray(const char *input, int *result, int &size) {
  size = 0;
  char cstr[strlen(input) + 1];
  strcpy(cstr, input);
  char *token = strtok(cstr, "[,]");
  while (token != nullptr) {
    int hexValue = strtol(token, nullptr, 16);
    int mappedValue = hexValue;
    result[size] = mappedValue;
    size++;
    token = strtok(nullptr, "[,]");
  }
}
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 1, 199);
unsigned int localPort = 8888;      // local port to listen on
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,
char ReplyBuffer[] = "acknowledged";        // a string to send back
EthernetUDP Udp;

// Define a new port for HEARTBEAT messages                                      #################################
unsigned int heartbeatPort = 8889; // Example port for heartbeat messages        #################################
EthernetUDP heartbeatUdp; // UDP instance for heartbeat messages                 #################################

void setup() {
  Serial.print("started setup");
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(8, OUTPUT);
  digitalWrite(2, LOW);
  digitalWrite(4, LOW);
  digitalWrite(6, LOW);
  digitalWrite(8, LOW);
  Ethernet.begin(mac, ip);
  Serial.begin(9600);

  pinMode(ledPin, OUTPUT);                                                     //######################################
    // Begin listening on the heartbeat port                                   //######################################
  heartbeatUdp.begin(heartbeatPort);                                          //######################################

unsigned long startMillis = millis();
while (!Serial && (millis() - startMillis < 5000)) {
  ; // wait for a maximum of 5 seconds for serial port to connect
}

  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }
  Udp.begin(localPort);
  Serial.print("finished setup");
}

bool commandHighPin4 = false; // Flag to indicate if pin 4 should be HIGH due to command
unsigned long lossStartTime = 0; // Variable to hold the time when the connection was first lost

void loop() {
    //Serial.print("in loop");
  // Check for HEARTBEAT messages                            #####################################
  int heartbeatPacketSize = heartbeatUdp.parsePacket();
  if (heartbeatPacketSize) {
    //Serial.println("Received HEARTBEAT");
    // Read the packet into packetBuffer
    heartbeatUdp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    // Check if it is a HEARTBEAT message
    if (strcmp(packetBuffer, "HEARTBEAT") == 0) {
      Serial.println("Got HEARTBEAT");
          // Ethernet link is restored
      if (!commandHighPin4) {
        digitalWrite(ledPin, LOW); // Only set pin 4 to LOW if there's no command to set it to HIGH
      }
    lossStartTime = 0;  // Reset the timer}
    }
    // Clear the packetBuffer after processing 
    memset(packetBuffer, 0, UDP_TX_PACKET_MAX_SIZE);
  }
  else {
    if (lossStartTime == 0) { // If this is the first time the link was lost, capture the current time
      lossStartTime = millis();
    } else if (millis() - lossStartTime >= 5000) { // 5 seconds have passed since the link was first lost
      // Write VCC level voltage to digital pin 4
      digitalWrite(ledPin, HIGH);
      Serial.println("NO heartbeat");
      commandHighPin4 = false;  // Reset the flag
    }
  }
  // End of the Heartbeat status check                      ########################################

  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remote = Udp.remoteIP();
    for (int i=0; i < 4; i++) {
      Serial.print(remote[i], DEC);
      if (i < 3) {
        Serial.print(".");
      }
    }
    Serial.print(", port ");
    Serial.println(Udp.remotePort());
    // read the packet into packetBufffer
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    Serial.println("Contents:");
    Serial.println(packetBuffer);
  int result[10]; // Assuming a maximum of 10 elements, you can adjust this
  int size; // The actual size of the array
  // Parse the input string
  parseHexArray(packetBuffer, result, size);
  Serial.print("Parsed array: ");
  for (int i = 0; i < size; i++) {
    Serial.print(result[i]);
    Serial.print(" ");
  }
  Serial.println();
  digitalWrite(2, result[0]);
  digitalWrite(4, result[1]);
  digitalWrite(6, result[2]);
  digitalWrite(8, result[3]);
  if (result[1] == HIGH) {
    commandHighPin4 = true;
  }// since pin 4 is for regular reward and disconnect reward, a flag is added 

    // send a reply to the IP address and port that sent us the packet we received
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(ReplyBuffer);
    Udp.endPacket();
  }
  delay(10);
}