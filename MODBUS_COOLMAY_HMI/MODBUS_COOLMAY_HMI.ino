#define MODBUS_BAUD_RATE 9600
#define RS485_RX_PIN 16
#define RS485_TX_PIN 17
#define RS485_DE_RE_PIN 4  // DE/RE pin for controlling TX/RX mode
#define MODBUS_SLAVE_ID 1
#define MODBUS_FUNCTION_READ_HOLDING_REGISTERS 0x03
#define MODBUS_FUNCTION_WRITE_SINGLE_REGISTER 0x06
// Received request: 1 6 0 0 0 A 9 CD

HardwareSerial RS485Serial(1);                                      // Using Serial1 for RS485 communication
uint16_t holdingRegisters[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };  // Holding register example

void setup() {
  Serial.begin(115200);
  RS485Serial.begin(MODBUS_BAUD_RATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);  // Start in receive mode

  Serial.println("Modbus RTU over RS485 initialized.");
}

void loop() {
  handleModbusRequest();
}

void handleModbusRequest() {
  byte request[8];  // Modbus RTU requests are usually 8 bytes
  uint8_t slaveId, functionCode;
  uint16_t registerAddress, valueOrLength;
  digitalWrite(RS485_DE_RE_PIN, LOW);  // Set DE/RE pin to receive mode

  // Wait for incoming data (Modbus frame is at least 8 bytes)
  if (RS485Serial.available() >= 8) {
    for (int i = 0; i < 8; i++) {
      request[i] = RS485Serial.read();
    }

    // CRC Check
    uint16_t receivedCRC = (request[6] | (request[7] << 8));  // CRC from request
    uint16_t calculatedCRC = calculateCRC16(request, 6);      // CRC on first 6 bytes

    if (receivedCRC != calculatedCRC) {
      Serial.println("CRC check failed!");
      return;  // Abort processing if CRC fails
    }

    // Extract slave ID, function code, and other details
    slaveId = request[0];
    functionCode = request[1];
    registerAddress = (request[2] << 8) | request[3];
    valueOrLength = (request[4] << 8) | request[5];

    // // Debug: Print the request bytes
    // Serial.print("Received request: ");
    // for (int i = 0; i < 8; i++) {
    //   Serial.print(request[i], HEX);
    //   Serial.print(" ");
    // }
    // Serial.println();

    // Check if request is for this slave
    if (slaveId != MODBUS_SLAVE_ID) {
      return;  // Ignore if it's not for this slave
    }

    // Handle Modbus function codes
    if (functionCode == MODBUS_FUNCTION_READ_HOLDING_REGISTERS) {
      readHoldingRegisters(registerAddress, valueOrLength);
    } else if (functionCode == MODBUS_FUNCTION_WRITE_SINGLE_REGISTER) {
      writeSingleRegister(registerAddress, valueOrLength);
      // Debug: Print the request bytes
      Serial.print("Received request: ");
      for (int i = 0; i < 8; i++) {
        Serial.print(request[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      // Unsupported function code, send error response
      sendErrorResponse(slaveId, functionCode, 0x01);  // 0x01 = Illegal function
    }
  }
}

void readHoldingRegisters(uint16_t registerAddress, uint16_t numRegisters) {
  if (registerAddress + numRegisters > 10) {  // Check register range
    Serial.println("Invalid register range.");
    return;
  }

  // Prepare response
  uint8_t response[5 + 2 * numRegisters];  // Slave ID, Function Code, Byte count, Data, CRC
  response[0] = MODBUS_SLAVE_ID;
  response[1] = MODBUS_FUNCTION_READ_HOLDING_REGISTERS;
  response[2] = numRegisters * 2;  // Byte count

  // Populate the register values
  for (int i = 0; i < numRegisters; i++) {
    response[3 + i * 2] = (holdingRegisters[registerAddress + i] >> 8) & 0xFF;  // High byte
    response[4 + i * 2] = holdingRegisters[registerAddress + i] & 0xFF;         // Low byte
  }

  // Append CRC
  uint16_t crc = calculateCRC16(response, 3 + 2 * numRegisters);
  response[3 + 2 * numRegisters] = crc & 0xFF;
  response[4 + 2 * numRegisters] = (crc >> 8) & 0xFF;

  // Send response
  sendModbusResponse(response, 5 + 2 * numRegisters);
}

void writeSingleRegister(uint16_t registerAddress, uint16_t value) {
  if (registerAddress >= 10) {  // Check if address is within range
    Serial.println("Invalid register address.");
    return;
  }

  // Write value to the register
  holdingRegisters[registerAddress] = value;

  // Echo back the request as a response
  byte response[8] = {
    MODBUS_SLAVE_ID,
    MODBUS_FUNCTION_WRITE_SINGLE_REGISTER,
    (registerAddress >> 8) & 0xFF, registerAddress & 0xFF,
    (value >> 8) & 0xFF, value & 0xFF
  };

  // Append CRC
  uint16_t crc = calculateCRC16(response, 6);
  response[6] = crc & 0xFF;
  response[7] = (crc >> 8) & 0xFF;

  sendModbusResponse(response, 8);
}

void sendErrorResponse(uint8_t slaveId, uint8_t functionCode, uint8_t exceptionCode) {
  // Construct Modbus error response (exception response)
  uint8_t response[5] = {
    slaveId,
    functionCode | 0x80,  // Set MSB to 1 for error
    exceptionCode
  };

  // Append CRC
  uint16_t crc = calculateCRC16(response, 3);
  response[3] = crc & 0xFF;
  response[4] = (crc >> 8) & 0xFF;

  sendModbusResponse(response, 5);
}

void sendModbusResponse(byte* response, size_t length) {
  digitalWrite(RS485_DE_RE_PIN, HIGH);  // Enable TX mode
  RS485Serial.write(response, length);
  RS485Serial.flush();                 // Ensure data is sent
  digitalWrite(RS485_DE_RE_PIN, LOW);  // Enable RX mode
}

// Example of CRC-16 calculation (Modbus)
uint16_t calculateCRC16(uint8_t* data, uint16_t length) {

  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}
