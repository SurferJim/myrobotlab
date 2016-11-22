/**
	Welcome to Msg.cpp
	Its created by running ArduinoMsgGenerator
	which combines the MrlComm message schema (src/resource/Arduino/arduinoMsg.schema)
	with the cpp template (src/resource/Arduino/Msg.template.cpp)

	IDL Type Conversions

	IDL        ARDUINO
	none		byte
	boolean		boolean
    b16			int
    bu16		unsigned int
    b32			long
    bu32		unsigned long
    str			String (maybe)
    []			byte array

	All message editing should be done in the arduinoMsg.schema

	The binary wire format of an Arduino is:

	MAGIC_NUMBER|MSG_SIZE|METHOD_NUMBER|PARAM0|PARAM1 ...

*/



void Msg::publishMRLCommError(byte code) {
  write(MAGIC_NUMBER);
  write(2); // size
  write(1); // msgType = 1
  write(code);

  flush();
}

void Msg::publishBoardInfo(byte version, byte boardType) {
  write(MAGIC_NUMBER);
  write(3); // size
  write(3); // msgType = 3
  write(version);
  write(boardType);

  flush();
}

void Msg::publishBoardStatus(byte microsPerLoop, byte sram, byte deviceCount) {
  write(MAGIC_NUMBER);
  write(4); // size
  write(17); // msgType = 17
  write(microsPerLoop);
  write(sram);
  write(deviceCount);

  flush();
}

void Msg::publishMessageAck() {
  write(MAGIC_NUMBER);
  write(1); // size
  write(18); // msgType = 18

  flush();
}

