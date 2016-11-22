/**
	Welcome to Msg.h
	Its created by running ArduinoMsgGenerator
	which combines the MrlComm message schema (src/resource/Arduino/generate/arduinoMsg.schema)
	with the cpp template (src/resource/Arduino/generate/Msg.template.cpp)

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

#ifndef Msg_h
#define Msg_h

#include "MrlIo.h"

class MrlMsg : public MrlIo {

public:
	// send MrlComm --> Arduino methods
	void publishMRLCommError(byte code);
	void publishBoardInfo(byte version, byte boardType);
	void publishBoardStatus(byte microsPerLoop, byte sram, byte deviceCount);
	void publishMessageAck();


	// handles all MrlComm <-- Arduino
	void handle(int[] ioCmd); // send size too ?
}

#endif // Mrl_h
