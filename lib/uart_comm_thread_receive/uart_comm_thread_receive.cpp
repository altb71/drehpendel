// includes
#include <cstdint>
#include "uart_comm_thread_receive.h"

extern DataLogger myDataLogger;
extern GPA myGPA;

// #### constructor
uart_comm_thread_receive::uart_comm_thread_receive(BufferedSerial *com, float Ts):
                                            thread(osPriorityBelowNormal1, 512*2)//thread(osPriorityHigh1, 1024)//
 {  
    // init serial
    this->uart = com;
    this->Ts = Ts;
}

uart_comm_thread_receive::~uart_comm_thread_receive() {}

void uart_comm_thread_receive::start_uart(void){
	thread.start(callback(this, &uart_comm_thread_receive::loop));
	ticker.attach(callback(this, &uart_comm_thread_receive::sendThreadFlag), Ts);
}

void uart_comm_thread_receive::sendThreadFlag() {
    thread.flags_set(threadFlag);
}

void uart_comm_thread_receive::loop(void)
{
	while(true)
    {
        ThisThread::flags_wait_any(threadFlag);
		readUartIntoSeparateMessageBuffer();
        //read_1_float();
	}
}

void uart_comm_thread_receive::recoverFromReadError(char newByte){
	if(isCsmErrorFlag){
		recovery_count++;
		char prevByte0 = errBuffer[errIx];
		bool isTerminatorReceived = (prevByte0 == UART_TERM_BY0 &&
									 newByte   == UART_TERM_BY1);
		if(isTerminatorReceived){
            messageBufIndex = 0;
			isCsmErrorFlag = false;
			recovery_count = 0;
		} else {
			char prevByte1 = errBuffer[(errIx+UART_ERR_LEN-1)%UART_ERR_LEN];
			bool isStartReceived = (prevByte1 == UART_HEAD_BY0 &&
									prevByte0 == UART_HEAD_BY1 &&
									newByte   == UART_HEAD_BY2 );
			if(isStartReceived){
				messageBuffer[0] = UART_HEAD_BY0;
				messageBuffer[1] = UART_HEAD_BY1;
				messageBuffer[2] = UART_HEAD_BY2;
            	messageBufIndex = 3;
				isCsmErrorFlag = false;
				recovery_count = 0;
			}
		}
	}
	errIx = (errIx+1)%UART_ERR_LEN;
	errBuffer[errIx] = newByte;
}

void uart_comm_thread_receive::readUartIntoSeparateMessageBuffer(){
     char pre[2] = {'\r','\n'};
    while(uart->readable()){
		bufLengthLLL = bufLengthLL;
		bufLengthLL = bufLengthLast;
		bufLengthLast = bufLength;
        bufLength = uart->read(buffer_rx, sizeof(buffer_rx));
		// if(bufLength==30){
		// 	bool fault = buffer_rx[0] == UART_HEAD_BY0 && 
		// 					buffer_rx[1] == UART_HEAD_BY1 && 
		// 					buffer_rx[2] == UART_HEAD_BY2 && 
		// 					buffer_rx[26] == 0 && 
		// 					buffer_rx[27] == 253;
		// 	if(fault){
		// 		m_data->debug_var[4] = msgCount;
		// 	}
		// }
        uint8_t k3,k2,k1,k0;
        for (int i = 0; i < bufLength; i++){
            messageBuffer[messageBufIndex] = buffer_rx[i]; // copy to separate buffer
			byte_count++;
            // Read Header
            if(messageBufIndex == 1){
                checkMessageBufferHeader();
            } 
            // Read Completed Message
            bool isMessageLengthReached = (messageBufIndex % 6  == 5) & headerValid;
            if(isMessageLengthReached){
                headerValid = false;
                k3 = (messageBufIndex-3+messageBufLen)%messageBufLen;
                k2 = (messageBufIndex-2+messageBufLen)%messageBufLen;
                k1 = (messageBufIndex-1+messageBufLen)%messageBufLen;
                k0 = messageBufIndex;
                myDataLogger.uint8_data[0] = messageBuffer[k3];
                myDataLogger.uint8_data[1] = messageBuffer[k2];
                myDataLogger.uint8_data[2] = messageBuffer[k1];
                myDataLogger.uint8_data[3] = messageBuffer[k0];
                messageBufIndex = ++messageBufIndex%messageBufLen;				// reset message buffer index to start (waiting for next message)
                uart->write(pre,2);
                uart->write(myDataLogger.uint8_data,4);
			}
			messageBufIndex = (messageBufIndex+1) % messageBufLen;
			// Recovery
			//recoverFromReadError(buffer_rx[i]);
        }
    }
}
void uart_comm_thread_receive::read_1_float()
{
    //bufLength = 0;
    //for(uint8_t k=0;k<6;k++)
    //    myDataLogger.uint8_data[k] = 100+k;//buffer_rx[k];
     while(uart->readable()){
		bufLength = uart->read(buffer_rx, sizeof(buffer_rx));
        for(uint8_t k=0;k<(bufLength-1);k++)
            {
            if(buffer_rx[k]==13 && buffer_rx[k+1]==10)
                {
                bufLength = uart->read(buffer_rx, sizeof(buffer_rx));
                if(bufLength>=4)
                    for(uint8_t k=0;k<4;k++)
                        myDataLogger.uint8_data[k] = buffer_rx[k];
                else
                    myDataLogger.uint8_data[0] = bufLength+100;
                }
            }
        }
//buffer_rx[5] = bufLength;
}

void uart_comm_thread_receive::checkMessageBufferHeader(){
	headerValid = 	messageBuffer[(messageBufIndex+messageBufLen-1)%messageBufLen] == 13 && 
						messageBuffer[messageBufIndex] == 10;
	isCsmErrorFlag = true;
	head_err_cnt += !headerValid;
	
}

int uart_comm_thread_receive::parseMessageBufferDataSize(){
	uint16_t dataSize = 256 * messageBuffer[6] + messageBuffer[5];
	dataSize =  min(dataSize,(uint16_t)UART_DATA_LEN);
	return dataSize;
}

bool uart_comm_thread_receive::verifyChecksumValid(uint16_t dataSize){
	char csm_calc = 0;
	bool csm_valid = false;
	if(dataSize>0 && dataSize<=UART_DATA_LEN){
		uint32_t csm_len = UART_HEAD_LEN+dataSize;
		if(csm_len<UART_BUF_LEN){
			for(int i=0; i<csm_len; i++){
				csm_calc += messageBuffer[i];
			}
			csm_valid = csm_calc == messageBuffer[csm_len];
		}
	}
	return csm_valid;
}

// -----------------------------------------------------------------------------
// analyse data, see comments at top of this file for numbering
bool uart_comm_thread_receive::parseMessageBuffer(int i){
    receive_cnt++;
	myDataLogger.uint8_data[0] = messageBuffer[2];
    myDataLogger.uint8_data[1] = messageBuffer[3];
    myDataLogger.uint8_data[2] = messageBuffer[4];
    myDataLogger.uint8_data[3] = messageBuffer[5];
	return false;	
}

