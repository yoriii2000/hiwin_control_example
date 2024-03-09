//	Name: Aldi
//	Email: Chaoliang94@gmail.com
//	Nationality: Indonesia


#include <iostream>
#include <windows.h> //For ONESTOPBIT, NOPARITY
#include <string>
#include <sstream> //For Stringstream
#include <vector> // For vector array

using namespace std;

class SerialCom {
public:
	//Data Members
	HANDLE serialHandle;
	DWORD  bytesRead;
	DWORD  BytesWritten = 0;

	// Member Functions()
	bool OpenSerialCom(string COM)
	{
		// Set the COM as following format " \\\\.\\COM2 "
		string COM_Input = "\\\\.\\" + COM;

		// Convert string to LPCSTR format.
		LPCSTR PortName = COM_Input.c_str();

		// Define the serial handle with following format:
		//port name, Read | Write, No Sharing, No Security, Open existing port only, Non overlapped I/O, Null for Comm Devices
		serialHandle = CreateFileA(PortName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

		// Check if serial port is opened.
		if (serialHandle == INVALID_HANDLE_VALUE)
		{
			cout << "Opening Serial Port [FALSE]" << endl;
			return FALSE;
		}
		else
		{
			cout << "Opening Serial Port [DONE]" << endl;
			return TRUE;
		}
	}

	void CloseSerialCom()
	{
		CloseHandle(serialHandle);
	}

	void SettingSerialCom(int BaudRate = 115200, BYTE ByteSize = 8, BYTE StopBits = ONESTOPBIT, BYTE Parity = NOPARITY)
	{
		// Do some basic settings
		DCB serialParams = { 0 };
		serialParams.DCBlength = sizeof(serialParams);

		GetCommState(serialHandle, &serialParams);
		serialParams.BaudRate = BaudRate;
		serialParams.ByteSize = ByteSize;
		serialParams.StopBits = StopBits;
		serialParams.Parity = Parity;
		SetCommState(serialHandle, &serialParams);

		// Set timeouts
		COMMTIMEOUTS timeout;
		timeout.ReadIntervalTimeout = 10;
		timeout.ReadTotalTimeoutConstant = 500;
		timeout.ReadTotalTimeoutMultiplier = 1;
		timeout.WriteTotalTimeoutConstant = 500;
		timeout.WriteTotalTimeoutMultiplier = 1;

		SetCommTimeouts(serialHandle, &timeout);
		
		cout << "Set the Serial Comm Settings: [DONE]" << endl;
	}

	string ReadSerialCom()
	{
		char ReadOutputPerBit[1] = {NULL};	// The data is read bit per bit, that's why char is defined as 1 element only
		string ReadOutput;
		
	/*	CONCEPT OF SERIAL COMMUNICATION:
		The data is sent letter per letter (bit per bit)
		Each letter maybe contains blankspace, letter, or \n
		"\n" means the data is completely sent, and go to next line data
		Because the data is sent letter per letter, so we need to make it run in real time, and when "\n" is sent, we stop and make them become string
		Then continue read the serial communication for next data.*/
		while (TRUE)
		{
			
			if (ReadFile(serialHandle, &ReadOutputPerBit, 1, &bytesRead, NULL) != 0)
			{
										
				if (ReadOutputPerBit[0] == '\n')		// If the data is \n, it means one line output data has been generated.
				{
					break;
				}
				else if (ReadOutputPerBit[0] == NULL)	// If there's no data received from serial comm.
				{
					cout << "No data is read from Serial Comm." << endl;
					break;
				}
				else
				{
					ReadOutput.push_back(ReadOutputPerBit[0]);	// The letter is sent to string
				}
			}
		}

		// ---------- To see the result of string---------
		//cout << "Read Serial Com:" << ReadOutput << endl;

		return ReadOutput;
	}

	// ================= Function: Splitting string with delimiter =========================
	template <typename Out>
	void split(const string &s, char delim, Out result)
	{
		istringstream iss(s);
		string item;
		while (getline(iss, item, delim))
		{
			if (!item.empty())
			{
				*result++ = item;
			}

		}
	}
	vector<string> split(const string &s, char delim)
	{
		vector<string> elems;
		split(s, delim, back_inserter(elems));
		return elems;
	}
	// ================= ====================== END ============================ =========================


	vector<string> GetValueSerialCom(string ReadSerialComOutput, char delimiter = ' ')
	{
		// Convert the string to value in vector array by splitting string with delimiter
		vector<string> SerialComValue = split(ReadSerialComOutput, delimiter);

		return SerialComValue;
	}

	bool Write(string SerialBuffer)
	{

		LPCVOID SerialBufferLPCVOID = SerialBuffer.c_str();
		bool WrittenStatus;
		WrittenStatus = WriteFile(serialHandle, SerialBufferLPCVOID, sizeof(SerialBuffer), &BytesWritten, NULL);
		
		if (WrittenStatus == FALSE)
		{
			cout << "Fail to write the data to serial comm." << endl;
			return FALSE;
		}
		else
		{
			cout << " Write: " << SerialBufferLPCVOID << endl;
			return TRUE;
		}
	}
};
