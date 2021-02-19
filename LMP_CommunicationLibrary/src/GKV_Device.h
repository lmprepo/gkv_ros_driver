#ifndef GKV_DEVICE_H
#define GKV_DEVICE_H

#include <LMP_Device.h>
namespace Gyrovert
{
// windows class for GKV serial port comm
#ifdef _WIN32
#include <windows.h>
    class GKV_Device : public LMP_Device
    {
    public:
        HANDLE hSerial;
        GKV_Device(std::string serial_port, uint32_t baudrate) : LMP_Device()
        {
            SerialInitialized=InitSerialPort(serial_port, baudrate);
        }
        ~GKV_Device() {}

        bool InitSerialPort(std::string port_name, uint32_t baudrate)
        {
            hSerial = CreateFileA(port_name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
            if (hSerial == INVALID_HANDLE_VALUE)
            {
                if (GetLastError() == ERROR_FILE_NOT_FOUND)
                {
                    return 0;
                }
                return 0;
            }
            DCB dcbSerialParams = { 0 };
            dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
            if (!GetCommState(hSerial, &dcbSerialParams))
            {
                return 0;
            }
            dcbSerialParams.BaudRate = baudrate;
            dcbSerialParams.ByteSize = 8;
            dcbSerialParams.StopBits = ONESTOPBIT;
            dcbSerialParams.Parity = NOPARITY;
            if (!SetCommState(hSerial, &dcbSerialParams))
            {
                return 0;
            }
            return 1;
        }

        void WriteDataToGKV(GKV_PacketBase* data) override
        {
            if (SerialInitialized)
            {
                DWORD dwBytesWritten;
                char iRet = WriteFile(hSerial, data, data->length + 8, &dwBytesWritten, NULL);
                Sleep(1);
            }
        }
        char ReadDataFromGKV() override
        {
            if (SerialInitialized)
            {
                DWORD iSize;
                char sReceivedChar;
                char iRet = 0;
                while (true)
                {
                    iRet = ReadFile(hSerial, &sReceivedChar, 1, &iSize, 0);
                    if (iSize > 0)
                        return sReceivedChar;
                }
            }
            return 0;
        }
        bool GetSerialConnectionState()
        {
            return SerialInitialized;
        }
    private:
        bool SerialInitialized = false;
    };
#else
#ifdef __linux
#include <stdlib.h>
#include <unistd.h>     
#include <fcntl.h>      
#include <termios.h>   

    class GKV_Device : public LMP_Device
    {
    public:
        int SerialPortHandle;
        GKV_Device(std::string serial_port, uint32_t baudrate) : LMP_Device()
        {
            SerialInitialized = InitSerialPort(serial_port, baudrate);
        }
        ~GKV_Device() {}

        bool InitSerialPort(std::string port_name, uint32_t baudrate)
        {
            SerialPortHandle = open(port_name.c_str(), O_RDWR | O_NOCTTY);
            if (SerialPortHandle < 0) {
                return 0;
            }
            struct termios tty;
            struct termios tty_old;
            memset(&tty, 0, sizeof tty);
            /* Error Handling */
            if (tcgetattr(SerialPortHandle, &tty) != 0) {
                return 0;
            }
            /* Save old tty parameters */
            tty_old = tty;
            /* Set Baud Rate */
            cfsetospeed(&tty, (speed_t)baudrate);
            cfsetispeed(&tty, (speed_t)baudrate);
            /* Setting other Port Stuff */
            tty.c_cflag &= ~PARENB;            // Make 8n1
            tty.c_cflag &= ~CSTOPB;
            tty.c_cflag &= ~CSIZE;
            tty.c_cflag |= CS8;

            tty.c_cflag &= ~CRTSCTS;           // no flow control
            tty.c_cc[VMIN] = 1;                  // read doesn't block
            tty.c_cc[VTIME] = 5;                  // 0.5 seconds read timeout
            tty.c_cflag |= CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
            /* Make raw */
            cfmakeraw(&tty);
            /* Flush Port, then applies attributes */
            tcflush(SerialPortHandle, TCIFLUSH);
            if (tcsetattr(SerialPortHandle, TCSANOW, &tty) != 0) {
                return 0;
            }
            return 1;
        }

        void WriteDataToGKV(GKV_PacketBase* data) override
        {
            if (SerialInitialized)
            {
                int iOut = write(SerialPortHandle, data, data->length + 8);
                usleep(1000);
            }
        }
        char ReadDataFromGKV() override
        {
            if (SerialInitialized)
            {
                char sReceivedChar;
                while (true)
                {
                    int iOut = read(SerialPortHandle, &sReceivedChar, 1);
                    return sReceivedChar;
                }
            }
            return 0;
        }
        bool GetSerialConnectionState()
        {
            return SerialInitialized;
        }
    private:
        bool SerialInitialized = false;
};
#endif // __linux
#endif

}
#endif