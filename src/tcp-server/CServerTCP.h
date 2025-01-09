/**
 * @file tcpServer.hpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-05-23
 * @note: References: GPT4o, IBM documentation (e.g., https://www.ibm.com/docs/en/zos/3.1.0?topic=functions-bind-bind-name-socket)
 */

#pragma once
#include <iostream>
#include <iomanip>
#include <memory>

// TCP socketsincludes
#include <chrono>
#include <unistd.h>
#ifdef _WIN32
// WINDOWS
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#define close_socket(s) closesocket(s)
#else
// LINUX
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#define close_socket(s) close(s)
#endif

// Protobufs includes

// DEFINES
#define MAX_NUM_OF_CLIENTS 1
#define MAX_RECV_BUFFER_SIZE 1024
#define MAX_SEND_BUFFER_SIZE 1024

#define POSDATA_ENTRIES 12                                     // Number of entries in a pose array
#define POSDATA_ROW_SIZE (POSDATA_ENTRIES * __SIZEOF_DOUBLE__) // Bytes corresponding to double 1x12 (position line for one control group)

#define JOINTDATA_ENTRIES 8                                     // Number of entries in a joint array
#define JOINTDATA_ROW_SIZE (JOINTDATA_ENTRIES * __SIZEOF_DOUBLE__) // Bytes corresponding to double 1x8 (joint line for one control group)

// Aliases
using std::string, std::cout, std::endl;

class tcpServer
{
public:
    // CONSTRUCTOR, DEVNOTE: does this work? It should create a unique pointer to controller API within the server object
    tcpServer(int portNumber_in) : portNumber_(portNumber_in) {};

    // DESTRUCTOR
    ~tcpServer()
    {
        // Server shutdown if open
        if (serverSocketDescriptor_ >= 0)
        {
            close(serverSocketDescriptor_);
        }
    };

    // DATA MEMBERS
    enum class CommModeType
    {
        SHUTDOWN = -1,
        PROTOBUF = 0,
        HARDWIRED_ID_EXAMPLE = 1
    };

    enum class HARDWIRED_MSG_SIZE
    {
    };
    // METHODS

    /**
     * @brief
     *
     */
    void Initialize();
    int AcceptConnections();

    int HandleClientRequest(int clientSocketDescriptor);

    void ReadBufferData();  // Method to read data from buffer
    void WriteBufferData(); // Method to write data to buffer

    // Methods to handle protobuf communication
    // TODO

    // Methods to handle hardwired messaged
    void ProcessHardWiredMsg();

private:
    // DATA MEMBERS

    struct sockaddr_in serverAddr_; // Server address struct
    struct sockaddr_in clientAddr_; // Client address struct
    int serverSocketDescriptor_, clientSocketDescriptor_;

    socklen_t serverAddrLen_ = sizeof(serverAddr_);
    socklen_t clientAddrLen_ = sizeof(clientAddr_);

    bool isClientConnected = false; // Default status: no client connected
    int serverInitStatusCode_ = -1; // Default status code before server initialization: not initialized
    int lastMessageSize_ = 0;
    int portNumber_ = 55555;
    CommModeType commMode = CommModeType::PROTOBUF; // Default communication mode

    int bufferToReceiveSize_ = 0;
    int bufferToSendSize_ = 0;

    char bufferToReceive_[MAX_RECV_BUFFER_SIZE];
    char bufferToSend_[MAX_SEND_BUFFER_SIZE];

    // FUNCTION MEMBERS
    void DefineSocket();
    void BindSocket();
    void ListenToConnection();
    bool ValidateReadBufferLength(int expectedBufferSize);
    bool ValidateWriteBufferLength(int expectedBufferSize);
};
