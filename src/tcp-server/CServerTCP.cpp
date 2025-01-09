/**
 * @file CServerTCP.cpp
 * @author PeterC (petercalifano.gs@gmail.com)
 * @brief
 * @version 0.1
 * @date 2025-01-09
 * @note: References: GPT4o, IBM documentation (e.g., https://www.ibm.com/docs/en/zos/3.1.0?topic=functions-bind-bind-name-socket)
 */

#include <iostream>
#include <iomanip>
#include <assert.h>
#include <string.h>

// TCP socketsincludes
#include <chrono>
#include <thread>
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

// tcpserver header
#include "CServerTCP.h"

using std::string, std::cout, std::endl;

// TCP SERVER MANAGER IMPLEMENTATION
void tcpServer::DefineSocket()
{
    // Create socket
    // NOTE: AF_INET --> IPv4, SOCK_STREAM --> TCP
    serverSocketDescriptor_ = socket(AF_INET, SOCK_STREAM, 0);
    serverInitStatusCode_ = serverSocketDescriptor_;

    int opt = 1;
    setsockopt(serverSocketDescriptor_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (serverInitStatusCode_ < 0)
    {
        std::__throw_runtime_error("Socket creation has failed. Execution stop.");
    };

    memset((char *)&serverAddr_, 0, sizeof(serverAddr_)); // Clear out the address memory

    serverAddr_.sin_family = AF_INET;
    serverAddr_.sin_port = htons(portNumber_);
    serverAddr_.sin_addr.s_addr = INADDR_ANY; // Server address. INADDR_ANY = Server will accept connections to all IP adresses of the host machine
};

/**
 * @brief
 * @author Pietro Califano
 * @date 2024-06-03
 */
void tcpServer::BindSocket()
{
    // Bind socket to server address
    serverInitStatusCode_ = bind(serverSocketDescriptor_, (struct sockaddr *)&serverAddr_, sizeof(serverAddr_));

    if (serverInitStatusCode_ < 0)
    {
        std::__throw_runtime_error("Socket binding has failed. Execution stop.");
    };
}

/**
 * @brief
 * @author Pietro Califano
 * @date 2024-06-03
 */
void tcpServer::ListenToConnection()
{
    serverInitStatusCode_ = listen(serverSocketDescriptor_, MAX_NUM_OF_CLIENTS);
    if (serverInitStatusCode_ < 0)
    {
        std::__throw_runtime_error("Socket has failed while listening to connections. Execution stop.");
    };
};

/**
 * @brief
 * @author Pietro Califano
 * @date 2024-06-16
 * @return int
 */
void tcpServer::Initialize()
{
    cout << "\nInitializing server... " << endl;
    // Define socket
    DefineSocket();
    cout << "Defined socket with HOST: " << serverAddr_.sin_addr.s_addr << " on PORT: " << portNumber_ << endl;

    // Bind socket
    BindSocket();
    cout << "Socket binding completed" << endl;

    // Start listening to connections
    ListenToConnection();
    cout << "Server has started listening to client connections..." << endl;
}

/**
 * @brief
 * @author Pietro Califano
 * @date 2024-06-03
 */
int tcpServer::AcceptConnections()
{
    while (true && serverSocketDescriptor_ > 0)
    {
        try
        { // While true, continue listening, accepting connections and handling requests
            clientSocketDescriptor_ = accept(serverSocketDescriptor_, (struct sockaddr *)&clientAddr_, &clientAddrLen_);

#if (DEBUG_MODE)
            cout << "Client descriptor: " << clientSocketDescriptor_ << endl;
#endif

            if (clientSocketDescriptor_ < 0)
            {
                std::cerr << "Accepting connection has failed... Closing connection with client at " << &clientAddr_ << std::endl;
                isClientConnected = false;
                continue;
            }
            else
            {
                std::cout << "\nEstablished connection with client: " << &clientAddr_ << endl;
                isClientConnected = true;
            };

            while (isClientConnected)
            {

                cout << "Waiting for Request from client reading buffer data..." << endl;
                HandleClientRequest(clientSocketDescriptor_);


            }

            // Close connection after having completed all the operations and client disconnection

            cout << "Closing connection with client at " << &clientAddr_ << std::endl;
            close(clientSocketDescriptor_);
            isClientConnected = false;
            cout << "Connection with client at " << &clientAddr_ << " CLOSED" << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error while accepting and handling request due to exception: " << e.what() << endl;
            std::cout << "Closing connection with client at " << &clientAddr_ << std::endl;
            close(clientSocketDescriptor_);
            isClientConnected = false;
            continue;
        }

#if (DEBUG_MODE)
        cout << "Waiting for connections... " << endl;
        cout << "serverSocketDescriptor_: " << serverSocketDescriptor_ << endl;
#endif
    }

    // Client has disconnected --> close connection server side
    close(clientSocketDescriptor_);
    isClientConnected = false;
    return 0;
};

/**
 * @brief Method to handle client request and process data
 * @author Pietro Califano
 * @date 2024-06-16
 * @param clientSocketDescriptor
 * @return int
 */
int tcpServer::HandleClientRequest(int clientSocketDescriptor)
{

    // Read buffer data from client as raw bytes stream
    ReadBufferData();
    cout << "---------------------------- HANDLING OF REQUEST BEGINS ----------------------------" << endl;

    // Process buffer depending on communication mode

    // PaylcheckForShutdownBufferoad data processing based on communication mode
    if (commMode == CommModeType::PROTOBUF)
    {
        cout << "ENTERING --> PROTOBUF MODE" << endl;
        // PROTOBUF MODE
    }
    else if (commMode != CommModeType::PROTOBUF)
    {
        cout << "ENTERING --> HARDWIRED MESSAGE MODE" << endl;
        // HARDWIRED MESSAGE: Call hardwired messages deserialized
        ProcessHardWiredMsg();
    }

    // Sending message back to client
    WriteBufferData();
    cout << "Buffer of size " << bufferToSendSize_ << " correctly sent to client. Request handling completed." << endl;
    cout << "---------------------------- HANDLING OF REQUEST ENDS ----------------------------" << endl;
    return 0;
};

/**
 * @brief Function to handle processing of hardwired messages supported by the server (application specific)
 * @author Pietro Califano
 * @date 
 * @details This performs deserialization, execution of operations and serialization based on the request type from client
 * @return char
 * // TODO (PC) Templatize this using variadic and make virtual (tag dispatching?)
 */
void tcpServer::ProcessHardWiredMsg()
{
    bool isValid = false;
    // Check enum type and related expected message length
    switch (commMode)
    {
    case CommModeType::HARDWIRED_ID_EXAMPLE:

    { // Single Joint Motion command to move to a specified pose

        if (isValid)
        {

        }
        else
        {
            //cout << "\nMessage size validation failed. Valid sizes: JOINTS:" << expectedSize_JOINTS << ", POSE: " << expectedSize_POSE << ". Received size:" << lastMessageSize_ << endl;
        }
        break;
    }
    default:
    {
        std::__throw_runtime_error("Message ID not matching any of the allowed hardwired messages!");
    }
    };
};

/**
 * @brief Function to verify that received message matches the expected message length
 * @author Pietro Califano
 * @date 2024-06-16
 * @return true
 * @return false
 * // TODO (PC) Templatize this using variadic and make virtual (tag dispatching?)
 */
bool tcpServer::ValidateReadBufferLength(int expectedBufferSize)
{
    return (lastMessageSize_ == expectedBufferSize) ? true : false;
};

/**
 * @brief Function to verify that message to write matches the expected message length
 * @author Pietro Califano
 * @date 2024-06-25
 * @return true
 * @return false
 * // TODO (PC) Templatize this using variadic and make virtual (tag dispatching?)
 */
bool tcpServer::ValidateWriteBufferLength(int expectedBufferSize)
{
    // NOTE: bufferToSendSize_ -8 to exclude the first 8 bytes of the buffer (length and commMode)
    return ((bufferToSendSize_ - 8) == expectedBufferSize) ? true : false;
};

/**
 * @brief
 * @author Pietro Califano
 * @date 
 * // TODO (PC) Make this function generic
 */
void tcpServer::ReadBufferData()
{
    // Receive data from client
    ssize_t totalPayloadBytesReceived = 0.0;
    ssize_t numOfPayloadReadBytes = 0.0;

    if (isClientConnected)
    {
        // Read 4 bytes for message length0
        char dataLengthBuffer[4];
        int bytesRecv = recv(clientSocketDescriptor_, dataLengthBuffer, 4, 0);

        cout << "recv skipped with bytes: " << bytesRecv << endl;

        memcpy(&bufferToReceiveSize_, dataLengthBuffer, 4); // Get number of bytes to receive

        // bufferToReceiveSize_ -= 4; // Do not count commMode bytes into payload length
        cout << "\tReading buffer from client... \n\tGot length of message. Receiving payload of length: " << bufferToReceiveSize_ - 4 << endl;

        // Read 4 bytes for communication mode specifier
        //char commModeBuffer[4];
        recv(clientSocketDescriptor_, &commMode, 4, 0);

        cout << "\tClient specified commMode: " << (int)commMode << endl;

        if (commMode == CommModeType::SHUTDOWN)
        {
            // Shutdown command received -_> start server shutdown procedure
            cout << "Server shutdown command received... Closing server." << endl;
            // Send acknowledge of server shutdown commmand received to client
            // WriteBuffer()
            close(clientSocketDescriptor_);
            serverSocketDescriptor_ = -1;
        }

        // Check to verify that MESSAGE LENGTH < MAX_BUFFER_SIZE
        if (bufferToReceiveSize_ > MAX_RECV_BUFFER_SIZE)
        {
            std::__throw_length_error("Length of message to receive from client would overflow reception buffer maximum capacity. Reception refused.");
        }

        while (numOfPayloadReadBytes < bufferToReceiveSize_)
        {
            numOfPayloadReadBytes = recv(clientSocketDescriptor_, bufferToReceive_, bufferToReceiveSize_, MSG_DONTWAIT);
            if (numOfPayloadReadBytes < 0)
            {
                std::__throw_runtime_error("Data reception from client failed.");
            }
            else if (numOfPayloadReadBytes == 0)
            {
                // Set flag indicating client connection end
                cout << "Client disconnected." << endl; // What? really needed? Shouldn't this be overriden by any other code pathway?
                isClientConnected = false;
            }
            else
            { // Increase counter of received bytes
                totalPayloadBytesReceived += numOfPayloadReadBytes;
            };
        }

        lastMessageSize_ = int(totalPayloadBytesReceived); // Really needed?
        cout << "\tReception completed. Received number of payload bytes: " << lastMessageSize_ << endl;
    }
    else
    {
        cout << "No open connection to clients. Reading operation skipped." << endl;
    }
}

/**
 * @brief
 * @author Pietro Califano
 * @date 
 * // TODO (PC) Make this function generic
 */
void tcpServer::WriteBufferData()
{
    // Receive data from client
    size_t numOfWrittenBytes = 0.0;

    if (isClientConnected)
    {
        bool isValid = true;
        int expectedMsgSize = 0;

        if (commMode != CommModeType::PROTOBUF)
        {
            // Select expected size of the message for validation
            switch (commMode)
            {
            case CommModeType::HARDWIRED_ID_EXAMPLE:
            { // NOTE: Expected size of message to send for GOTOPOSE is identical to READPOSE
                //expectedMsgSize = (int)HARDWIRED_MSG_SIZE::READPOSE;
                break;
            }
            }
            // Check if bufferToSendSize_ matches expected size
            isValid = ValidateWriteBufferLength(expectedMsgSize);
            cout << "\tWriting buffer to client with total size " << bufferToSendSize_;

            numOfWrittenBytes = write(clientSocketDescriptor_, bufferToSend_, bufferToSendSize_);
            if (numOfWrittenBytes < 0)
            {
                std::__throw_runtime_error("Socket has failed while writing data to client. Execution stop.");
            };

            cout << "... Transmission: OK. " << endl;
        }
        else
        {
            cout << "No open connection to clients. Writing operation skipped." << endl;
        }
    }
};

/**
 * @brief
 * @author Pietro Califano
 * @date 2024-06-19
 */
// DEVNOTE: Not sure this is required by generic implementation
//void tcpServer::CleanTempData()
//{
//    cout << "Cleaning temporary data after request handling..." << endl;
//    // Clear temp data by resetting to default values
//    lastMessageSize_ = 0;
//    commMode = CommModeType::PROTOBUF;
//    bufferToReceiveSize_ = 0;
//    bufferToSendSize_ = 0;
//    memset(bufferToReceive_, 0, MAX_RECV_BUFFER_SIZE);
//    memset(bufferToSend_, 0, MAX_SEND_BUFFER_SIZE);
//}