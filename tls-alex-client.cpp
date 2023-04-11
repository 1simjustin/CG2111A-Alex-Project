
// Routines to create a TLS client
#include "headers/make_tls_client.h"

// Network packet types
#include "headers/netconstants.h"

// Packet types, error codes, etc.
#include "Alex/constants.h"

// Tells us that the network is running.
static volatile int networkActive=0;

void handleError(const char *buffer)
{
	switch(buffer[1])
	{
		case RESP_OK:
			printf("Command / Status OK\n");
			break;

		case RESP_BAD_PACKET:
			printf("BAD MAGIC NUMBER FROM ARDUINO\n");
			break;

		case RESP_BAD_CHECKSUM:
			printf("BAD CHECKSUM FROM ARDUINO\n");
			break;

		case RESP_BAD_COMMAND:
			printf("PI SENT BAD COMMAND TO ARDUINO\n");
			break;

		case RESP_BAD_RESPONSE:
			printf("PI GOT BAD RESPONSE FROM ARDUINO\n");
			break;

		default:
			printf("PI IS CONFUSED!\n");
	}
}

void handleStatus(const char *buffer)
{
	int32_t data[16];
	memcpy(data, &buffer[1], sizeof(data));

	printf("\n ------- ALEX STATUS REPORT ------- \n\n");
	printf("Left Forward Ticks:\t\t%d\n", data[0]);
	printf("Right Forward Ticks:\t\t%d\n", data[1]);
	printf("Left Reverse Ticks:\t\t%d\n", data[2]);
	printf("Right Reverse Ticks:\t\t%d\n", data[3]);
	printf("Left Forward Ticks Turns:\t%d\n", data[4]);
	printf("Right Forward Ticks Turns:\t%d\n", data[5]);
	printf("Left Reverse Ticks Turns:\t%d\n", data[6]);
	printf("Right Reverse Ticks Turns:\t%d\n", data[7]);
	printf("Forward Distance:\t\t%d\n", data[8]);
	printf("Reverse Distance:\t\t%d\n", data[9]);
	printf("\n---------------------------------------\n\n");
}

void handleMessage(const char *buffer)
{
	printf("MESSAGE FROM ALEX: %s\n", &buffer[1]);
}

void handleCommand(const char *buffer)
{
	// We don't do anything because we issue commands
	// but we don't get them. Put this here
	// for future expansion
}

void handleNetwork(const char *buffer, int len)
{
	// The first byte is the packet type
	int type = buffer[0];

	switch(type)
	{
		case NET_ERROR_PACKET:
		handleError(buffer);
		break;

		case NET_STATUS_PACKET:
		handleStatus(buffer);
		break;

		case NET_MESSAGE_PACKET:
		handleMessage(buffer);
		break;

		case NET_COMMAND_PACKET:
		handleCommand(buffer);
		break;
	}
}

void sendData(void *conn, const char *buffer, int len)
{
	int c;
	printf("\nSENDING %d BYTES DATA\n\n", len);
	if(networkActive)
	{
		/* TODO: Insert SSL write here to write buffer to network */

		// int sslWrite(void *conn, const char *buffer, int len);
		c = sslWrite(conn, buffer, sizeof(buffer));

		/* END TODO */	
		networkActive = (c > 0);
	}
}

void *readerThread(void *conn)
{
	char buffer[128];
	int len;

	while(networkActive)
	{
		/* TODO: Insert SSL read here into buffer */

        // int sslRead(void *conn, char *buffer, int len);
		len = sslRead(conn, buffer, sizeof(buffer));
		printf("read %d bytes from server.\n", len);
		
		/* END TODO */

		networkActive = (len > 0);

		if(networkActive)
			handleNetwork(buffer, len);
	}

	printf("Exiting network listener thread\n");
    
    /* TODO: Stop the client loop and call EXIT_THREAD */

	EXIT_THREAD(conn);

    /* END TODO */
}

void flushInput()
{
	char c;

	while((c = getchar()) != '\n' && c != EOF);
}

void getParams(int32_t *params)
{
	printf("Enter distance/angle in cm/degrees (e.g. 50) and power in %% (e.g. 75) separated by space.\n");
	printf("E.g. 50 75 means go at 50 cm at 75%% power for forward/backward, or 50 degrees left or right turn at 75%%  power\n");
	scanf("%d %d", &params[0], &params[1]);
	flushInput();
}

void *writerThread(void *conn)
{
	int quit=0;

	while(!quit)
	{
		char ch;
		printf("Command (w=forward, s=reverse, a=left, d=right, e=stop, c=clear stats, g=get stats, i= get color, q=exit)\n");
		scanf("%c", &ch);

		// Purge extraneous characters from input stream
		flushInput();

		char buffer[10];
		int32_t params[2];

		buffer[0] = NET_COMMAND_PACKET;
		switch(ch)
		{
			case 'w':
			case 'a':
			case 's':
			case 'd':
				getParams(params);
				buffer[1] = ch;
				memcpy(&buffer[2], params, sizeof(params));
				sendData(conn, buffer, sizeof(buffer));
				break;

			case 'W':
			case 'S':
				params[0] = 5;
				params[1] = 80;
				buffer[1] = ch;
				memcpy(&buffer[2], params, sizeof(params));
				sendData(conn, buffer, sizeof(buffer));
				break;
			case 'A':
			case 'D':
				params[0] = 10;
				params[1] = 80;
				buffer[1] = ch;
				memcpy(&buffer[2], params, sizeof(params));
				sendData(conn, buffer, sizeof(buffer));
				break;

			case 'e': // e to stop
			case 'E':
			case 'c':
			case 'C':
			case 'g':
			case 'G':
			case 'i':
			case 'I':
				params[0] = 0;
				params[1] = 0;
				memcpy(&buffer[2], params, sizeof(params));
				buffer[1] = ch;
				sendData(conn, buffer, sizeof(buffer));
				break;
			case 'q':
			case 'Q':
				quit=1;
				break;
			default:
				printf("BAD COMMAND\n");
		}
	}

	printf("Exiting keyboard thread\n");

    /* TODO: Stop the client loop and call EXIT_THREAD */

	EXIT_THREAD(conn);

    /* END TODO */
}

/* TODO: #define filenames for the client private key, certificatea,
   CA filename, etc. that you need to create a client */

#define SERVER_NAME 			"172.20.10.11"
#define CA_CERT_FNAME 			"network/signing.pem"
#define PORT_NUM 				5000
#define CLIENT_CERT_FNAME 		"network/laptop.crt"
#define CLIENT_KEY_FNAME 		"network/laptop.key"
#define SERVER_NAME_ON_CERT 	"alex.epp.com"

/* END TODO */
void connectToServer(const char *serverName, int portNum)
{
    /* TODO: Create a new client */

	// void createClient(const char *serverName, const int serverPort, int verifyServer, const char *caCertFname, const char *serverNameOnCert, int sendCert, const char *clientCertFname, const char *clientPrivateKey, void *(*readerThread)(void *), void *(*writerThread)(void *));
	createClient(SERVER_NAME, PORT_NUM, 1, CA_CERT_FNAME, SERVER_NAME_ON_CERT, 1, CLIENT_CERT_FNAME, CLIENT_KEY_FNAME, readerThread, writerThread);

    /* END TODO */
}

int main(int ac, char **av)
{
	if(ac != 3)
	{
		fprintf(stderr, "\n\n%s <IP address> <Port Number>\n\n", av[0]);
		exit(-1);
	}

    networkActive = 1;
    connectToServer(av[1], atoi(av[2]));

    /* TODO: Add in while loop to prevent main from exiting while the
    client loop is running */

	while (client_is_running());

    /* END TODO */
	printf("\nMAIN exiting\n\n");
}
