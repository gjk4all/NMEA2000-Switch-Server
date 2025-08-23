#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <errno.h>
#include <microhttpd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// Initial NMEA2000 values
#define N2K_PRIO	3	/* Priority of the messages on the network */
#define N2K_ID		112	/* Initial NMEA2K sender id */
#define RELAY_BOARD_ID	128	/* Switch bank id (0-255), Only top 4 bits count, lower 4 are set on the command line */
#define RELAY_CHANNELS	8	/* Number of relays on this board */

// Initial ISO NAME parameters
#define ISO_IDENT       0x1FFFF	/* AKA serial number used in address claim */
#define MFG_CODE	666	/* Manufacturer code (0-2048) 666 for homemade */
#define DEV_INST	0	/* Device instance */
#define DEV_FUNCT	135	/* Switch Interface */
#define DEV_CLASS	110	/* Human Interface */
#define SYS_INST	0	/* System instance */
#define IND_GROUP	4	/* Industry group = 4 (Marine Industry) */
#define ARB_ADDRESS	1	/* Arbitrary address capable */
#define ISO_MAX_RETRIES	8	/* Maximum number of address claim retries */

// NMEA2K Link States
#define LINK_STATE_IDLE		0
#define LINK_STATE_CLAIMED	1
#define LINK_STATE_ACTIVE	2
#define LINK_STATE_ABORT	99

// TCP constants
#define TCP_PORT        8088

// Compare macro
#define compare(a, b) (((a) < (b)) ? -1 : (((a) > (b)) ? 1 : 0))

// NMEA2K  message frasme
struct NMEA2000_FRAME {
    uint8_t     priority;
    uint32_t    pgn;
    uint8_t     senderId;
    uint8_t     dataLength;
    uint8_t     data[8];
};

// NMEA2K network operational NAME parameters
struct NMEA2K_NETWORK {
    uint8_t     senderId;
    uint32_t    identityNumber;
    uint16_t    manufacturerCode;
    uint8_t	deviceInstance;
    uint8_t	deviceFunction;
    uint8_t	deviceClass;
    uint8_t	systemInstance;
    uint8_t	industryGroup;
    uint8_t	linkState;
    uint8_t	linkRetries;
};


void cleanup();
void nmea2k_receive(struct can_frame *frame);
void n2k_127501_handler(struct NMEA2000_FRAME *n2kFrame);
void n2k_60928_handler(struct NMEA2000_FRAME *n2kFrame);
int Compare_NameWeight(uint8_t *data);
void send_n2k_60928_message();
enum MHD_Result answer_to_connection (void *cls, struct MHD_Connection *connection,
                                      const char *url,
                                      const char *method, const char *version,
                                      const char *upload_data,
                                      size_t *upload_data_size, void **req_cls);


int canSocket;
int pid;
uint8_t relayId = RELAY_BOARD_ID;
uint32_t switchStatus;
struct NMEA2K_NETWORK nmea2kNetwork;
struct MHD_Daemon *mh_daemon;


int main(int argc, char *argv[]) {
    int ret = 0, nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    clock_t time, timer;

    // Define exit cleanup function
    atexit(cleanup);

    // Populate initial N2K network structure   
    nmea2kNetwork.senderId = N2K_ID;
    nmea2kNetwork.identityNumber = ISO_IDENT;
    nmea2kNetwork.manufacturerCode = MFG_CODE;
    nmea2kNetwork.deviceInstance = DEV_INST;
    nmea2kNetwork.deviceFunction = DEV_FUNCT;
    nmea2kNetwork.deviceClass = DEV_CLASS;
    nmea2kNetwork.systemInstance = SYS_INST;
    nmea2kNetwork.industryGroup = IND_GROUP;
    nmea2kNetwork.linkState = LINK_STATE_IDLE;
    nmea2kNetwork.linkRetries = 0;


    // Open a socket for the Waveshare RS485 CAN HAT (HW dependant)
    canSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (canSocket < 0) {
        perror("socket PF_CAN failed");
        return 1;
    }

    // Connect to if can0 (HW dependant)
    strcpy(ifr.ifr_name, "can0");
    ret = ioctl(canSocket, SIOCGIFINDEX, &ifr);
    if (ret < 0) {
        perror("ioctl failed");
        return 1;
    }

    // Bind the socket to can0 (HW dependant)
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    ret = bind(canSocket, (struct sockaddr *)&addr, sizeof(addr));
    if (ret < 0) {
        perror("bind failed");
        return 1;
    }

    // Set O_NONBLOCK for the receive
    fcntl(canSocket, F_SETFL, O_NONBLOCK);

    // Build a CAN filter to listen to apropiate messages
    struct can_filter pgn_filter[2];

    // Filter for the ISO Protocol messages
    pgn_filter[0].can_id = 0x0E800 << 8;
    pgn_filter[0].can_mask = CAN_EFF_MASK & 0x01F80000U;

    // Filter for the NMEA2000 Binary Switch Bank Status messages
    pgn_filter[1].can_id = 127501U << 8;
    pgn_filter[1].can_mask = CAN_EFF_MASK & 0x01FFFF00U;

    // Install the filters
    setsockopt(canSocket, SOL_CAN_RAW, CAN_RAW_FILTER, &pgn_filter, sizeof(pgn_filter));

    // Main loop
    while (1) {
        // Get system time (clicks) for second report interval
        time = clock();

        // Check NMEA2000 network status
        if (nmea2kNetwork.linkState == LINK_STATE_IDLE) {
            // Check retries
            if (nmea2kNetwork.linkRetries > ISO_MAX_RETRIES) {
                perror("No free NMEA2000 address found!");
                return 1;
            }

            // Send network claim
            send_n2k_60928_message();
            nmea2kNetwork.linkState = LINK_STATE_CLAIMED;
            timer = time + (CLOCKS_PER_SEC / 4);    // 250ms delay
        }

        // Handle the network claim timer
        if ((nmea2kNetwork.linkState == LINK_STATE_CLAIMED) && (time > timer)) {
            nmea2kNetwork.linkState = LINK_STATE_ACTIVE;
            nmea2kNetwork.linkRetries = 0;
            mh_daemon = MHD_start_daemon (MHD_USE_INTERNAL_POLLING_THREAD, TCP_PORT, NULL, NULL,
                             &answer_to_connection, NULL, MHD_OPTION_END);
            if (mh_daemon == NULL) {
                perror("Error starting microhttpd daemon!");
                return 3;
            }
        }

        // Check the CAN receive buffer
        if ((nbytes = read(canSocket, &frame, sizeof(frame))) > 0) {
            nmea2k_receive(&frame);
        }
    }

    // Close the socket
    close(canSocket);

    return 0;
}

void cleanup() {
    MHD_stop_daemon (mh_daemon);
}

void nmea2k_receive(struct can_frame *frame) {
    struct NMEA2000_FRAME n2kFrame;

    if (!(frame->can_id & 0x80000000))
        return;     // Not an extended ID, so not a NMEA2000 message

    // Create the N2K frame
    n2kFrame.priority   = (frame->can_id & 0x1C000000) >> 26;
    n2kFrame.pgn        = (frame->can_id & 0x01FFFF00) >> 8;
    n2kFrame.senderId   = (frame->can_id & 0x000000FF);
    n2kFrame.dataLength = frame->len;
    memcpy(n2kFrame.data, frame->data, frame->len);

    // Check for a Binary Switch Bank Status message
    if (n2kFrame.pgn == 127501U) {
        n2k_127501_handler(&n2kFrame);
    }

    // Check for a ISO Address Claim message
    if ((n2kFrame.pgn & 0x0001FF00 == 60928U)) {
        n2k_60928_handler(&n2kFrame);
    }
}

// Handle Binary Switch Bank Status messages
void n2k_127501_handler(struct NMEA2000_FRAME *n2kFrame) {
    int index, offset;

    if (n2kFrame->data[0] != relayId)
        return;

    for (int i = 0; i < RELAY_CHANNELS; i++) {
        index  = i / 4 + 1;
        offset = (3 - (i % 4)) * 2;
        if ((n2kFrame->data[index] >> offset) & 0x01)
            switchStatus |= 1 << i;
        else
            switchStatus &= ~(1 << i);
    }
}

void n2k_60928_handler(struct NMEA2000_FRAME *n2kFrame) {
    // Check if message is ment fore us
    if (n2kFrame->senderId != nmea2kNetwork.senderId)
        return;

    if (nmea2kNetwork.linkState == LINK_STATE_CLAIMED) {
        nmea2kNetwork.linkState = LINK_STATE_IDLE;
        nmea2kNetwork.linkRetries += 1;
        nmea2kNetwork.senderId += nmea2kNetwork.linkRetries;
    }

    if (nmea2kNetwork.linkState == LINK_STATE_ACTIVE) {
        if (!(n2kFrame->data[7] & 0x01) || (Compare_NameWeight(n2kFrame->data) == 1)) {
            nmea2kNetwork.linkState = LINK_STATE_IDLE;
            nmea2kNetwork.linkRetries += 1;
            nmea2kNetwork.senderId += nmea2kNetwork.linkRetries;
        }
        else {
            if (Compare_NameWeight(n2kFrame->data) == 0)
                nmea2kNetwork.deviceInstance += 1;
            send_n2k_60928_message();
        }
    }
}

int Compare_NameWeight(uint8_t *data) {
	int ret;

	// compare is defined as a macro
	if ((ret = compare(data[0], ((nmea2kNetwork.identityNumber >> 13) & 0xFF))) != 0)
		return ret;
	if ((ret = compare(data[1], ((nmea2kNetwork.identityNumber >> 5) & 0xFF))) != 0)
		return ret;
	if ((ret = compare(data[2], (((nmea2kNetwork.identityNumber << 3) & 0xF8) | ((nmea2kNetwork.manufacturerCode >> 8) & 0x07)))) != 0)
		return ret;
	if ((ret = compare(data[3], (nmea2kNetwork.manufacturerCode & 0xFF))) != 0)
		return ret;
	if ((ret = compare(data[4], nmea2kNetwork.deviceInstance)) != 0)
		return ret;
	if ((ret = compare(data[5], nmea2kNetwork.deviceFunction)) != 0)
		return ret;
	if ((ret = compare(data[6], (nmea2kNetwork.deviceClass & 0x7F))) != 0)
		return ret;
	return compare(data[7], (((nmea2kNetwork.systemInstance << 4) & 0xF0) | ((nmea2kNetwork.industryGroup << 1) & 0x0E) | 0x01));
}

void send_n2k_60928_message() {
    int nbytes;
    struct can_frame frame;

    frame.can_id = (1 << 31) | (6 << 26) | (60928 << 8) | (0xFF << 8) | nmea2kNetwork.senderId;
    frame.len    = 8;

    frame.data[0] = (nmea2kNetwork.identityNumber >> 13) & 0xFF;
    frame.data[1] = (nmea2kNetwork.identityNumber >> 5) & 0xFF;
    frame.data[2] = ((nmea2kNetwork.identityNumber << 3) & 0xF8) | ((nmea2kNetwork.manufacturerCode >> 8) & 0x07);
    frame.data[3] = nmea2kNetwork.manufacturerCode & 0xFF;
    frame.data[4] = nmea2kNetwork.deviceInstance;
    frame.data[5] = nmea2kNetwork.deviceFunction;
    frame.data[6] = nmea2kNetwork.deviceClass & 0x7F;
    frame.data[7] = ((nmea2kNetwork.systemInstance << 4) & 0xF0) | ((nmea2kNetwork.industryGroup << 1) & 0x0E) | 0x0;

    if ((nbytes = write(canSocket, &frame, sizeof(frame))) != sizeof(frame)) {
        perror("NMEA2000 transmit error!");
        exit(2);
    }
}

enum MHD_Result answer_to_connection (void *cls, struct MHD_Connection *connection,
                                      const char *url,
                                      const char *method, const char *version,
                                      const char *upload_data,
                                      size_t *upload_data_size, void **req_cls)
{
    char page[255];
    struct MHD_Response *response;
    int ret;

    sprintf(page, "<html><body>Hello, browser!<br>%d</body></html>", switchStatus);
    response = MHD_create_response_from_buffer (strlen (page), (void*) page, MHD_RESPMEM_PERSISTENT);

    ret = MHD_queue_response (connection, MHD_HTTP_OK, response);
    MHD_destroy_response (response);

    return ret;
}
