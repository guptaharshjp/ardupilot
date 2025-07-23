#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/socket.h>

// Include generated MAVLink headers
#include "../mavlink/generated/custom/custom/mavlink.h"

int main() {
    int sock;
    struct sockaddr_in gcAddr;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t msg;
    uint16_t len;

    // Create UDP socket
    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        perror("socket creation failed");
        return 1;
    }

    // Set destination address (usually GCS on 127.0.0.1:14550)
    memset(&gcAddr, 0, sizeof(gcAddr));
    gcAddr.sin_family = AF_INET;
    gcAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    gcAddr.sin_port = htons(14550);

    // Send MY_TEST_MESSAGE repeatedly
    while (1) {
        // Pack custom MAVLink message
        mavlink_msg_my_test_message_pack(
            1,    // system ID
            200,  // component ID
            &msg,
            42,       // id
            3.1415f   // value
        );

        // Serialize and send
        len = mavlink_msg_to_send_buffer(buf, &msg);
        sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(gcAddr));

        // Print debug output
        printf("Sent MY_TEST_MESSAGE { id=42, value=3.1415 }\n");

        usleep(1000000); // Wait 1 second
    }

    close(sock);
    return 0;
}
