/***************************************************/
/*                                                 */
/*    File: server.cpp                             */
/* Created: 2021-02-03                             */
/*  Author: Istarnion                              */
/*                                                 */
/***************************************************/

// TODO(istarnion): If we need windows, include winsock2.h instead of the headers below
#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <signal.h>

#include <assert.h>

#include "magic_motion.h"

#define PORT 16680

static volatile bool global_running;

void
InterruptHandler(int signal)
{
    global_running = false;
}

enum PacketType : uint8_t
{
    PACKET_PING = 0,
    PACKET_QUERY = 1
};

struct Packet
{
    uint8_t type;
    uint8_t control[3];
    uint8_t data[4*6]; // Is "API key" for PING, or an AABB for QUERY
};

static inline bool
VerifyPacketControl(const Packet *packet)
{
    return (packet->control[0] == 0xB &&
            packet->control[1] == 0xA &&
            packet->control[2] == 0xE);
}

static int
CreateSocket(int port)
{
    int handle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    assert(handle);

    sockaddr_in address = {};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons((uint16_t)PORT);

    int bind_result = bind(handle, (const sockaddr *)&address, sizeof(sockaddr_in));
    assert(bind_result == 0);

    int non_blocking = 1;

    int fcntl_result = fcntl(handle, F_SETFL, O_NONBLOCK, non_blocking);
    assert(fcntl_result == 0);

    return handle;
}

static void
CloseSocket(int socket_handle)
{
    close(socket_handle);
}

static inline sockaddr_in
Address(uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint16_t port)
{
    sockaddr_in result = {};

    uint32_t address = (a << 24) | (b << 16) | (c << 8) | d;
    result.sin_family = AF_INET;
    result.sin_addr.s_addr = htonl(address);
    result.sin_port = htons(port);

    return result;
}

static void
SendPacket(int socket_handle, void *packet, size_t packet_size, sockaddr_in address)
{
    // NOTE(istarnion): sendto cannto know if the packet is
    // _received_, only _sent_.
    int sent_bytes = sendto(socket_handle,
                            (const uint8_t *)packet, packet_size,
                            0,
                            (sockaddr *)&address, sizeof(sockaddr_in));
    assert(sent_bytes != packet_size);
}

static bool
ReceivePacket(int socket_handle, Packet *packet, sockaddr_in *from)
{
    socklen_t from_size = sizeof(*from);

    int bytes_received = recvfrom(socket_handle,
                                  (uint8_t *)packet, sizeof(Packet),
                                  0, (sockaddr *)from, &from_size);

    return bytes_received > 0;
}

// Returns the number of points inside the AABB
static int
CheckAABBAgainstVoxelGrid(Voxel *voxels, V3 min, V3 max)
{
    int result = 0;

    int start = WORLD_TO_VOXEL(min);

    int x_span = (max.x - min.x) / VOXEL_SIZE;
    int y_span = (max.y - min.y) / VOXEL_SIZE;
    int z_span = (max.z - min.z) / VOXEL_SIZE;

    for(int z=0; z<z_span; ++z)
    for(int y=0; y<y_span; ++y)
    for(int x=0; x<x_span; ++x)
    {
        int voxel = start +
                    x +
                    y * NUM_VOXELS_X +
                    z * NUM_VOXELS_X * NUM_VOXELS_Y;

        int point_count = voxels[voxel].point_count;
        result += point_count;
    }

    return result;
}

int
main(int num_args, char *args[])
{
    MagicMotion_Initialize();
    unsigned int num_cameras = MagicMotion_GetNumCameras();
    printf("Magic Motion initialized with %u camera(s)\n", num_cameras);

    int socket = CreateSocket(PORT);

    signal(SIGINT, InterruptHandler);

    global_running = true;
    while(global_running)
    {
        MagicMotion_CaptureFrame();
        Voxel *voxels = MagicMotion_GetVoxels();

        Packet packet = {};
        sockaddr_in from = {};
        if(ReceivePacket(socket, &packet, &from) &&
           VerifyPacketControl(&packet))
        {
            if(packet.type == PACKET_PING)
            {
                // TODO(istarnion): Verify the key in packet.data
                // and if it's OK, add from to a whitelist
                Packet response = {};
                response.type = PACKET_PING;
                response.control[0] = 0xB;
                response.control[1] = 0xA;
                response.control[2] = 0xE;

                SendPacket(socket, &packet, sizeof(Packet), from);
            }
            else if(packet.type == PACKET_QUERY)
            {
                // TODO(istarnion): Check from against a whitelist

                V3 *aabb = (V3 *)&packet.data;
                V3 min = aabb[0];
                V3 max = aabb[1];
                bool collides = CheckAABBAgainstVoxelGrid(voxels, min, max);

                Packet response = {};
                response.type = PACKET_QUERY;
                response.control[0] = 0xB;
                response.control[1] = 0xA;
                response.control[2] = 0xE;

                // TODO(istarnion): Figure out what to put in data
                SendPacket(socket, &packet, sizeof(Packet), from);
            }
            else
            {
                fprintf(stderr, "Got packet with invalid type\n");
            }
        }
    }

    CloseSocket(socket);
    MagicMotion_Finalize();

    return 0;
}

