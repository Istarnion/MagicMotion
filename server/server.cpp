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
#define WHITELIST_LENGTH 16

static volatile bool global_running;

/// Callback for when/if the process is
/// interrupted (e.g. by SIGINT caused by <Ctrl+C>
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

struct PacketHeader
{
    uint8_t type;
    uint16_t control;
    uint32_t sequence;
    uint8_t data;
} __attribute__((packed)) __attribute__((aligned(1)));

struct PacketAABB
{
    float min[3];
    float max[3];
} __attribute__((packed)) __attribute__((aligned(1)));

static inline bool
VerifyPacketControl(const PacketHeader *header)
{
    return header->control == 0x69; // Nice
}

/// Create a UDP socket for listening on incoming packets.
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

// Create a sockaddr_in struct from the IP address bytes and port number
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
SendPacket(int socket_handle, void *packet, size_t packet_size, sockaddr_in address, int flags)
{
    // NOTE(istarnion): sendto cannto know if the packet is
    // _received_, only _sent_.
    int sent_bytes = sendto(socket_handle,
                            (const uint8_t *)packet, packet_size,
                            flags,
                            (sockaddr *)&address, sizeof(sockaddr_in));
    assert(sent_bytes == packet_size);
}

/// Try to receive a packet. If there is no packet available from
/// the network card, it does not block, but returns false.
/// Else, packet gets filled and the function returns true.
static bool
ReceivePacket(int socket_handle, void *packet, size_t packet_length, sockaddr_in *from)
{
    socklen_t from_size = sizeof(*from);

    int bytes_received = recvfrom(socket_handle,
                                  (uint8_t *)packet, packet_length,
                                  0, (sockaddr *)from, &from_size);

    return bytes_received == packet_length;
}

/// Returns the number of points inside the AABB
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

/// Add address to the whitelist
static inline void
Whitelist(uint32_t *whitelist, const sockaddr_in *address)
{
    uint32_t ip = ntohl(address->sin_addr.s_addr);
    for(int i=0; i<WHITELIST_LENGTH; ++i)
    {
        if(whitelist[i] == ip)
        {
            break;
        }
        else if(whitelist[i] == 0)
        {
            whitelist[i] = ip;
            break;
        }
    }
}

/// Check if address is whitelisted
static inline bool
IsWhitelisted(const uint32_t *whitelist, const sockaddr_in *address)
{
    uint32_t ip = ntohl(address->sin_addr.s_addr);
    bool result = false;
    for(int i=0; i<WHITELIST_LENGTH; ++i)
    {
        if(whitelist[i] == ip)
        {
            result = true;
            break;
        }
    }

    return result;
}

int
main(int num_args, char *args[])
{
    printf("%lu\n", sizeof(PacketHeader));
    MagicMotion_Initialize();
    unsigned int num_cameras = MagicMotion_GetNumCameras();
    printf("Magic Motion initialized with %u camera(s)\n", num_cameras);

    int socket = CreateSocket(PORT);

    signal(SIGINT, InterruptHandler);

    uint32_t whitelist[WHITELIST_LENGTH];
    PacketAABB aabbs[256];
    uint8_t results[256];

    global_running = true;
    while(global_running)
    {
        MagicMotion_CaptureFrame();
        Voxel *voxels = MagicMotion_GetVoxels();

        PacketHeader header = {};
        sockaddr_in from = {};
        while(ReceivePacket(socket, &header, sizeof(PacketHeader), &from))
        {
            if(VerifyPacketControl(&header))
            {
                if(header.type == PACKET_PING)
                {
                    puts("Ping packet");
                    Whitelist(whitelist, &from);
                    // Return the PING packet
                    SendPacket(socket, &header, sizeof(PacketHeader), from, MSG_CONFIRM);
                }
                else if(header.type == PACKET_QUERY)
                {
                    if(IsWhitelisted(whitelist, &from))
                    {
                        int num_aabbs = header.data;
                        if(ReceivePacket(socket, aabbs, sizeof(PacketAABB) * num_aabbs, &from))
                        {
                            printf("Got %d AABBs\n", num_aabbs);
                            for(int i=0; i<num_aabbs; ++i)
                            {
                                V3 min = {
                                    aabbs[i].min[0],
                                    aabbs[i].min[1],
                                    aabbs[i].min[2]
                                };

                                V3 max = {
                                    aabbs[i].max[0],
                                    aabbs[i].max[1],
                                    aabbs[i].max[2]
                                };

                                bool collides = CheckAABBAgainstVoxelGrid(voxels, min, max);
                                results[i] = collides ? 1 : 0;
                            }

                            SendPacket(socket, &header, sizeof(PacketHeader), from, MSG_MORE);
                            SendPacket(socket, results, num_aabbs, from, MSG_CONFIRM);
                        }
                        else
                        {
                            header.data = 0;
                            SendPacket(socket, &header, sizeof(PacketHeader), from, MSG_CONFIRM);
                        }
                    }
                    else
                    {
                        fprintf(stderr, "Got query packet from non-whitelisted IP\n");
                    }
                }
                else
                {
                    fprintf(stderr, "Got packet with invalid type\n");
                }
            }
            else
            {
                fprintf(stderr, "Incoming packet failed control\n");
            }
        }
    }

    CloseSocket(socket);
    MagicMotion_Finalize();

    return 0;
}

