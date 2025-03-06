#ifndef MAVLINK_APPLICATION_H
#define MAVLINK_APPLICATION_H

#include "c_library_v2/common/mavlink.h"
#include "c_library_v2/minimal/mavlink_msg_heartbeat.h"

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"

#include <random>
#include <unordered_map>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("MAVLinkApplication");

double probablity = 0.999;
int packetSize = 512;

double
randomNumber()
{
    std::random_device rd;
    std::mt19937 generator(rd());

    std::uniform_real_distribution<double> dis(0.0, 1.0);

    double random_number = dis(generator);
    return random_number;
}

int
randomInteger(int min, int max)
{
    std::random_device rd;
    std::mt19937 generator(rd());

    std::uniform_int_distribution<> dis(min, max);

    int random_integer = dis(generator);
    return random_integer;
}

// Custom Application for sending MAVLink heartbeat messages
class droneApplication : public Application
{
  public:
    droneApplication()
    {
    }

    void Setup(Address address, uint16_t port)
    {
        m_peerAddress = address;
        m_peerPort = port;
    }

  protected:
    virtual void StartApplication() override
    {
        // Singnal monitor
        Config::ConnectWithoutContext(
            "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",
            MakeCallback(&droneApplication::RxCallback, this));

        m_socket = Socket::CreateSocket(GetNode(), TcpSocketFactory::GetTypeId());
        InetSocketAddress local =
            InetSocketAddress(InetSocketAddress::ConvertFrom(m_peerAddress).GetIpv4(), m_peerPort);
        m_socket->Connect(local);

        // Schedule the first heartbeat
        Simulator::Schedule(Seconds(1.0), &droneApplication::SendHeartbeat, this);
    }

    virtual void StopApplication() override
    {
        if (m_socket)
        {
            m_socket->Close();
        }
    }

  private:
    void RxCallback(Ptr<const Packet> packet,
                    uint16_t channelFreqMhz,
                    WifiTxVector txVector,
                    MpduInfo aMpdu,
                    SignalNoiseDbm signalNoise,
                    uint16_t staId)
    {
        WifiMacHeader macHeader;

        // Check if the packet contains a valid MAC header
        if (packet->PeekHeader(macHeader))
        {
            if (macHeader.IsData())
            {
                Ptr<Node> node = GetNode();
                uint32_t device_id = node->GetId();

                // Retrieve the MAC address of the node's Wi-Fi interface
                Ptr<NetDevice> device = node->GetDevice(0); // Assuming the first device is Wi-Fi
                Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice>(device);

                Mac48Address nodeMac = wifiDevice->GetMac()->GetAddress();

                // Filter: Process only packets addressed to this node
                if (macHeader.GetAddr1() == nodeMac)
                {
                    double rssi = signalNoise.signal; // Signal strength in dBm
                    double noise = signalNoise.noise; // Noise floor in dBm
                    double snr = rssi - noise;        // Signal-to-noise ratio
                    Vector currPosition = getPosition();

                    // NS_LOG_INFO("device_type: Mobile Devices, "
                    //             << "timestamp: " << Simulator::Now().GetSeconds() << ", "
                    //             << "event: signal, "
                    //             << "device_id: " << device_id << ", "
                    //             << "position: " << currPosition << ", "
                    //             << "RSSI: " << rssi << ", "
                    //             << "SNR: " << snr);
                }
            }
        }
    }

    Vector getPosition()
    {
        Ptr<Node> node = GetNode();
        Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
        Vector pos = mob->GetPosition();
        return pos;
    }

    void SendHeartbeat()
    {
        mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

        Ptr<Node> node = GetNode();
        uint32_t device_id = node->GetId();

        mavlink_msg_heartbeat_pack(1,
                                   200,
                                   &msg,
                                   MAV_TYPE_QUADROTOR,
                                   MAV_AUTOPILOT_GENERIC,
                                   MAV_MODE_GUIDED_ARMED,
                                   device_id,
                                   MAV_STATE_ACTIVE);

        uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
        Ptr<Packet> packet = Create<Packet>(buffer, packetSize);

        double rand = randomNumber();
        bool sent = false;
        if (rand < probablity)
        {
            m_socket->Send(packet);
            sent = true;
            Vector currPosition = getPosition();
            NS_LOG_INFO("device_type: Mobile Devices, "
                        << "timestamp: " << Simulator::Now().GetSeconds()
                        << ", "
                        // << "event: send_heartbeat" << ", "
                        << "device_id: " << device_id << ", "
                        << "position: " << currPosition);

            Simulator::Schedule(Seconds(1.0), &droneApplication::SendHeartbeat, this);
        }
        else
        {
            Vector currPosition = getPosition();
            // NS_LOG_INFO("device_type: Mobile Devices, "
            //             << "timestamp: " << Simulator::Now().GetSeconds() << ", "
            //             << "event: failure" << ", "
            //             << "device_id: " << device_id);
        }
    }

    Ptr<Socket> m_socket;
    Address m_peerAddress;
    uint16_t m_peerPort;
};

class apApplication : public Application
{
  public:
    apApplication()
    {
    }

    void Setup(Address address, uint16_t port)
    {
        m_peerAddress = address;
        m_peerPort = port;
    }

  protected:
    virtual void StartApplication() override
    {
        m_socket = Socket::CreateSocket(GetNode(), TcpSocketFactory::GetTypeId());
        InetSocketAddress local =
            InetSocketAddress(InetSocketAddress::ConvertFrom(m_peerAddress).GetIpv4(), m_peerPort);
        m_socket->Connect(local);

        // Schedule the first heartbeat
        Simulator::Schedule(Seconds(1.0), &apApplication::SendHeartbeat, this);
    }

    virtual void StopApplication() override
    {
        if (m_socket)
        {
            m_socket->Close();
        }
    }

  private:
    Vector getPosition()
    {
        Ptr<Node> node = GetNode();
        Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
        Vector pos = mob->GetPosition();
        return pos;
    }

    void SendHeartbeat()
    {
        mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

        Ptr<Node> node = GetNode();
        uint32_t device_id = node->GetId();

        mavlink_msg_heartbeat_pack(1,
                                   200,
                                   &msg,
                                   MAV_TYPE_QUADROTOR,
                                   MAV_AUTOPILOT_GENERIC,
                                   MAV_MODE_GUIDED_ARMED,
                                   device_id,
                                   MAV_STATE_ACTIVE);

        uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
        Ptr<Packet> packet = Create<Packet>(buffer, packetSize);

        double rand = randomNumber();
        bool sent = false;
        if (rand < probablity)
        {
            m_socket->Send(packet);
            sent = true;
            Vector currPosition = getPosition();
            NS_LOG_INFO("device_type: In-situ Devices, "
                        << "timestamp: " << Simulator::Now().GetSeconds()
                        << ", "
                        // << "event: send_heartbeat" << ", "
                        << "device_id: " << device_id);

            Simulator::Schedule(Seconds(1.0), &apApplication::SendHeartbeat, this);
        }
        else
        {
            Vector currPosition = getPosition();
            // NS_LOG_INFO("device_type: In-situ Devices, "
            //             << "timestamp: " << Simulator::Now().GetSeconds() << ", "
            //             << "event: failure" << ", "
            //             << "device_id: " << device_id);
        }
    }

    Ptr<Socket> m_socket;
    Address m_peerAddress;
    uint16_t m_peerPort;
};

class gcsApplication : public Application
{
  public:
    gcsApplication()
    {
    }

    void Setup(uint16_t port, uint32_t device_num)
    {
        m_port = port;
        num = device_num;
        // Initialize the map with drone device IDs and their last received time.
        for (uint32_t id = 1; id <= device_num; ++id)
        {
            m_lastReceivedTime[id] = 1.0;
        }
    }

  protected:
    virtual void StartApplication() override
    {
        m_socket = Socket::CreateSocket(GetNode(), TcpSocketFactory::GetTypeId());
        InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), m_port);
        m_socket->Bind(local);
        m_socket->Listen();

        m_socket->SetAcceptCallback(MakeNullCallback<bool, Ptr<Socket>, const Address&>(),
                                    MakeCallback(&gcsApplication::HandleAccept, this));
        m_socket->SetRecvCallback(MakeCallback(&gcsApplication::HandleRead, this));

        // Schedule the heartbeat check every 1 second
        Simulator::Schedule(Seconds(3.0), &gcsApplication::CheckHeartbeat, this);
    }

    virtual void StopApplication() override
    {
        if (m_socket)
        {
            m_socket->Close();
        }
    }

    void HandleAccept(Ptr<Socket> socket, const Address& from)
    {
        m_connectedSocket = socket;
        m_connectedSocket->SetRecvCallback(MakeCallback(&gcsApplication::HandleRead, this));
    }

    void HandleRead(Ptr<Socket> socket)
    {
        Address from;
        Ptr<Packet> packet;

        while ((packet = socket->RecvFrom(from)))
        {
            if (packet->GetSize() > 0)
            {
                uint8_t buffer[packetSize];
                packet->CopyData(buffer, packet->GetSize());

                mavlink_message_t msg;
                mavlink_status_t status;

                for (uint32_t i = 0; i < packet->GetSize(); ++i)
                {
                    if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status))
                    {
                        if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)
                        {
                            mavlink_heartbeat_t heartbeat;
                            mavlink_msg_heartbeat_decode(&msg, &heartbeat);

                            uint32_t device_id =
                                heartbeat.custom_mode; // Use custom_mode as the device ID

                            // Update the last received time for this device
                            double currentTime = Simulator::Now().GetSeconds();
                            m_lastReceivedTime[device_id] = currentTime;

                            NS_LOG_INFO("device_type: GCS, " << "timestamp: " << currentTime << ", "
                                                             << "event: recv_heartbeat" << ", "
                                                             << "device_id: " << (int)device_id);
                        }
                    }
                }
            }
        }
    }

    void CheckHeartbeat()
    {
        bool disconnection = false;
        int disconnect_nodes = 0;
        double currentTime = Simulator::Now().GetSeconds();
        for (const auto& entry : m_lastReceivedTime)
        {
            uint32_t device_id = entry.first;
            double lastTime = entry.second;

            // If the last received time for the device is more than 5 second ago, warning
            if (lastTime != -1 && currentTime - lastTime >= 5.0)
            {
                disconnection = true;
                m_lastReceivedTime[device_id] = -1;
                NS_LOG_WARN("device_type: GCS, " << "timestamp: " << currentTime << ","
                                                 << "event: check_failure" << ", "
                                                 << (int)device_id);
            }
            if (lastTime == -1 || lastTime == 1)
            {
                disconnect_nodes++;
            }
        }
        // if (!disconnection)
        // {
        //     double currentTime = Simulator::Now().GetSeconds();
        //     NS_LOG_INFO("device_type: GCS, " << "timestamp: " << currentTime << ", "
        //                                      << "event: check_heartbeat" << ", "
        //                                      << "status: success");
        // }

        // if (int(currentTime) == 322)
        //     NS_LOG_INFO(disconnect_nodes);

        // Reschedule the heartbeat check for the next second
        Simulator::Schedule(Seconds(1.0), &gcsApplication::CheckHeartbeat, this);
    }

  private:
    Ptr<Socket> m_socket;
    Ptr<Socket> m_connectedSocket;
    uint16_t m_port;
    uint32_t num;

    // Store the last received time for each device (device_id -> last received time)
    std::unordered_map<uint32_t, double> m_lastReceivedTime;
};

#endif