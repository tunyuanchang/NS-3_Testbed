#include "application.h"

#include "ns3/applications-module.h"
#include "ns3/buildings-module.h"
#include "ns3/core-module.h"
#include "ns3/csma-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"

#include <fstream>

using namespace ns3;

struct Point
{
    double x, y, z;
};

std::pair<std::vector<Point>, std::vector<double>>
readWaypointFile()
{
    Point p;
    std::string line;
    std::vector<Point> wp;
    std::vector<double> height;

    std::ifstream file_wp("scratch/AMPHI/wp.txt");
    if (file_wp.is_open())
    {
        while (std::getline(file_wp, line))
        {
            if (line.empty() || line[0] == '#')
            {
                continue; // Skip empty lines and comments
            }

            std::istringstream ss(line);
            if (ss >> p.x >> p.y)
            {
                wp.push_back(p);
            }
        }
        file_wp.close();
    }

    double h = 0.0;
    std::ifstream file_wp_h("scratch/AMPHI/wp_h.txt");
    if (file_wp_h.is_open())
    {
        while (std::getline(file_wp_h, line))
        {
            if (line.empty() || line[0] == '#')
            {
                continue; // Skip empty lines and comments
            }

            std::istringstream ss(line);
            if (ss >> h)
            {
                height.push_back(h);
            }
        }
        file_wp_h.close();
    }

    return std::make_pair(wp, height);
}

int
main(int argc, char* argv[])
{
    // Variables to store input arguments
    uint32_t gcs_num = 1; // GCS
    uint32_t ap_num = 1;  // In-situ devices
    uint32_t mob_num = 1; // Mobility devices
    int idx = 0;
    double txpower = 50.0;

    LogComponentEnable("MAVLinkApplication", LOG_LEVEL_INFO);

    // Parse command-line arguments
    CommandLine cmd;
    cmd.AddValue("idx", "Index", idx);
    cmd.AddValue("txp", "TXPower", txpower);
    cmd.AddValue("gcs_num", "Number of GCS (always 1)", gcs_num);
    cmd.AddValue("ap_num", "Number of In-situ devices", ap_num);
    cmd.AddValue("mob_num", "Number of Mobile devices (leq 12)", mob_num);
    cmd.Parse(argc, argv);

    std::cout << "Number of GCS: " << gcs_num << ", Number of In-situ devices: " << ap_num
              << ", Number of Mobile devices: " << mob_num << std::endl;

    NodeContainer gcsNode, apNodes, mobNodes;
    gcsNode.Create(gcs_num);
    mobNodes.Create(mob_num);
    apNodes.Create(ap_num);

    NodeContainer csmaNodes;
    csmaNodes.Add(gcsNode);
    csmaNodes.Add(apNodes);

    // Mobility
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gcsNode);
    Ptr<MobilityModel> mobilityModel_gcs = gcsNode.Get(0)->GetObject<MobilityModel>();
    mobilityModel_gcs->SetPosition(Vector(50.0, 40.0, 0.1));

    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "MinX",
                                  DoubleValue(8.0),
                                  "MinY",
                                  DoubleValue(5.0),
                                  "DeltaX",
                                  DoubleValue(20.0),
                                  "DeltaY",
                                  DoubleValue(80.0 * 4 / (ap_num + 1)),
                                  "GridWidth",
                                  UintegerValue(4),
                                  "LayoutType",
                                  StringValue("RowFirst"));

    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(apNodes);

    mobility.SetMobilityModel("ns3::WaypointMobilityModel");
    mobility.Install(mobNodes);

    // Define waypoint schedule to each drone
    std::pair<std::vector<Point>, std::vector<double>> waypoint = readWaypointFile();
    std::vector<Point> waypoint_list = waypoint.first;
    std::vector<double> height = waypoint.second;

    int len = waypoint_list.size();

    Ptr<WaypointMobilityModel> mobilityModel;
    for (uint32_t i = 0; i < mob_num; i++)
    {
        mobilityModel = mobNodes.Get(i)->GetObject<WaypointMobilityModel>();

        double h = height[i];
        double time = 2.0, period = 10.0;

        for (int p = 0; p < len; p++)
        {
            mobilityModel->AddWaypoint(Waypoint(
                Seconds(time),
                Vector(waypoint_list[(p + 3 * i) % len].x, waypoint_list[(p + 3 * i) % len].y, h)));
            time += period;
        }
    }

    // Network
    // Ethernet for GCS & In-situ devices
    CsmaHelper csma;
    csma.SetChannelAttribute("DataRate", StringValue("100Mbps"));
    csma.SetChannelAttribute("Delay", TimeValue(NanoSeconds(2000)));

    NetDeviceContainer csmaDevices = csma.Install(csmaNodes);

    // Configure Wi-Fi for drones
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211g); // 2.4 Hz

    // PropagationLossModel
    YansWifiPhyHelper wifiPhy;
    Ptr<YansWifiChannel> wifiChannel = CreateObject<YansWifiChannel>();
    Ptr<HybridBuildingsPropagationLossModel> lossModel =
        CreateObject<HybridBuildingsPropagationLossModel>();
    Ptr<ConstantSpeedPropagationDelayModel> delayModel =
        CreateObject<ConstantSpeedPropagationDelayModel>();

    wifiChannel->SetPropagationLossModel(lossModel);
    wifiChannel->SetPropagationDelayModel(delayModel);
    wifiPhy.SetChannel(wifiChannel);

    wifiPhy.Set("TxPowerStart", DoubleValue(txpower));
    wifiPhy.Set("TxPowerEnd", DoubleValue(txpower));

    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::StaWifiMac",
                    "Ssid",
                    SsidValue(Ssid("GcsNetwork")),
                    "ActiveProbing",
                    BooleanValue(false));

    NetDeviceContainer droneDevices = wifi.Install(wifiPhy, wifiMac, mobNodes);

    wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(Ssid("GcsNetwork")));
    NetDeviceContainer gcsWifiDevice = wifi.Install(wifiPhy, wifiMac, gcsNode);
    NetDeviceContainer apWifiDevices = wifi.Install(wifiPhy, wifiMac, apNodes);

    InternetStackHelper stack;
    stack.Install(mobNodes);
    stack.Install(apNodes);
    stack.Install(gcsNode);

    // Assign IP addresses
    Ipv4AddressHelper wifiAddress;
    wifiAddress.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer droneInterfaces = wifiAddress.Assign(droneDevices);
    Ipv4InterfaceContainer gcsWifiInterface = wifiAddress.Assign(gcsWifiDevice);
    Ipv4InterfaceContainer apWifiInterface = wifiAddress.Assign(apWifiDevices);

    Ipv4AddressHelper csmaAddress;
    csmaAddress.SetBase("192.168.1.0", "255.255.255.0");
    Ipv4InterfaceContainer EthernetInterface = csmaAddress.Assign(csmaDevices);

    // Enable routing between Wi-Fi and Ethernet
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // Building
    Ptr<Building> building1 = CreateObject<Building>();
    building1->SetBoundaries(Box(0, 32.86, 25.72, 58.58, 0.0, 36.0));
    building1->SetBuildingType(Building::Office);
    building1->SetExtWallsType(Building::ConcreteWithWindows);
    building1->SetNFloors(12);
    building1->SetNRoomsX(3);
    building1->SetNRoomsY(4);

    Ptr<Building> building2 = CreateObject<Building>();
    building2->SetBoundaries(Box(3.53, 68.38, 0, 25.72, 0.0, 36.0));
    building2->SetBuildingType(Building::Office);
    building2->SetExtWallsType(Building::ConcreteWithWindows);
    building2->SetNFloors(12);
    building2->SetNRoomsX(9);
    building2->SetNRoomsY(3);

    BuildingContainer buildings;
    buildings.Add(building1);
    buildings.Add(building2);

    BuildingsHelper::Install(gcsNode); // This will automatically associate MobilityBuildingInfo
    BuildingsHelper::Install(apNodes);
    BuildingsHelper::Install(mobNodes);

    double programStopTime = 10 * len + 3; // Simulation Time

    // Application
    uint16_t port = 14550;

    // Install MAVLink Receiver on GCS
    Ptr<gcsApplication> gcsApp = CreateObject<gcsApplication>();
    gcsApp->Setup(port, ap_num + mob_num);
    gcsNode.Get(0)->AddApplication(gcsApp);
    gcsApp->SetStartTime(Seconds(0.0));
    gcsApp->SetStopTime(Seconds(programStopTime));

    // Install MAVLink Heartbeat Sender on drones
    Ipv4Address gcsAddress = EthernetInterface.GetAddress(0);

    for (uint32_t i = 0; i < ap_num; ++i)
    {
        Ptr<apApplication> apApp = CreateObject<apApplication>();
        apApp->Setup(InetSocketAddress(gcsAddress, port), port);
        apNodes.Get(i)->AddApplication(apApp);
        apApp->SetStartTime(Seconds(1.0));
        apApp->SetStopTime(Seconds(programStopTime));
    }

    for (uint32_t i = 0; i < mob_num; ++i)
    {
        Ptr<droneApplication> droneApp = CreateObject<droneApplication>();
        droneApp->Setup(InetSocketAddress(gcsAddress, port), port);

        mobNodes.Get(i)->AddApplication(droneApp);
        droneApp->SetStartTime(Seconds(1.0));
        droneApp->SetStopTime(Seconds(programStopTime));
    }

    // MonitorSniffRx for RSSI and SNR
    // Config::Connect("/NodeList/*/DeviceList/*/Phy/MonitorSnifferRx", MakeCallback(&RxCallback));

    AnimationInterface anim("scratch/AMPHI/scenarioAnimation.xml");
    for (uint32_t i = 0; i < mob_num; i++)
    {
        anim.UpdateNodeColor(mobNodes.Get(i), 0, 0, 255);
    }
    for (uint32_t i = 0; i < ap_num; i++)
    {
        anim.UpdateNodeColor(apNodes.Get(i), 255, 255, 255);
    }
    anim.SkipPacketTracing();

    // Run the simulation with flow monitor
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Simulator::Stop(Seconds(programStopTime));
    Simulator::Run();

    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    // std::string csvFileName = "/home/tunyuan/delay_" + std::to_string(mob_num + ap_num) + "_" +
    //                           std::to_string(idx) + ".csv";
    // std::ofstream outFile(csvFileName);

    // outFile << "ID,MeanDelay(ms)" << std::endl;
    // for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin();
    //      i != stats.end();
    //      ++i)
    // {
    //     Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
    //     if (i->first > ap_num + mob_num)
    //         break;
    //     outFile << i->first << "," << i->second.delaySum.GetSeconds() * 1000 /
    //     i->second.rxPackets
    //             << "\n";
    //     // std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
    //     // std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
    //     // std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
    //     // std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
    //     // std::cout << "  Throughput: "
    //     //           << i->second.rxBytes * 8.0 /
    //     //                  (i->second.timeLastRxPacket.GetSeconds() -
    //     //                   i->second.timeFirstTxPacket.GetSeconds()) /
    //     //                  1000000
    //     //           << " Mbps\n";
    //     // std::cout << "  Mean Jitter: "
    //     //           << i->second.jitterSum.GetSeconds() / (i->second.rxPackets - 1) << " s\n";
    //     // std::cout << "  Packet Loss Rate: "
    //     //           << 100.0 * (i->second.txPackets - i->second.rxPackets) / i->second.txPackets
    //     //           << "%\n";
    // }

    Simulator::Destroy();
    return 0;
}