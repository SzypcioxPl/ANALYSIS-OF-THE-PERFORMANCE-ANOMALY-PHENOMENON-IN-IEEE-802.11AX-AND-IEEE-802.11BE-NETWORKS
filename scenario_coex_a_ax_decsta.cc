/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#include <tuple>
#include <vector>
#include <sstream>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"
#include <cmath>
#include "../helpers/populate-arp.h"
#include "../helpers/airtime-logger.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WifiCoexistenceExample");

int
main (int argc, char *argv[])
{
    uint32_t beMaxAmpdu = 0;
    double simulationTime = 260.0;   // seconds
    double clientInterval = 0.0001;  // seconds

    CommandLine cmd;
    cmd.AddValue ("beMaxAmpdu", "Maximum A-MPDU size for BE traffic (bytes, 0 disables aggregation)", beMaxAmpdu);
    cmd.AddValue ("simulationTime", "Total simulation time (s)", simulationTime);
    cmd.AddValue ("clientInterval", "UDP client packet interval (s)", clientInterval);
    cmd.Parse (argc, argv);

    Config::SetDefault ("ns3::WifiMac::BE_MaxAmpduSize", UintegerValue (beMaxAmpdu));

    const uint32_t axStaCount = 10;
    ns3::ShowProgress sp (Seconds (5));
    AirtimeLogger airtimeLogger;

    NodeContainer wifiApNodes;
    wifiApNodes.Create (2);
    NodeContainer wifiStaNodes;
    wifiStaNodes.Create (1 + axStaCount);

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
    channel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel",
                                "Exponent", DoubleValue (1.0),
                                "ReferenceLoss", DoubleValue (0.0));
    Ptr<YansWifiChannel> sharedChannel = channel.Create ();

    YansWifiPhyHelper phyLegacy;
    YansWifiPhyHelper phyAx;
    phyLegacy.SetChannel (sharedChannel);
    phyAx.SetChannel (sharedChannel);
    phyLegacy.Set ("ChannelSettings", StringValue ("{36, 20, BAND_5GHZ, 0}"));
    phyAx.Set ("ChannelSettings", StringValue ("{36, 20, BAND_5GHZ, 0}"));

    WifiHelper wifiLegacy;
    WifiHelper wifiAx;
    wifiLegacy.SetStandard (WIFI_STANDARD_80211a);
    wifiAx.SetStandard (WIFI_STANDARD_80211ax);
    wifiLegacy.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                        "DataMode", StringValue ("OfdmRate6Mbps"),
                                        "ControlMode", StringValue ("OfdmRate6Mbps"));
    wifiAx.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                    "DataMode", StringValue ("HeMcs11"),
                                    "ControlMode", StringValue ("HeMcs0"));

    WifiMacHelper mac;
    Ssid ssidLegacy = Ssid ("network-80211a");
    Ssid ssidAx = Ssid ("network-80211ax");

    mac.SetType ("ns3::StaWifiMac",
                 "Ssid", SsidValue (ssidLegacy),
                 "ActiveProbing", BooleanValue (true));
    NetDeviceContainer staDeviceLegacy = wifiLegacy.Install (phyLegacy, mac, wifiStaNodes.Get (0));
    airtimeLogger.TrackDevices(staDeviceLegacy, "staDeviceLegacy");

    mac.SetType ("ns3::ApWifiMac",
                 "Ssid", SsidValue (ssidLegacy),
                 "EnableBeaconJitter", BooleanValue (false));
    NetDeviceContainer apDeviceLegacy = wifiLegacy.Install (phyLegacy, mac, wifiApNodes.Get (0));

    mac.SetType ("ns3::ApWifiMac",
                 "Ssid", SsidValue (ssidAx),
                 "EnableBeaconJitter", BooleanValue (false));
    NetDeviceContainer apDeviceAx = wifiAx.Install (phyAx, mac, wifiApNodes.Get (1));

    mac.SetType ("ns3::StaWifiMac",
                 "Ssid", SsidValue (ssidAx),
                 "ActiveProbing", BooleanValue (true));
    NetDeviceContainer staDevicesAx;
    for (uint32_t i = 0; i < axStaCount; ++i)
    {
        NetDeviceContainer staDevice = wifiAx.Install (phyAx, mac, wifiStaNodes.Get (1 + i));
        staDevicesAx.Add (staDevice);
    }
    airtimeLogger.TrackDevices(staDevicesAx, "staDevicesAx");


    InternetStackHelper stack;
    stack.Install (wifiApNodes);
    stack.Install (wifiStaNodes);

    Ipv4AddressHelper address;
    address.SetBase ("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterfaceLegacy = address.Assign (apDeviceLegacy);
    Ipv4InterfaceContainer staInterfaceLegacy = address.Assign (staDeviceLegacy);

    address.SetBase ("10.2.1.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterfaceAx = address.Assign (apDeviceAx);
    Ipv4InterfaceContainer staInterfacesAx = address.Assign (staDevicesAx);
    (void) staInterfaceLegacy;
    (void) staInterfacesAx;

    PopulateArpCache ();

    // --- MOBILITY: AP-y w ustalonych pozycjach, STAs na okręgu r=5 m wokół odpowiednich AP
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
    const Vector apLegacyPos (0.0, 0.0, 0.0);
    const Vector apAxPos (0.0, 0.0, 0.0);
    positionAlloc->Add (apLegacyPos);
    positionAlloc->Add (apAxPos);

    const double r = 5.0;
    const double twoPi = 6.283185307179586;   // 2*pi
    const double theta0 = 0.39269908169872414; // pi/8
    positionAlloc->Add (Vector (apLegacyPos.x + r, apLegacyPos.y, apLegacyPos.z));

    for (uint32_t i = 0; i < axStaCount; ++i)
    {
        double theta = theta0 + twoPi * static_cast<double> (i) / static_cast<double> (axStaCount);
        double x = apAxPos.x + r * std::cos (theta);
        double y = apAxPos.y + r * std::sin (theta);
        positionAlloc->Add (Vector (x, y, apAxPos.z));
    }

    mobility.SetPositionAllocator (positionAlloc);
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (wifiApNodes);
    mobility.Install (wifiStaNodes);

    AnimationInterface anim ("scratch/netanim/scenario_coex_a_ax_decsta.xml");
    anim.EnablePacketMetadata (true);
    anim.SetMobilityPollInterval (Seconds (0.25));

    uint32_t apLegacyId = wifiApNodes.Get (0)->GetId ();
    uint32_t apAxId = wifiApNodes.Get (1)->GetId ();
    uint32_t staLegacyId = wifiStaNodes.Get (0)->GetId ();

    anim.SetConstantPosition (wifiApNodes.Get (0), apLegacyPos.x, apLegacyPos.y);
    anim.SetConstantPosition (wifiApNodes.Get (1), apAxPos.x, apAxPos.y);
    anim.SetConstantPosition (wifiStaNodes.Get (0), apLegacyPos.x + r, apLegacyPos.y);

    anim.UpdateNodeDescription (apLegacyId, "AP-802.11a");
    anim.UpdateNodeDescription (apAxId, "AP-802.11ax");
    anim.UpdateNodeDescription (staLegacyId, "STA-802.11a");

    anim.UpdateNodeColor (apLegacyId, 220, 20, 60);
    anim.UpdateNodeColor (staLegacyId, 255, 99, 71);
    anim.UpdateNodeColor (apAxId, 65, 105, 225);

    const std::vector<std::tuple<uint8_t, uint8_t, uint8_t>> axColors = {
        {135, 206, 250}, {70, 130, 180}, {25, 25, 112}, {0, 191, 255},
        {30, 144, 255}, {123, 104, 238}, {72, 61, 139}, {0, 0, 205},
        {95, 158, 160}, {176, 224, 230}
    };

    for (uint32_t i = 0; i < axStaCount; ++i)
    {
        Ptr<Node> staNode = wifiStaNodes.Get (1 + i);
        double theta = theta0 + twoPi * static_cast<double> (i) / static_cast<double> (axStaCount);
        double x = apAxPos.x + r * std::cos (theta);
        double y = apAxPos.y + r * std::sin (theta);
        anim.SetConstantPosition (staNode, x, y);

        std::ostringstream oss;
        oss << "STA-802.11ax #" << (i + 1);
        anim.UpdateNodeDescription (staNode->GetId (), oss.str ());

        auto [r, g, b] = axColors[i % axColors.size ()];
        anim.UpdateNodeColor (staNode->GetId (), r, g, b);
    }

    uint16_t portLegacy = 9000;
    std::vector<uint16_t> axPorts;
    axPorts.reserve (axStaCount);
    for (uint16_t basePort = 9001; basePort < 9001 + axStaCount; ++basePort)
    {
        axPorts.push_back (basePort);
    }

    UdpServerHelper udpServerLegacy (portLegacy);
    ApplicationContainer serverAppLegacy = udpServerLegacy.Install (wifiApNodes.Get (0));
    serverAppLegacy.Start (Seconds (0.0));
    serverAppLegacy.Stop (Seconds (simulationTime + 1.0));

    UdpClientHelper udpClientLegacy (apInterfaceLegacy.GetAddress (0), portLegacy);
    udpClientLegacy.SetAttribute ("MaxPackets", UintegerValue (4294967295u));
    udpClientLegacy.SetAttribute ("Interval", TimeValue (Seconds (clientInterval)));
    udpClientLegacy.SetAttribute ("PacketSize", UintegerValue (1472));
    ApplicationContainer clientAppLegacy = udpClientLegacy.Install (wifiStaNodes.Get (0));
    clientAppLegacy.Start (Seconds (1.0));
    clientAppLegacy.Stop (Seconds (simulationTime + 1.0));

    std::vector<ApplicationContainer> axServerApps;
    std::vector<ApplicationContainer> axClientApps;
    axServerApps.reserve (axStaCount);
    axClientApps.reserve (axStaCount);

    for (uint32_t i = 0; i < axStaCount; ++i)
    {
        uint16_t port = axPorts[i];

        UdpServerHelper serverHelper (port);
        ApplicationContainer serverApp = serverHelper.Install (wifiApNodes.Get (1));
        serverApp.Start (Seconds (0.0));
        serverApp.Stop (Seconds (simulationTime + 1.0));
        axServerApps.push_back (serverApp);

        UdpClientHelper clientHelper (apInterfaceAx.GetAddress (0), port);
        clientHelper.SetAttribute ("MaxPackets", UintegerValue (4294967295u));
        clientHelper.SetAttribute ("Interval", TimeValue (Seconds (clientInterval)));
        clientHelper.SetAttribute ("PacketSize", UintegerValue (1472));
        ApplicationContainer clientApp = clientHelper.Install (wifiStaNodes.Get (1 + i));
        clientApp.Start (Seconds (1.0));
        clientApp.Stop (Seconds (simulationTime + 1.0));
        axClientApps.push_back (clientApp);
    }

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll ();

    Simulator::Stop (Seconds (simulationTime + 1.5));
    Simulator::Run ();

    monitor->CheckForLostPackets ();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();

    double throughputLegacy = 0.0;
    double avgDelayLegacy = 0.0;
    double avgJitterLegacy = 0.0;

    std::vector<double> throughputAx (axStaCount, 0.0);
    std::vector<double> avgDelayAx (axStaCount, 0.0);
    std::vector<double> avgJitterAx (axStaCount, 0.0);

    for (auto iter = stats.begin (); iter != stats.end (); ++iter)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (iter->first);
        if (t.destinationPort == portLegacy)
        {
            uint32_t rx = iter->second.rxPackets;
            if (rx > 0)
            {
                throughputLegacy = (iter->second.rxBytes * 8.0) / (simulationTime * 1e6);
                avgDelayLegacy = iter->second.delaySum.GetSeconds () / rx;
                if (rx > 1)
                {
                    avgJitterLegacy = iter->second.jitterSum.GetSeconds () / (rx - 1);
                }
            }
            continue;
        }

        for (uint32_t i = 0; i < axStaCount; ++i)
        {
            if (t.destinationPort == axPorts[i])
            {
                uint32_t rx = iter->second.rxPackets;
                if (rx > 0)
                {
                    throughputAx[i] = (iter->second.rxBytes * 8.0) / (simulationTime * 1e6);
                    avgDelayAx[i] = iter->second.delaySum.GetSeconds () / rx;
                    if (rx > 1)
                    {
                        avgJitterAx[i] = iter->second.jitterSum.GetSeconds () / (rx - 1);
                    }
                }
            }
        }
    }

    std::cout << "Results after " << simulationTime << " seconds of simulation:" << std::endl;
    std::cout << "802.11a network - Throughput: " << throughputLegacy << " Mbit/s"
              << ", Average delay: " << (avgDelayLegacy * 1000) << " ms"
              << ", Average jitter: " << (avgJitterLegacy * 1000) << " ms" << std::endl;

    for (uint32_t i = 0; i < axStaCount; ++i)
    {
        std::cout << "802.11ax STA #" << (i + 1)
                  << " - Throughput: " << throughputAx[i] << " Mbit/s"
                  << ", Average delay: " << (avgDelayAx[i] * 1000) << " ms"
                  << ", Average jitter: " << (avgJitterAx[i] * 1000) << " ms" << std::endl;
    }

    airtimeLogger.PrintSummary(simulationTime);
    monitor->SerializeToXmlFile ("scratch/flowmon/scenario_coex_a_ax_decsta.flowmon", true, true);
    Simulator::Destroy ();
    return 0;
}
