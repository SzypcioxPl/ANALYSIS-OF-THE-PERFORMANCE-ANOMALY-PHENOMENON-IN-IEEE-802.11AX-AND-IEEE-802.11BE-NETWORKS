/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#include <vector>
#include <cmath>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/sta-wifi-mac.h"
#include "../helpers/populate-arp.h"
#include "../helpers/airtime-logger.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WifiAllBeScenario");

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

    const uint32_t beStaCount = 3;

    ns3::ShowProgress progress (Seconds (5));
    AirtimeLogger airtimeLogger;

    NodeContainer apNodes;
    apNodes.Create (1);
    NodeContainer staNodes;
    staNodes.Create (beStaCount);

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
    channel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel",
                                "Exponent", DoubleValue (1.0),
                                "ReferenceLoss", DoubleValue (0.0));
    Ptr<YansWifiChannel> sharedChannel = channel.Create ();

    YansWifiPhyHelper phy;
    phy.SetChannel (sharedChannel);
    phy.Set ("ChannelSettings", StringValue ("{36, 20, BAND_5GHZ, 0}"));

    WifiHelper wifi;
    wifi.SetStandard (WIFI_STANDARD_80211be);
    wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                  "DataMode", StringValue ("EhtMcs11"),
                                  "ControlMode", StringValue ("EhtMcs0"));

    WifiMacHelper mac;
    Ssid ssid = Ssid ("network-80211be");

    mac.SetType ("ns3::ApWifiMac",
                 "Ssid", SsidValue (ssid),
                 "EnableBeaconJitter", BooleanValue (false));
    NetDeviceContainer apDevices = wifi.Install (phy, mac, apNodes);

    mac.SetType ("ns3::StaWifiMac",
                 "Ssid", SsidValue (ssid),
                 "ActiveProbing", BooleanValue (true),
                 "AssocType", EnumValue (WifiAssocType::LEGACY));
    NetDeviceContainer staDevices = wifi.Install (phy, mac, staNodes);
    airtimeLogger.TrackDevices(staDevices, "staDevices");

    InternetStackHelper stack;
    stack.Install (apNodes);
    stack.Install (staNodes);

    Ipv4AddressHelper address;
    address.SetBase ("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterface = address.Assign (apDevices);
    Ipv4InterfaceContainer staInterfaces = address.Assign (staDevices);
    (void) staInterfaces; // suppress unused warning if not compiled with flowmon

    PopulateArpCache ();

    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
    positionAlloc->Add (Vector (0.0, 0.0, 0.0)); // AP

    const double r = 2.0;                     // promień okręgu (m)
    const double twoPi = 6.283185307179586;   // 2*pi
    const double theta0 = 0.39269908169872414; // pi/8

    for (uint32_t i = 0; i < beStaCount; ++i)
    {
        double theta = theta0 + twoPi * static_cast<double> (i) / static_cast<double> (beStaCount);
        double x = r * std::cos (theta);
        double y = r * std::sin (theta);
        positionAlloc->Add (Vector (x, y, 0.0));
    }

    mobility.SetPositionAllocator (positionAlloc);
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (apNodes);
    mobility.Install (staNodes);

    std::vector<uint16_t> ports;
    ports.reserve (beStaCount);
    for (uint16_t basePort = 9000; basePort < 9000 + beStaCount; ++basePort)
    {
        ports.push_back (basePort);
    }

    std::vector<ApplicationContainer> serverApps;
    std::vector<ApplicationContainer> clientApps;
    serverApps.reserve (beStaCount);
    clientApps.reserve (beStaCount);

    for (uint32_t i = 0; i < beStaCount; ++i)
    {
        uint16_t port = ports[i];

        UdpServerHelper serverHelper (port);
        ApplicationContainer serverApp = serverHelper.Install (apNodes.Get (0));
        serverApp.Start (Seconds (0.0));
        serverApp.Stop (Seconds (simulationTime + 1.0));
        serverApps.push_back (serverApp);

        UdpClientHelper clientHelper (apInterface.GetAddress (0), port);
        clientHelper.SetAttribute ("MaxPackets", UintegerValue (4294967295u));
        clientHelper.SetAttribute ("Interval", TimeValue (Seconds (clientInterval)));
        clientHelper.SetAttribute ("PacketSize", UintegerValue (1472));
        ApplicationContainer clientApp = clientHelper.Install (staNodes.Get (i));
        clientApp.Start (Seconds (1.0));
        clientApp.Stop (Seconds (simulationTime + 1.0));
        clientApps.push_back (clientApp);
    }

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll ();

    Simulator::Stop (Seconds (simulationTime + 1.5));
    Simulator::Run ();

    monitor->CheckForLostPackets ();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();

    std::vector<double> throughput (beStaCount, 0.0);
    std::vector<double> avgDelay (beStaCount, 0.0);
    std::vector<double> avgJitter (beStaCount, 0.0);

    for (const auto &flow : stats)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (flow.first);
        for (uint32_t i = 0; i < beStaCount; ++i)
        {
            if (t.destinationPort == ports[i])
            {
                uint32_t rxPackets = flow.second.rxPackets;
                if (rxPackets > 0)
                {
                    throughput[i] = (flow.second.rxBytes * 8.0) / (simulationTime * 1e6);
                    avgDelay[i] = flow.second.delaySum.GetSeconds () / rxPackets;
                    if (rxPackets > 1)
                    {
                        avgJitter[i] = flow.second.jitterSum.GetSeconds () / (rxPackets - 1);
                    }
                }
            }
        }
    }

    std::cout << "Results after " << simulationTime << " seconds of simulation:" << std::endl;
    for (uint32_t i = 0; i < beStaCount; ++i)
    {
        std::cout << "802.11be STA #" << (i + 1)
                  << " - Throughput: " << throughput[i] << " Mbit/s"
                  << ", Average delay: " << (avgDelay[i] * 1000) << " ms"
                  << ", Average jitter: " << (avgJitter[i] * 1000) << " ms" << std::endl;
    }

    airtimeLogger.PrintSummary(simulationTime);
    monitor->SerializeToXmlFile ("scratch/flowmon/scenario_coex_be_3sta.flowmon", true, true);
    Simulator::Destroy ();
    return 0;
}
