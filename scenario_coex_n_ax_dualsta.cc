/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"     // NETANIM: dodane
#include <cmath>
#include "../helpers/populate-arp.h"
#include "../helpers/airtime-logger.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WifiCoexistenceExample");

int main(int argc, char *argv[])
{
    uint32_t beMaxAmpdu = 0;
    double simulationTime = 260.0;   // seconds
    double clientInterval = 0.0001;  // seconds

    CommandLine cmd;
    cmd.AddValue("beMaxAmpdu", "Maximum A-MPDU size for BE traffic (bytes, 0 disables aggregation)", beMaxAmpdu);
    cmd.AddValue("simulationTime", "Total simulation time (s)", simulationTime);
    cmd.AddValue("clientInterval", "UDP client packet interval (s)", clientInterval);
    cmd.Parse(argc, argv);

    Config::SetDefault("ns3::WifiMac::BE_MaxAmpduSize", UintegerValue(beMaxAmpdu));

    ns3::ShowProgress sp(Seconds(5)); // co ~5 s zegara ściennego
    AirtimeLogger airtimeLogger;

    NodeContainer wifiApNodes;
    wifiApNodes.Create(2);
    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(3);

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel", "Exponent", DoubleValue(1.0), "ReferenceLoss", DoubleValue(0.0));
    Ptr<YansWifiChannel> sharedChannel = channel.Create();

    YansWifiPhyHelper phyA;
    YansWifiPhyHelper phyB;
    phyA.SetChannel(sharedChannel);
    phyB.SetChannel(sharedChannel);
    phyA.Set("ChannelSettings", StringValue("{36, 20, BAND_5GHZ, 0}"));
    phyB.Set("ChannelSettings", StringValue("{36, 20, BAND_5GHZ, 0}"));

    WifiHelper wifiA, wifiB;
    wifiA.SetStandard(WIFI_STANDARD_80211n);
    wifiB.SetStandard(WIFI_STANDARD_80211ax);
    wifiA.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue("HtMcs0"), "ControlMode", StringValue("HtMcs0"));
    wifiB.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue("HeMcs11"), "ControlMode", StringValue("HeMcs0"));

    WifiMacHelper mac;
    Ssid ssidA = Ssid("network-80211n");
    Ssid ssidB = Ssid("network-80211ax");

    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssidA), "ActiveProbing", BooleanValue(true));
    NetDeviceContainer staDeviceA = wifiA.Install(phyA, mac, wifiStaNodes.Get(0));
    airtimeLogger.TrackDevices(staDeviceA, "staDeviceA");
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssidA), "EnableBeaconJitter", BooleanValue(false));
    NetDeviceContainer apDeviceA = wifiA.Install(phyA, mac, wifiApNodes.Get(0));

    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssidB), "ActiveProbing", BooleanValue(true));
    NetDeviceContainer staDeviceB1 = wifiB.Install(phyB, mac, wifiStaNodes.Get(1));
    airtimeLogger.TrackDevices(staDeviceB1, "staDeviceB1");
    NetDeviceContainer staDeviceB2 = wifiB.Install(phyB, mac, wifiStaNodes.Get(2));
    airtimeLogger.TrackDevices(staDeviceB2, "staDeviceB2");
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssidB), "EnableBeaconJitter", BooleanValue(false));
    NetDeviceContainer apDeviceB = wifiB.Install(phyB, mac, wifiApNodes.Get(1));

    InternetStackHelper stack;
    stack.Install(wifiApNodes);
    stack.Install(wifiStaNodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterfaceA = address.Assign(apDeviceA);
    Ipv4InterfaceContainer staInterfaceA = address.Assign(staDeviceA);
    address.SetBase("10.2.1.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterfaceB = address.Assign(apDeviceB);
    Ipv4InterfaceContainer staInterfaceB1 = address.Assign(staDeviceB1);
    Ipv4InterfaceContainer staInterfaceB2 = address.Assign(staDeviceB2);

    PopulateArpCache ();

    // --- MOBILITY: AP-y w ustalonych pozycjach, STAs na okręgu r=5 m wokół odpowiednich AP
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
    const Vector apNPos (0.0, 0.0, 0.0);
    const Vector apAxPos (0.0, 0.0, 0.0);
    positionAlloc->Add (apNPos);
    positionAlloc->Add (apAxPos);

    const double r = 5.0;
    const double twoPi = 6.283185307179586;   // 2*pi
    const double theta0 = 0.39269908169872414; // pi/8

    if (wifiStaNodes.GetN () > 0)
    {
        positionAlloc->Add (Vector (apNPos.x + r, apNPos.y, apNPos.z));
    }

    const uint32_t axStaCount = (wifiStaNodes.GetN () > 0) ? (wifiStaNodes.GetN () - 1) : 0;
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

    // -------- NETANIM: generator pliku animation.xml + opisy/kolory ----------
    AnimationInterface anim("scratch/netanim/scenario_coex_n_ax_dualsta.xml");
    anim.EnablePacketMetadata(true);                // pokaż nagłówki pakietów w NetAnim
    anim.SetMobilityPollInterval(Seconds(0.25));    // próbkowanie pozycji (opcjonalnie)

    // Id węzłów w animacji
    uint32_t apAId  = wifiApNodes.Get(0)->GetId();
    uint32_t apBId  = wifiApNodes.Get(1)->GetId();
    uint32_t staAId = wifiStaNodes.Get(0)->GetId();
    uint32_t staB1Id = wifiStaNodes.Get(1)->GetId();
    uint32_t staB2Id = wifiStaNodes.Get(2)->GetId();

    // Przyklejamy te same pozycje co w Mobility (żeby GUI miało punkt odniesienia)
    anim.SetConstantPosition (wifiApNodes.Get (0), apNPos.x, apNPos.y);
    anim.SetConstantPosition (wifiApNodes.Get (1), apAxPos.x, apAxPos.y);

    if (wifiStaNodes.GetN () > 0)
    {
        anim.SetConstantPosition (wifiStaNodes.Get (0), apNPos.x + r, apNPos.y);
    }

    for (uint32_t i = 0; i < axStaCount; ++i)
    {
        Ptr<Node> staNode = wifiStaNodes.Get (1 + i);
        double theta = theta0 + twoPi * static_cast<double> (i) / static_cast<double> (axStaCount);
        double x = apAxPos.x + r * std::cos (theta);
        double y = apAxPos.y + r * std::sin (theta);
        anim.SetConstantPosition (staNode, x, y);
    }

    // Opisy
    anim.UpdateNodeDescription(apAId,  "AP-802.11n");
    anim.UpdateNodeDescription(apBId,  "AP-802.11ax");
    anim.UpdateNodeDescription(staAId, "STA-802.11n");
    anim.UpdateNodeDescription(staB1Id, "STA-802.11ax #1");
    anim.UpdateNodeDescription(staB2Id, "STA-802.11ax #2");

    // Kolory: 802.11n = odcienie zieleni, 802.11ax = odcienie niebieskiego
    anim.UpdateNodeColor(apAId,  34, 139, 34);   // forest green
    anim.UpdateNodeColor(staAId, 60, 179, 113);  // medium sea green
    anim.UpdateNodeColor(apBId,  65, 105, 225);  // royal blue
    anim.UpdateNodeColor(staB1Id, 135, 206, 250); // light sky blue
    anim.UpdateNodeColor(staB2Id, 70, 130, 180);  // steel blue
    // -------------------------------------------------------------------------

    uint16_t portA = 9000;
    uint16_t portB = 9001;
    uint16_t portC = 9002;
    UdpServerHelper udpServerA(portA);
    ApplicationContainer serverAppA = udpServerA.Install(wifiApNodes.Get(0));
    serverAppA.Start(Seconds(0.0));
    serverAppA.Stop(Seconds(simulationTime + 1.0));

    UdpClientHelper udpClientA(apInterfaceA.GetAddress(0), portA);
    udpClientA.SetAttribute("MaxPackets", UintegerValue(4294967295u));
    udpClientA.SetAttribute("Interval", TimeValue(Seconds(clientInterval)));
    udpClientA.SetAttribute("PacketSize", UintegerValue(1472));
    ApplicationContainer clientAppA = udpClientA.Install(wifiStaNodes.Get(0));
    clientAppA.Start(Seconds(1.0));
    clientAppA.Stop(Seconds(simulationTime + 1.0));

    UdpServerHelper udpServerB(portB);
    ApplicationContainer serverAppB = udpServerB.Install(wifiApNodes.Get(1));
    serverAppB.Start(Seconds(0.0));
    serverAppB.Stop(Seconds(simulationTime + 1.0));

    UdpClientHelper udpClientB(apInterfaceB.GetAddress(0), portB);
    udpClientB.SetAttribute("MaxPackets", UintegerValue(4294967295u));
    udpClientB.SetAttribute("Interval", TimeValue(Seconds(clientInterval)));
    udpClientB.SetAttribute("PacketSize", UintegerValue(1472));
    ApplicationContainer clientAppB = udpClientB.Install(wifiStaNodes.Get(1));
    clientAppB.Start(Seconds(1.0));
    clientAppB.Stop(Seconds(simulationTime + 1.0));

    UdpServerHelper udpServerC(portC);
    ApplicationContainer serverAppC = udpServerC.Install(wifiApNodes.Get(1));
    serverAppC.Start(Seconds(0.0));
    serverAppC.Stop(Seconds(simulationTime + 1.0));

    UdpClientHelper udpClientC(apInterfaceB.GetAddress(0), portC);
    udpClientC.SetAttribute("MaxPackets", UintegerValue(4294967295u));
    udpClientC.SetAttribute("Interval", TimeValue(Seconds(clientInterval)));
    udpClientC.SetAttribute("PacketSize", UintegerValue(1472));
    ApplicationContainer clientAppC = udpClientC.Install(wifiStaNodes.Get(2));
    clientAppC.Start(Seconds(1.0));
    clientAppC.Stop(Seconds(simulationTime + 1.0));

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Simulator::Stop(Seconds(simulationTime + 1.5));
    Simulator::Run();

    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

    double throughputA = 0.0, throughputB = 0.0, throughputC = 0.0;
    double avgDelayA = 0.0, avgDelayB = 0.0, avgDelayC = 0.0;
    double avgJitterA = 0.0, avgJitterB = 0.0, avgJitterC = 0.0;

    for (auto iter = stats.begin(); iter != stats.end(); ++iter) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(iter->first);
        if (t.destinationPort == portA) {
            uint32_t rxA = iter->second.rxPackets;
            if (rxA > 0) {
                throughputA = (iter->second.rxBytes * 8.0) / (simulationTime * 1e6);
                avgDelayA = iter->second.delaySum.GetSeconds() / rxA;
                if (rxA > 1) {
                    avgJitterA = iter->second.jitterSum.GetSeconds() / (rxA - 1);
                }
            }
        } else if (t.destinationPort == portB) {
            uint32_t rxB = iter->second.rxPackets;
            if (rxB > 0) {
                throughputB = (iter->second.rxBytes * 8.0) / (simulationTime * 1e6);
                avgDelayB = iter->second.delaySum.GetSeconds() / rxB;
                if (rxB > 1) {
                    avgJitterB = iter->second.jitterSum.GetSeconds() / (rxB - 1);
                }
            }
        } else if (t.destinationPort == portC) {
            uint32_t rxC = iter->second.rxPackets;
            if (rxC > 0) {
                throughputC = (iter->second.rxBytes * 8.0) / (simulationTime * 1e6);
                avgDelayC = iter->second.delaySum.GetSeconds() / rxC;
                if (rxC > 1) {
                    avgJitterC = iter->second.jitterSum.GetSeconds() / (rxC - 1);
                }
            }
        }
    }

    std::cout << "Results after " << simulationTime << " seconds of simulation:" << std::endl;
    std::cout << "802.11n network - Throughput: " << throughputA << " Mbit/s"
              << ", Average delay: " << (avgDelayA * 1000) << " ms"
              << ", Average jitter: " << (avgJitterA * 1000) << " ms" << std::endl;
    std::cout << "802.11ax STA #1 - Throughput: " << throughputB << " Mbit/s"
              << ", Average delay: " << (avgDelayB * 1000) << " ms"
              << ", Average jitter: " << (avgJitterB * 1000) << " ms" << std::endl;
    std::cout << "802.11ax STA #2 - Throughput: " << throughputC << " Mbit/s"
              << ", Average delay: " << (avgDelayC * 1000) << " ms"
              << ", Average jitter: " << (avgJitterC * 1000) << " ms" << std::endl;

    airtimeLogger.PrintSummary(simulationTime);
    monitor->SerializeToXmlFile("scratch/flowmon/scenario_coex_n_ax_dualsta.flowmon", true, true);
    Simulator::Destroy();
    return 0;
}
