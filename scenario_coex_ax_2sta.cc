/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Simulation code for coexistence of two Wi-Fi networks (802.11a and 802.11ax) on a single 5 GHz channel.
 * Each network has one AP and one STA, using channel 36 (20 MHz bandwidth) and separate SSIDs.
 * A-MPDU frame aggregation is disabled for both networks.
 * Stations generate saturated UDP uplink traffic (to their AP) at maximum rate.
 * FlowMonitor is used to collect throughput, delay, and jitter statistics, which are output and saved to XML.
 */
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
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
    wifiStaNodes.Create(2);

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
    wifiA.SetStandard(WIFI_STANDARD_80211ax);
    wifiB.SetStandard(WIFI_STANDARD_80211ax);
    wifiA.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue("HeMcs11"), "ControlMode", StringValue("HeMcs0"));
    wifiB.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue("HeMcs11"), "ControlMode", StringValue("HeMcs0"));

    WifiMacHelper mac;
    Ssid ssidA = Ssid("network-80211ax-1");
    Ssid ssidB = Ssid("network-80211ax");

    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssidA), "ActiveProbing", BooleanValue(true));
    NetDeviceContainer staDeviceA = wifiA.Install(phyA, mac, wifiStaNodes.Get(0));
    airtimeLogger.TrackDevices(staDeviceA, "staDeviceA");
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssidA), "EnableBeaconJitter", BooleanValue(false));
    NetDeviceContainer apDeviceA = wifiA.Install(phyA, mac, wifiApNodes.Get(0));

    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssidB), "ActiveProbing", BooleanValue(true));
    NetDeviceContainer staDeviceB = wifiB.Install(phyB, mac, wifiStaNodes.Get(1));
    airtimeLogger.TrackDevices(staDeviceB, "staDeviceB");
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
    Ipv4InterfaceContainer staInterfaceB = address.Assign(staDeviceB);

    PopulateArpCache ();

    // --- MOBILITY: oba AP w ustalonych pozycjach, STAs równomiernie na okręgach r=5 m wokół swoich AP
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

    const Vector apPosA (0.0, 0.0, 0.0);
    const Vector apPosB (0.0, 0.0, 0.0);
    positionAlloc->Add (apPosA);
    positionAlloc->Add (apPosB);

    const double r = 5.0;
    positionAlloc->Add (Vector (apPosA.x + r, apPosA.y, apPosA.z));
    positionAlloc->Add (Vector (apPosB.x + r, apPosB.y, apPosB.z));

    mobility.SetPositionAllocator (positionAlloc);
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (wifiApNodes);
    mobility.Install (wifiStaNodes);

    uint16_t portA = 9000;
    uint16_t portB = 9001;
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

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Simulator::Stop(Seconds(simulationTime + 1.5));
    Simulator::Run();

    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

    double throughputA = 0.0, throughputB = 0.0;
    double avgDelayA = 0.0, avgDelayB = 0.0;
    double avgJitterA = 0.0, avgJitterB = 0.0;

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
        }
    }

    std::cout << "Results after " << simulationTime << " seconds of simulation:" << std::endl;
    std::cout << "802.11ax network 1 - Throughput: " << throughputA << " Mbit/s"
              << ", Average delay: " << (avgDelayA * 1000) << " ms"
              << ", Average jitter: " << (avgJitterA * 1000) << " ms" << std::endl;
    std::cout << "802.11ax network - Throughput: " << throughputB << " Mbit/s"
              << ", Average delay: " << (avgDelayB * 1000) << " ms"
              << ", Average jitter: " << (avgJitterB * 1000) << " ms" << std::endl;

    airtimeLogger.PrintSummary(simulationTime);
    monitor->SerializeToXmlFile("scratch/flowmon/scenario_coex_ax_ax.flowmon", true, true);
    Simulator::Destroy();
    return 0;
}
