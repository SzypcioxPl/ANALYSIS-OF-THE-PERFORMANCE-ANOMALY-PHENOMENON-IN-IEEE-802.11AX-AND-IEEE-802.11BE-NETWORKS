/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
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

NS_LOG_COMPONENT_DEFINE ("WifiCoexistenceAcBeExample");

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

    ns3::ShowProgress sp(Seconds(5));
    AirtimeLogger airtimeLogger;

    NodeContainer wifiApNodes;
    wifiApNodes.Create(2);
    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(2);

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel", "Exponent", DoubleValue(1.0), "ReferenceLoss", DoubleValue(0.0));
    Ptr<YansWifiChannel> sharedChannel = channel.Create();

    YansWifiPhyHelper phyLegacy;
    YansWifiPhyHelper phyBe;
    phyLegacy.SetChannel(sharedChannel);
    phyBe.SetChannel(sharedChannel);
    phyLegacy.Set("ChannelSettings", StringValue("{36, 20, BAND_5GHZ, 0}"));
    phyBe.Set("ChannelSettings", StringValue("{36, 20, BAND_5GHZ, 0}"));

    WifiHelper wifiLegacy, wifiBe;
    wifiLegacy.SetStandard(WIFI_STANDARD_80211ac);
    wifiBe.SetStandard(WIFI_STANDARD_80211be);
    wifiLegacy.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                       "DataMode", StringValue("VhtMcs0"),
                                       "ControlMode", StringValue("VhtMcs0"));
    wifiBe.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                   "DataMode", StringValue("EhtMcs13"),
                                   "ControlMode", StringValue("OfdmRate54Mbps"));

    WifiMacHelper mac;
    Ssid ssidLegacy = Ssid("network-80211ac");
    Ssid ssidBe = Ssid("network-80211be");

    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssidLegacy), "ActiveProbing", BooleanValue(true));
    NetDeviceContainer staDeviceLegacy = wifiLegacy.Install(phyLegacy, mac, wifiStaNodes.Get(0));
    airtimeLogger.TrackDevices(staDeviceLegacy, "staDeviceLegacy");
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssidLegacy), "EnableBeaconJitter", BooleanValue(false));
    NetDeviceContainer apDeviceLegacy = wifiLegacy.Install(phyLegacy, mac, wifiApNodes.Get(0));

    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssidBe), "ActiveProbing", BooleanValue(true));
    NetDeviceContainer staDeviceBe = wifiBe.Install(phyBe, mac, wifiStaNodes.Get(1));
    airtimeLogger.TrackDevices(staDeviceBe, "staDeviceBe");
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssidBe), "EnableBeaconJitter", BooleanValue(false));
    NetDeviceContainer apDeviceBe = wifiBe.Install(phyBe, mac, wifiApNodes.Get(1));

    InternetStackHelper stack;
    stack.Install(wifiApNodes);
    stack.Install(wifiStaNodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterfaceLegacy = address.Assign(apDeviceLegacy);
    Ipv4InterfaceContainer staInterfaceLegacy = address.Assign(staDeviceLegacy);
    address.SetBase("10.2.1.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterfaceBe = address.Assign(apDeviceBe);
    Ipv4InterfaceContainer staInterfaceBe = address.Assign(staDeviceBe);

    PopulateArpCache ();

    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
    const Vector apLegacyPos (0.0, 0.0, 0.0);
    const Vector apBePos (0.0, 0.0, 0.0);
    positionAlloc->Add (apLegacyPos);
    positionAlloc->Add (apBePos);

    const double r = 2.0;
    const double twoPi = 6.283185307179586;
    const double theta0 = 0.39269908169872414;

    if (wifiStaNodes.GetN () > 0)
    {
        positionAlloc->Add (Vector (apLegacyPos.x + r, apLegacyPos.y, apLegacyPos.z));
    }

    const uint32_t beStaCount = (wifiStaNodes.GetN () > 0) ? (wifiStaNodes.GetN () - 1) : 0;
    for (uint32_t i = 0; i < beStaCount; ++i)
    {
        double theta = theta0 + twoPi * static_cast<double> (i) / static_cast<double> (beStaCount);
        double x = apBePos.x + r * std::cos (theta);
        double y = apBePos.y + r * std::sin (theta);
        positionAlloc->Add (Vector (x, y, apBePos.z));
    }

    mobility.SetPositionAllocator (positionAlloc);
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (wifiApNodes);
    mobility.Install (wifiStaNodes);

    AnimationInterface anim("scratch/netanim/scenario_coex_ac_be.xml");
    anim.EnablePacketMetadata(true);
    anim.SetMobilityPollInterval(Seconds(0.25));

    uint32_t apLegacyId  = wifiApNodes.Get(0)->GetId();
    uint32_t apBeId  = wifiApNodes.Get(1)->GetId();
    uint32_t staLegacyId = wifiStaNodes.Get(0)->GetId();
    uint32_t staBeId = wifiStaNodes.Get(1)->GetId();

    anim.SetConstantPosition (wifiApNodes.Get (0), apLegacyPos.x, apLegacyPos.y);
    anim.SetConstantPosition (wifiApNodes.Get (1), apBePos.x, apBePos.y);

    if (wifiStaNodes.GetN () > 0)
    {
        anim.SetConstantPosition (wifiStaNodes.Get (0), apLegacyPos.x + r, apLegacyPos.y);
    }

    for (uint32_t i = 0; i < beStaCount; ++i)
    {
        Ptr<Node> staNode = wifiStaNodes.Get (1 + i);
        double theta = theta0 + twoPi * static_cast<double> (i) / static_cast<double> (beStaCount);
        double x = apBePos.x + r * std::cos (theta);
        double y = apBePos.y + r * std::sin (theta);
        anim.SetConstantPosition (staNode, x, y);
    }

    anim.UpdateNodeDescription(apLegacyId,  "AP-802.11ac");
    anim.UpdateNodeDescription(apBeId,  "AP-802.11be");
    anim.UpdateNodeDescription(staLegacyId, "STA-802.11ac");
    anim.UpdateNodeDescription(staBeId, "STA-802.11be");

    anim.UpdateNodeColor(apLegacyId,  138, 43, 226);
    anim.UpdateNodeColor(staLegacyId, 186, 85, 211);
    anim.UpdateNodeColor(apBeId,  65, 105, 225);
    anim.UpdateNodeColor(staBeId, 135, 206, 250);

    uint16_t portLegacy = 9000;
    uint16_t portBe = 9001;
    UdpServerHelper udpServerLegacy(portLegacy);
    ApplicationContainer serverAppLegacy = udpServerLegacy.Install(wifiApNodes.Get(0));
    serverAppLegacy.Start(Seconds(0.0));
    serverAppLegacy.Stop(Seconds(simulationTime + 1.0));

    UdpClientHelper udpClientLegacy(apInterfaceLegacy.GetAddress(0), portLegacy);
    udpClientLegacy.SetAttribute("MaxPackets", UintegerValue(4294967295u));
    udpClientLegacy.SetAttribute("Interval", TimeValue(Seconds(clientInterval)));
    udpClientLegacy.SetAttribute("PacketSize", UintegerValue(1472));
    ApplicationContainer clientAppLegacy = udpClientLegacy.Install(wifiStaNodes.Get(0));
    clientAppLegacy.Start(Seconds(1.0));
    clientAppLegacy.Stop(Seconds(simulationTime + 1.0));

    UdpServerHelper udpServerBe(portBe);
    ApplicationContainer serverAppBe = udpServerBe.Install(wifiApNodes.Get(1));
    serverAppBe.Start(Seconds(0.0));
    serverAppBe.Stop(Seconds(simulationTime + 1.0));

    UdpClientHelper udpClientBe(apInterfaceBe.GetAddress(0), portBe);
    udpClientBe.SetAttribute("MaxPackets", UintegerValue(4294967295u));
    udpClientBe.SetAttribute("Interval", TimeValue(Seconds(clientInterval)));
    udpClientBe.SetAttribute("PacketSize", UintegerValue(1472));
    ApplicationContainer clientAppBe = udpClientBe.Install(wifiStaNodes.Get(1));
    clientAppBe.Start(Seconds(1.0));
    clientAppBe.Stop(Seconds(simulationTime + 1.0));

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Simulator::Stop(Seconds(simulationTime + 1.5));
    Simulator::Run();

    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

    double throughputLegacy = 0.0, throughputBe = 0.0;
    double avgDelayLegacy = 0.0, avgDelayBe = 0.0;
    double avgJitterLegacy = 0.0, avgJitterBe = 0.0;

    for (auto iter = stats.begin(); iter != stats.end(); ++iter) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(iter->first);
        if (t.destinationPort == portLegacy) {
            uint32_t rxLegacy = iter->second.rxPackets;
            if (rxLegacy > 0) {
                throughputLegacy = (iter->second.rxBytes * 8.0) / (simulationTime * 1e6);
                avgDelayLegacy = iter->second.delaySum.GetSeconds() / rxLegacy;
                if (rxLegacy > 1) {
                    avgJitterLegacy = iter->second.jitterSum.GetSeconds() / (rxLegacy - 1);
                }
            }
        } else if (t.destinationPort == portBe) {
            uint32_t rxBe = iter->second.rxPackets;
            if (rxBe > 0) {
                throughputBe = (iter->second.rxBytes * 8.0) / (simulationTime * 1e6);
                avgDelayBe = iter->second.delaySum.GetSeconds() / rxBe;
                if (rxBe > 1) {
                    avgJitterBe = iter->second.jitterSum.GetSeconds() / (rxBe - 1);
                }
            }
        }
    }

    std::cout << "Results after " << simulationTime << " seconds of simulation:" << std::endl;
    std::cout << "802.11ac network - Throughput: " << throughputLegacy << " Mbit/s"
              << ", Average delay: " << (avgDelayLegacy * 1000) << " ms"
              << ", Average jitter: " << (avgJitterLegacy * 1000) << " ms" << std::endl;
    std::cout << "802.11be network - Throughput: " << throughputBe << " Mbit/s"
              << ", Average delay: " << (avgDelayBe * 1000) << " ms"
              << ", Average jitter: " << (avgJitterBe * 1000) << " ms" << std::endl;

    airtimeLogger.PrintSummary(simulationTime);
    monitor->SerializeToXmlFile("scratch/flowmon/scenario_coex_ac_be.flowmon", true, true);
    Simulator::Destroy();
    return 0;
}
