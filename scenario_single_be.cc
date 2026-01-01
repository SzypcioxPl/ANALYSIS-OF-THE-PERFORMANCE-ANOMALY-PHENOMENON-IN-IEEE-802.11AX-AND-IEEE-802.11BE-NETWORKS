/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"
#include "ns3/sta-wifi-mac.h"
#include <cmath>
#include "../helpers/populate-arp.h"
#include "../helpers/airtime-logger.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiSingleBeExample");

int main(int argc, char* argv[])
{
    uint32_t beMaxAmpdu = 0;
    double simulationTime = 260.0;
    double clientInterval = 0.0001;

    CommandLine cmd;
    cmd.AddValue("beMaxAmpdu", "Maximum A-MPDU size for BE traffic (bytes, 0 disables aggregation)", beMaxAmpdu);
    cmd.AddValue("simulationTime", "Total simulation time (s)", simulationTime);
    cmd.AddValue("clientInterval", "UDP client packet interval (s)", clientInterval);
    cmd.Parse(argc, argv);

    Config::SetDefault("ns3::WifiMac::BE_MaxAmpduSize", UintegerValue(beMaxAmpdu));

    ns3::ShowProgress sp(Seconds(5));
    AirtimeLogger airtimeLogger;

    NodeContainer wifiApNodes;
    wifiApNodes.Create(1);
    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(1);

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel", "Exponent", DoubleValue(1.0), "ReferenceLoss", DoubleValue(0.0));
    Ptr<YansWifiChannel> sharedChannel = channel.Create();

    YansWifiPhyHelper phy;
    phy.SetChannel(sharedChannel);
    phy.Set("ChannelSettings", StringValue("{36, 20, BAND_5GHZ, 0}"));

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211be);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode", StringValue("EhtMcs13"),
                                 "ControlMode", StringValue("OfdmRate54Mbps"));

    WifiMacHelper mac;
    Ssid ssid = Ssid("network-80211be");

    mac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(ssid),
                "ActiveProbing", BooleanValue(true),
                "AssocType", EnumValue(WifiAssocType::LEGACY));
    NetDeviceContainer staDevice = wifi.Install(phy, mac, wifiStaNodes);
    airtimeLogger.TrackDevices(staDevice, "staDevice");
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid), "EnableBeaconJitter", BooleanValue(false));
    NetDeviceContainer apDevice = wifi.Install(phy, mac, wifiApNodes);

    InternetStackHelper stack;
    stack.Install(wifiApNodes);
    stack.Install(wifiStaNodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterface = address.Assign(apDevice);
    Ipv4InterfaceContainer staInterface = address.Assign(staDevice);
    (void)apInterface;
    (void)staInterface;

    PopulateArpCache();

    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    const Vector apPos(0.0, 0.0, 0.0);
    const Vector staPos(2.0, 0.0, 0.0);
    positionAlloc->Add(apPos);
    positionAlloc->Add(staPos);

    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(wifiApNodes);
    mobility.Install(wifiStaNodes);

    AnimationInterface anim("scratch/netanim/scenario_single_be.xml");
    anim.EnablePacketMetadata(true);
    anim.SetMobilityPollInterval(Seconds(0.25));
    anim.SetConstantPosition(wifiApNodes.Get(0), apPos.x, apPos.y);
    anim.SetConstantPosition(wifiStaNodes.Get(0), staPos.x, staPos.y);
    anim.UpdateNodeDescription(wifiApNodes.Get(0)->GetId(), "AP-802.11be");
    anim.UpdateNodeDescription(wifiStaNodes.Get(0)->GetId(), "STA-802.11be");
    anim.UpdateNodeColor(wifiApNodes.Get(0)->GetId(), 65, 105, 225);
    anim.UpdateNodeColor(wifiStaNodes.Get(0)->GetId(), 135, 206, 250);

    uint16_t port = 9000;
    UdpServerHelper udpServer(port);
    ApplicationContainer serverApp = udpServer.Install(wifiApNodes.Get(0));
    serverApp.Start(Seconds(0.0));
    serverApp.Stop(Seconds(simulationTime + 1.0));

    UdpClientHelper udpClient(apInterface.GetAddress(0), port);
    udpClient.SetAttribute("MaxPackets", UintegerValue(4294967295u));
    udpClient.SetAttribute("Interval", TimeValue(Seconds(clientInterval)));
    udpClient.SetAttribute("PacketSize", UintegerValue(1472));
    ApplicationContainer clientApp = udpClient.Install(wifiStaNodes.Get(0));
    clientApp.Start(Seconds(1.0));
    clientApp.Stop(Seconds(simulationTime + 1.0));

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Simulator::Stop(Seconds(simulationTime + 1.5));
    Simulator::Run();

    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

    double throughput = 0.0;
    double avgDelay = 0.0;
    double avgJitter = 0.0;

    for (auto iter = stats.begin(); iter != stats.end(); ++iter)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(iter->first);
        if (t.destinationPort == port)
        {
            uint32_t rx = iter->second.rxPackets;
            if (rx > 0)
            {
                throughput = (iter->second.rxBytes * 8.0) / (simulationTime * 1e6);
                avgDelay = iter->second.delaySum.GetSeconds() / rx;
                if (rx > 1)
                {
                    avgJitter = iter->second.jitterSum.GetSeconds() / (rx - 1);
                }
            }
        }
    }

    std::cout << "Results after " << simulationTime << " seconds of simulation:" << std::endl;
    std::cout << "802.11be network - Throughput: " << throughput << " Mbit/s"
              << ", Average delay: " << (avgDelay * 1000) << " ms"
              << ", Average jitter: " << (avgJitter * 1000) << " ms" << std::endl;

    airtimeLogger.PrintSummary(simulationTime);
    monitor->SerializeToXmlFile("scratch/flowmon/scenario_single_be.flowmon", true, true);
    Simulator::Destroy();
    return 0;
}
