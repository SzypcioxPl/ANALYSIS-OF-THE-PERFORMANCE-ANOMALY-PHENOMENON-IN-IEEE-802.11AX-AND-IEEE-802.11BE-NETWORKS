/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#include <vector>
#include <tuple>
#include <sstream>
#include <cmath>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"
#include "../helpers/populate-arp.h"
#include "../helpers/airtime-logger.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiCoexistenceABeDecSta");

int main(int argc, char *argv[])
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

    const uint32_t beStaCount = 10;
    ns3::ShowProgress sp(Seconds(5));
    AirtimeLogger airtimeLogger;

    NodeContainer wifiApNodes;
    wifiApNodes.Create(2);
    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(1 + beStaCount);

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
    wifiLegacy.SetStandard(WIFI_STANDARD_80211a);
    wifiBe.SetStandard(WIFI_STANDARD_80211be);
    wifiLegacy.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                       "DataMode", StringValue("OfdmRate6Mbps"),
                                       "ControlMode", StringValue("OfdmRate6Mbps"));
    wifiBe.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                   "DataMode", StringValue("EhtMcs13"),
                                   "ControlMode", StringValue("OfdmRate54Mbps"));

    WifiMacHelper mac;
    Ssid ssidLegacy = Ssid("network-80211a");
    Ssid ssidBe = Ssid("network-80211be");

    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssidLegacy), "ActiveProbing", BooleanValue(true));
    NetDeviceContainer staDeviceLegacy = wifiLegacy.Install(phyLegacy, mac, wifiStaNodes.Get(0));
    airtimeLogger.TrackDevices(staDeviceLegacy, "staDeviceLegacy");
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssidLegacy), "EnableBeaconJitter", BooleanValue(false));
    NetDeviceContainer apDeviceLegacy = wifiLegacy.Install(phyLegacy, mac, wifiApNodes.Get(0));

    mac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(ssidBe),
                "ActiveProbing", BooleanValue(true),
                "AssocType", EnumValue(WifiAssocType::LEGACY));
    NetDeviceContainer staDevicesBe;
    for (uint32_t i = 0; i < beStaCount; ++i)
    {
        NetDeviceContainer sta = wifiBe.Install(phyBe, mac, wifiStaNodes.Get(1 + i));
        staDevicesBe.Add(sta);
    }
    airtimeLogger.TrackDevices(staDevicesBe, "staDevicesBe");

    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssidBe), "EnableBeaconJitter", BooleanValue(false));
    NetDeviceContainer apDeviceBe = wifiBe.Install(phyBe, mac, wifiApNodes.Get(1));

    InternetStackHelper stack;
    stack.Install(wifiApNodes);
    stack.Install(wifiStaNodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterfaceLegacy = address.Assign(apDeviceLegacy);
    Ipv4InterfaceContainer staInterfaceLegacy = address.Assign(staDeviceLegacy);
    (void)staInterfaceLegacy;

    address.SetBase("10.2.1.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterfaceBe = address.Assign(apDeviceBe);
    Ipv4InterfaceContainer staInterfacesBe = address.Assign(staDevicesBe);
    (void)staInterfacesBe;

    PopulateArpCache();

    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    const Vector apLegacyPos(0.0, 0.0, 0.0);
    const Vector apBePos(0.0, 0.0, 0.0);
    positionAlloc->Add(apLegacyPos);
    positionAlloc->Add(apBePos);

    const double r = 2.0;
    const double twoPi = 6.283185307179586;
    const double theta0 = 0.39269908169872414;

    positionAlloc->Add(Vector(apLegacyPos.x + r, apLegacyPos.y, apLegacyPos.z));

    for (uint32_t i = 0; i < beStaCount; ++i)
    {
        double theta = theta0 + twoPi * static_cast<double>(i) / static_cast<double>(beStaCount);
        double x = apBePos.x + r * std::cos(theta);
        double y = apBePos.y + r * std::sin(theta);
        positionAlloc->Add(Vector(x, y, apBePos.z));
    }

    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(wifiApNodes);
    mobility.Install(wifiStaNodes);

    AnimationInterface anim("scratch/netanim/scenario_coex_a_be_decsta.xml");
    anim.EnablePacketMetadata(true);
    anim.SetMobilityPollInterval(Seconds(0.25));

    uint32_t apLegacyId = wifiApNodes.Get(0)->GetId();
    uint32_t apBeId = wifiApNodes.Get(1)->GetId();
    anim.SetConstantPosition(wifiApNodes.Get(0), apLegacyPos.x, apLegacyPos.y);
    anim.SetConstantPosition(wifiApNodes.Get(1), apBePos.x, apBePos.y);

    anim.SetConstantPosition(wifiStaNodes.Get(0), apLegacyPos.x + r, apLegacyPos.y);

    std::vector<uint32_t> beStaIds;
    beStaIds.reserve(beStaCount);
    for (uint32_t i = 0; i < beStaCount; ++i)
    {
        Ptr<Node> staNode = wifiStaNodes.Get(1 + i);
        double theta = theta0 + twoPi * static_cast<double>(i) / static_cast<double>(beStaCount);
        double x = apBePos.x + r * std::cos(theta);
        double y = apBePos.y + r * std::sin(theta);
        anim.SetConstantPosition(staNode, x, y);
        beStaIds.push_back(staNode->GetId());
    }

    anim.UpdateNodeDescription(apLegacyId, "AP-802.11a");
    anim.UpdateNodeDescription(apBeId, "AP-802.11be");
    anim.UpdateNodeDescription(wifiStaNodes.Get(0)->GetId(), "STA-802.11a");
    for (uint32_t i = 0; i < beStaIds.size(); ++i)
    {
        std::ostringstream oss;
        oss << "STA-802.11be #" << (i + 1);
        anim.UpdateNodeDescription(beStaIds[i], oss.str());
    }

    anim.UpdateNodeColor(apLegacyId, 138, 43, 226);
    anim.UpdateNodeColor(wifiStaNodes.Get(0)->GetId(), 186, 85, 211);
    anim.UpdateNodeColor(apBeId, 65, 105, 225);
    const std::vector<std::tuple<uint8_t, uint8_t, uint8_t>> beColors = {
        {135, 206, 250}, {70, 130, 180}, {25, 25, 112}, {0, 0, 128}, {72, 61, 139},
        {0, 191, 255}, {30, 144, 255}, {65, 105, 225}, {100, 149, 237}, {176, 196, 222}
    };
    for (uint32_t i = 0; i < beStaIds.size(); ++i)
    {
        auto [rColor, gColor, bColor] = beColors[i % beColors.size()];
        anim.UpdateNodeColor(beStaIds[i], rColor, gColor, bColor);
    }

    uint16_t portLegacy = 9000;
    std::vector<uint16_t> bePorts(beStaCount);
    for (uint32_t i = 0; i < beStaCount; ++i)
    {
        bePorts[i] = 9001 + i;
    }

    UdpServerHelper legacyServer(portLegacy);
    ApplicationContainer serverAppLegacy = legacyServer.Install(wifiApNodes.Get(0));
    serverAppLegacy.Start(Seconds(0.0));
    serverAppLegacy.Stop(Seconds(simulationTime + 1.0));

    UdpClientHelper legacyClient(apInterfaceLegacy.GetAddress(0), portLegacy);
    legacyClient.SetAttribute("MaxPackets", UintegerValue(4294967295u));
    legacyClient.SetAttribute("Interval", TimeValue(Seconds(clientInterval)));
    legacyClient.SetAttribute("PacketSize", UintegerValue(1472));
    ApplicationContainer clientAppLegacy = legacyClient.Install(wifiStaNodes.Get(0));
    clientAppLegacy.Start(Seconds(1.0));
    clientAppLegacy.Stop(Seconds(simulationTime + 1.0));

    std::vector<ApplicationContainer> beServerApps;
    std::vector<ApplicationContainer> beClientApps;
    beServerApps.reserve(beStaCount);
    beClientApps.reserve(beStaCount);

    for (uint32_t i = 0; i < beStaCount; ++i)
    {
        UdpServerHelper beServer(bePorts[i]);
        ApplicationContainer serverApp = beServer.Install(wifiApNodes.Get(1));
        serverApp.Start(Seconds(0.0));
        serverApp.Stop(Seconds(simulationTime + 1.0));
        beServerApps.push_back(serverApp);

        UdpClientHelper beClient(apInterfaceBe.GetAddress(0), bePorts[i]);
        beClient.SetAttribute("MaxPackets", UintegerValue(4294967295u));
        beClient.SetAttribute("Interval", TimeValue(Seconds(clientInterval)));
        beClient.SetAttribute("PacketSize", UintegerValue(1472));
        ApplicationContainer clientApp = beClient.Install(wifiStaNodes.Get(1 + i));
        clientApp.Start(Seconds(1.0));
        clientApp.Stop(Seconds(simulationTime + 1.0));
        beClientApps.push_back(clientApp);
    }

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Simulator::Stop(Seconds(simulationTime + 1.5));
    Simulator::Run();

    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

    double throughputLegacy = 0.0;
    double avgDelayLegacy = 0.0;
    double avgJitterLegacy = 0.0;

    std::vector<double> throughputBe(beStaCount, 0.0);
    std::vector<double> avgDelayBe(beStaCount, 0.0);
    std::vector<double> avgJitterBe(beStaCount, 0.0);

    for (auto iter = stats.begin(); iter != stats.end(); ++iter)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(iter->first);
        if (t.destinationPort == portLegacy)
        {
            uint32_t rx = iter->second.rxPackets;
            if (rx > 0)
            {
                throughputLegacy = (iter->second.rxBytes * 8.0) / (simulationTime * 1e6);
                avgDelayLegacy = iter->second.delaySum.GetSeconds() / rx;
                if (rx > 1)
                {
                    avgJitterLegacy = iter->second.jitterSum.GetSeconds() / (rx - 1);
                }
            }
            continue;
        }

        for (uint32_t i = 0; i < beStaCount; ++i)
        {
            if (t.destinationPort == bePorts[i])
            {
                uint32_t rx = iter->second.rxPackets;
                if (rx > 0)
                {
                    throughputBe[i] = (iter->second.rxBytes * 8.0) / (simulationTime * 1e6);
                    avgDelayBe[i] = iter->second.delaySum.GetSeconds() / rx;
                    if (rx > 1)
                    {
                        avgJitterBe[i] = iter->second.jitterSum.GetSeconds() / (rx - 1);
                    }
                }
            }
        }
    }

    std::cout << "Results after " << simulationTime << " seconds of simulation:" << std::endl;
    std::cout << "802.11a network - Throughput: " << throughputLegacy << " Mbit/s"
              << ", Average delay: " << (avgDelayLegacy * 1000) << " ms"
              << ", Average jitter: " << (avgJitterLegacy * 1000) << " ms" << std::endl;

    for (uint32_t i = 0; i < beStaCount; ++i)
    {
        std::cout << "802.11be STA #" << (i + 1)
                  << " - Throughput: " << throughputBe[i] << " Mbit/s"
                  << ", Average delay: " << (avgDelayBe[i] * 1000) << " ms"
                  << ", Average jitter: " << (avgJitterBe[i] * 1000) << " ms" << std::endl;
    }

    airtimeLogger.PrintSummary(simulationTime);
    monitor->SerializeToXmlFile("scratch/flowmon/scenario_coex_a_be_decsta.flowmon", true, true);
    Simulator::Destroy();
    return 0;
}
