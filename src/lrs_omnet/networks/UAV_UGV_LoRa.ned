//
// UAV-UGV LoRa Communication Network
//
// This network models point-to-point LoRa communication between
// a UAV (aerial node) and UGV (ground node). Positions are updated
// dynamically from Gazebo via ROS bridge.
//

package networks;

import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.inet.AdhocHost;
import inet.physicallayer.wireless.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.wireless.apsk.packetlevel.ApskScalarRadioMedium;
import inet.environment.common.PhysicalEnvironment;


//
// UAV Node with LoRa radio
//
module UAVNode extends AdhocHost
{
    parameters:
        @display("i=misc/drone");
        mobility.typename = "StationaryMobility";  // Position set by ROS bridge
        wlan[*].typename = "LoRaRadio";
        numWlanInterfaces = 1;
}

//
// UGV Node with LoRa radio
//
module UGVNode extends AdhocHost
{
    parameters:
        @display("i=misc/car");
        mobility.typename = "StationaryMobility";  // Position set by ROS bridge
        wlan[*].typename = "LoRaRadio";
        numWlanInterfaces = 1;
}

//
// Simple LoRa radio definition
//
moduleinterface ILoRaRadio
{
    gates:
        input upperLayerIn;
        output upperLayerOut;
        input radioIn @labels(IRadioFrame);
}

//
// Main network: UAV and UGV with LoRa communication
//
network UAV_UGV_LoRa
{
    parameters:
        @display("bgb=1000,1000");
        
    submodules:
        radioMedium: ApskScalarRadioMedium {
            @display("p=50,50");
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=50,100");
        }
        
        uav: UAVNode {
            @display("p=500,200");
        }
        
        ugv: UGVNode {
            @display("p=500,700");
        }
}
