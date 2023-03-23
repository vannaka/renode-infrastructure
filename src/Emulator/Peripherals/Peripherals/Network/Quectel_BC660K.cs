//
// Copyright (c) 2010-2023 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;

namespace Antmicro.Renode.Peripherals.Network
{
    public class Quectel_BC660K : QuectelModem
    {
        public Quectel_BC660K(Machine machine, string imeiNumber = DefaultImeiNumber,
            string softwareVersionNumber = DefaultSoftwareVersionNumber,
            string serialNumber = DefaultSerialNumber) : base(machine, imeiNumber, softwareVersionNumber, serialNumber)
        {
        }

        // QCFG - System Configuration
        [AtCommand("AT+QCFG", CommandType.Write)]
        protected override Response Qcfg(string function, int value)
        {
            switch(function)
            {
                case "dsevent":
                    deepSleepEventEnabled = value != 0;
                    break;
                case "DataInactTimer": // inactivity timer
                case "EPCO": // extended protocol configuration options
                case "faultaction": // action performed by the UE after an error occurs
                case "GPIO": // GPIO status
                case "logbaudrate": // baud rate
                case "MacRAI": // enable or disable RAI in MAC layer
                case "NBcategory": // UE category
                case "NcellMeas": // NcellMeas
                case "OOSScheme": // network searching mechanism in OOS
                case "relversion": // protocol release version
                case "SimBip": // enable or disable SIMBIP
                case "slplocktimes": // sleep duration
                case "statisr": // report interval of the statistics URC
                case "wakeupRXD": // whether the UE can be woken up by RXD
                    this.Log(LogLevel.Warning, "Config value '{0}' set to {1}, not implemented", function, value);
                    break;
                default:
                    return base.Qcfg(function, value);
            }
            return Ok;
        }

        protected override bool IsValidContextId(int id)
        {
            return id == 0;
        }

        protected override string Vendor => "Quectel_Ltd";
        protected override string ModelName => "Quectel_BC660K-GL";
        protected override string Revision => "Revision: QCX212";
        protected override string ManufacturerRevision => "BC660KGLAAR01A03";
        protected override string SoftwareRevision => "01.002.01.002";

        private const string DefaultImeiNumber = "866818039921444";
        private const string DefaultSoftwareVersionNumber = "31";
        private const string DefaultSerialNumber = "<serial number>";
    }
}
