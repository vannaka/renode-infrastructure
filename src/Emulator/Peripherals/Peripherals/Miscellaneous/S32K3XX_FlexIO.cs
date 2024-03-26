//
// Copyright (c) 2010-2024 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class S32K3XX_FlexIO : BasicDoubleWordPeripheral, IKnownSize
    {
        public S32K3XX_FlexIO(IMachine machine) : base(machine)
        {
            DefineRegisters();
        }

        public long Size => 0x4000;

        private void DefineRegisters()
        {
            Registers.VersionID.Define(this, 0x02010003)
                .WithTag("MajorVersionNumber", 24, 8)
                .WithTag("MinorVersionNumber", 16, 8)
                .WithTag("FeatureSpecificationNumber", 0, 16);

            Registers.Parameter.Define(this, 0x04200808)
                .WithTag("TriggerNumber", 24, 8)
                .WithTag("PinNumber", 16, 8)
                .WithTag("TimerNumber", 8, 8)
                .WithTag("ShifterNumber", 0, 8);

            Registers.Control.Define(this)
                .WithReservedBits(31, 1)
                .WithTaggedFlag("DebugEnable", 30)
                .WithReservedBits(2, 28)
                .WithTaggedFlag("SoftwareReset", 1)
                .WithTaggedFlag("Enable", 0);

            Registers.PinState.Define(this)
                .WithTag("PinDataInput", 0, 32);

            Registers.ShifterStatus.Define(this)
                .WithReservedBits(8, 24)
                .WithTag("ShifterStatusFlags", 0, 8);

            Registers.ShifterError.Define(this)
                .WithReservedBits(8, 24)
                .WithTag("ShifterErrorFlags", 0, 8);

            Registers.TimerStatus.Define(this)
                .WithReservedBits(8, 24)
                .WithTag("TimerStatusFlags", 0, 8);

            Registers.ShifterStatusInterruptEnable.Define(this)
                .WithReservedBits(8, 24)
                .WithTag("ShifterStatusInterruptEnable", 0, 8);

            Registers.ShifterErrorInterruptEnable.Define(this)
                .WithReservedBits(8, 24)
                .WithTag("ShifterErrorInterruptEnable", 0, 8);

            Registers.TimerInterruptEnable.Define(this)
                .WithReservedBits(8, 24)
                .WithTag("TimerStatusInterruptEnable", 0, 8);

            Registers.ShifterStatusDMAEnable.Define(this)
                .WithReservedBits(8, 24)
                .WithTag("ShifterStatusDMAEnable", 0, 8);

            Registers.TimerStatusDMAEnable.Define(this)
                .WithReservedBits(8, 24)
                .WithTag("TimerStatusDMAEnable", 0, 8);

            Registers.ShifterState.Define(this)
                .WithReservedBits(3, 29)
                .WithTag("CurrentStatePointer", 0, 3);

            Registers.TriggerStatus.Define(this)
                .WithReservedBits(4, 28)
                .WithTag("ExternalTriggerStatusFlag", 0, 4);

            Registers.ExternalTriggerInterruptEnable.Define(this)
                .WithReservedBits(4, 28)
                .WithTag("ExternalTriggerInterruptEnable", 0, 4);

            Registers.PinStatus.Define(this)
                .WithTag("PinStatusFlag", 0, 32);

            Registers.PinInterruptEnable.Define(this)
                .WithTag("PinStatusInterruptEnable", 0, 32);

            Registers.PinRisingEdgeEnable.Define(this)
                .WithTag("PinRisingEdge", 0, 32);

            Registers.PinFallingEdgeEnable.Define(this)
                .WithTag("PinFallingEdge", 0, 32);

            Registers.PinOutputData.Define(this)
                .WithTag("OutputData", 0, 32);

            Registers.PinOutputEnable.Define(this)
                .WithTag("OutputEnable", 0, 32);

            Registers.PinOutputDisable.Define(this)
                .WithTag("OutputDisable", 0, 32);

            Registers.PinOutputClear.Define(this)
                .WithTag("OutputClear", 0, 32);

            Registers.PinOutputSet.Define(this)
                .WithTag("OutputSet", 0, 32);

            Registers.PinOutputToggle.Define(this)
                .WithTag("OutputToggle", 0, 32);

            Registers.ShifterControl0.DefineMany(this, BlockCount, (reg, index) => reg
                .WithReservedBits(27, 5)
                .WithTag("TimerSelect", 24, 3)
                .WithTaggedFlag("TimerPolarity", 23)
                .WithReservedBits(18, 5)
                .WithTag("ShifterPinConfiguration", 16, 2)
                .WithReservedBits(13, 3)
                .WithTag("ShifterPinSelect", 8, 5)
                .WithTaggedFlag("ShifterPinPolarity", 7)
                .WithReservedBits(3, 4)
                .WithTag("ShifterMode", 0, 3)
            );

            Registers.ShifterConfiguration0.DefineMany(this, BlockCount, (reg, index) => reg
                .WithReservedBits(21, 11)
                .WithTag("ParallelWidth", 16, 5)
                .WithReservedBits(13, 3)
                .WithTaggedFlag("ShifterSize", 12)
                .WithReservedBits(10, 2)
                .WithTaggedFlag("LateStore", 9)
                .WithTaggedFlag("InputSource", 8)
                .WithReservedBits(6, 2)
                .WithTag("ShifterStopBit", 4, 2)
                .WithReservedBits(2, 2)
                .WithTag("ShifterStartBit", 0, 2)
            );

            // The ShifterBuffer and 3 ShifterBuffer*Swapped registers have same layout.
            Registers.ShifterBuffer0.DefineMany(this, BlockCount * 4, (reg, index) => reg
                .WithTag("ShiftBuffer", 0, 32)
            );

            Registers.TimerControl0.DefineMany(this, BlockCount, (reg, index) => reg
                .WithReservedBits(30, 2)
                .WithTag("TriggerSelect", 24, 6)
                .WithTaggedFlag("TriggerPolarity", 23)
                .WithTaggedFlag("TriggerSource", 22)
                .WithReservedBits(18, 4)
                .WithTag("TimerPinConfiguration", 16, 2)
                .WithReservedBits(13, 3)
                .WithTag("TimerPinSelect", 8, 5)
                .WithTaggedFlag("TimerPinPolarity", 7)
                .WithTaggedFlag("TimerPinInputSelect", 6)
                .WithTaggedFlag("TimerOneTimeOperation", 5)
                .WithReservedBits(3, 2)
                .WithTag("TimerMode", 0, 3)
            );

            Registers.TimerConfiguration0.DefineMany(this, BlockCount, (reg, index) => reg
                .WithReservedBits(26, 6)
                .WithTag("TimerOutput", 24, 2)
                .WithReservedBits(23, 1)
                .WithTag("TimerDecrement", 20, 3)
                .WithReservedBits(19, 1)
                .WithTag("TimerReset", 16, 3)
                .WithReservedBits(15, 1)
                .WithTag("TimerDisable", 12, 3)
                .WithReservedBits(11, 1)
                .WithTag("TimerEnable", 8, 3)
                .WithReservedBits(6, 2)
                .WithTag("TimerStopBit", 4, 2)
                .WithReservedBits(2, 2)
                .WithTaggedFlag("TimerStartBit", 1)
                .WithReservedBits(0, 1)
            );

            Registers.TimerCompare0.DefineMany(this, BlockCount, (reg, index) => reg
                .WithReservedBits(16, 16)
                .WithTag("TimerCompareValue", 0, 16)
            );

            // The ShifterBuffer*Swapped registers have same layout.
            Registers.ShifterBuffer0NibbleByteSwapped.DefineMany(this, BlockCount * 6, (reg, index) => reg
                .WithTag("ShiftBuffer", 0, 32)
            );
        }

        private const uint BlockCount = 8;

        public enum Registers
        {
            VersionID = 0x0, // VERID
            Parameter = 0x4, // PARAM
            Control = 0x8, // CTRL
            PinState = 0xC, // PIN
            ShifterStatus = 0x10, // SHIFTSTAT
            ShifterError = 0x14, // SHIFTERR
            TimerStatus = 0x18, // TIMSTAT
            ShifterStatusInterruptEnable = 0x20, // SHIFTSIEN
            ShifterErrorInterruptEnable = 0x24, // SHIFTEIEN
            TimerInterruptEnable = 0x28, // TIMIEN
            ShifterStatusDMAEnable = 0x30, // SHIFTSDEN
            TimerStatusDMAEnable = 0x38, // TIMERSDEN
            ShifterState = 0x40, // SHIFTSTATE
            TriggerStatus = 0x48, // TRGSTAT
            ExternalTriggerInterruptEnable = 0x4C, // TRIGIEN
            PinStatus = 0x50, // PINSTAT
            PinInterruptEnable = 0x54, // PINIEN
            PinRisingEdgeEnable = 0x58, // PINREN
            PinFallingEdgeEnable = 0x5C, // PINFEN
            PinOutputData = 0x60, // PINOUTD
            PinOutputEnable = 0x64, // PINOUTE
            PinOutputDisable = 0x68, // PINOUTDIS
            PinOutputClear = 0x6C, // PINOUTCLR
            PinOutputSet = 0x70, // PINOUTSET
            PinOutputToggle = 0x74, // PINOUTTOG
            ShifterControl0 = 0x80, // SHIFTCTL0
            ShifterControl7 = 0x9C, // SHIFTCTL7
            ShifterConfiguration0 = 0x100, // SHIFTCFG0
            ShifterConfiguration7 = 0x11C, // SHIFTCFG7
            ShifterBuffer0 = 0x200, // SHIFTBUF0
            ShifterBuffer7 = 0x21C, // SHIFTBUF7
            ShifterBuffer0BitSwapped = 0x280, // SHIFTBUFBIS0
            ShifterBuffer7BitSwapped = 0x29C, // SHIFTBUFBIS7
            ShifterBuffer0ByteSwapped = 0x300, // SHIFTBUFBYS0
            ShifterBuffer7ByteSwapped = 0x31C, // SHIFTBUFBYS7
            ShifterBuffer0BitByteSwapped = 0x380, // SHIFTBUFBBS0
            ShifterBuffer7BitByteSwapped = 0x39C, // SHIFTBUFBBS7
            TimerControl0 = 0x400, // TIMCTL0
            TimerControl7 = 0x41C, // TIMCTL7
            TimerConfiguration0 = 0x480, // TIMCFG0
            TimerConfiguration7 = 0x49C, // TIMCFG7
            TimerCompare0 = 0x500, // TIMCMP0
            TimerCompare7 = 0x51C, // TIMCMP7
            ShifterBuffer0NibbleByteSwapped = 0x680, // SHIFTBUFNBS0
            ShifterBuffer7NibbleByteSwapped = 0x69C, // SHIFTBUFNBS7
            ShifterBuffer0HalfwordSwapped = 0x700, // SHIFTBUFHWS0
            ShifterBuffer7HalfwordSwapped = 0x71C, // SHIFTBUFHWS7
            ShifterBuffer0NibbleSwapped = 0x780, // SHIFTBUFNIS0
            ShifterBuffer7NibbleSwapped = 0x79C, // SHIFTBUFNIS7
            ShifterBuffer0OddEvenSwapped = 0x800, // SHIFTBUFOES0
            ShifterBuffer7OddEvenSwapped = 0x81C, // SHIFTBUFOES7
            ShifterBuffer0EvenOddSwapped = 0x880, // SHIFTBUFEOS0
            ShifterBuffer7EvenOddSwapped = 0x89C, // HIFTBUFEOS7
            ShifterBuffer0HalfWordByteSwapped = 0x900, // SHIFTBUFHBS0
            ShifterBuffer7HalfwordByteSwapped = 0x91C, // SHIFTBUFHBS7
        }
    }
}
