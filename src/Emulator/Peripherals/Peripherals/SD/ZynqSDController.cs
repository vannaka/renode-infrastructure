//
// Copyright (c) 2010-2018 Antmicro
// Copyright (c) 2011-2015 Realtime Embedded
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using System;
using System.Collections.Generic;
using System.Linq;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Storage;
using Antmicro.Renode.Utilities;
using System.IO;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Core.Structure.Registers;
using static Antmicro.Renode.Utilities.BitHelper;

namespace Antmicro.Renode.Peripherals.SD
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public sealed class ZynqSDController : NullRegistrationPointPeripheralContainer<SDCard>, IDoubleWordPeripheral, IProvidesRegisterCollection<DoubleWordRegisterCollection>, IKnownSize, IDisposable
    {        
        public ZynqSDController(IMachine machine) : base(machine)
        {
            IRQ = new GPIO();
            sysbus = machine.GetSystemBus(this);
            irqManager = new InterruptManager<Interrupts>(this);
            internalBuffer = new Queue<byte>();
            
            RegistersCollection = new DoubleWordRegisterCollection(this);
            InitializeRegisters();
        }
    
        public uint ReadDoubleWord(long offset)
        {
            return RegistersCollection.Read(offset);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            RegistersCollection.Write(offset, value);
        }

        public override void Reset()
        {
            RegisteredPeripheral?.Reset();
            RegistersCollection.Reset();
            irqManager.Reset();
            internalBuffer.Clear();
            bytesRead = 0;
        }

        public void Dispose()
        {
            RegisteredPeripheral?.Dispose();
        }

        private void InitializeRegisters()
        {
            var responseFields = new IValueRegisterField[4];

            Registers.BlockSizeBlockCount.Define(this)
                .WithValueField(0, 12, out blockSizeField, name: "Transfer Block Size")
                .WithTag("Host SDMA Buffer Size", 12, 3)
                .WithValueField(16, 16, out blockCountField, name: "Block Count For Current Transfer")
            ;

            Registers.Argument.Define(this)
                .WithValueField(0, 32, out commandArgumentField, name: "Command Argument")
            ;

            Registers.CommandTransferMode.Define(this)
                .WithFlag(0, out isDmaEnabled, name: "DMA Enable")
                .WithFlag(1, out isblockCountEnabled, name: "Block Count Enable")
                .WithTag("Auto CMD12 Enable (ACE)", 2, 2)
                .WithTag("Data Transfer Direction Select (DTDS)", 4, 1)
                .WithEnumField(16, 2, out responseTypeSelectField, name: "Response Type Select (RTS)")
                .WithTag("Command CRC Check Enable (CRCCE)", 19, 1)
                .WithTag("Command Index Check Enable (CICE)", 20, 1)
                .WithTag("Data Present Select (DPS)", 21, 1)
                .WithEnumField(24, 6, out commandIndex, name: "Command Index (CI)")
                .WithWriteCallback((_, val) =>
                {
                    var sdCard = RegisteredPeripheral;
                    if(sdCard == null)
                    {
                        this.Log(LogLevel.Warning, "Tried to send a command, but no SD card is currently attached");
                        return;
                    }

                    var commandResult = sdCard.HandleCommand((uint)commandIndex.Value, (uint)commandArgumentField.Value);
                    switch(responseTypeSelectField.Value)
                    {
                        case ResponseType.NoResponse:
                            if(commandResult.Length != 0)
                            {
                                this.Log(LogLevel.Warning, "Expected no response, but {0} bits received", commandResult.Length);
                                return;
                            }
                            break;
                        case ResponseType.Response136Bits:
                            // our response does not contain 8 bits:
                            // * start bit
                            // * transmission bit
                            // * command index / reserved bits (6 bits)
                            if(commandResult.Length != 128)
                            {
                                this.Log(LogLevel.Warning, "Unexpected a response of length 128 bits (excluding control bits), but {0} received", commandResult.Length);
                                return;
                            }
                            // the following bits are considered a part of returned register, but are not included in the response buffer:
                            // * CRC7 (7 bits)
                            // * end bit
                            // that's why we are skipping the initial 8-bits
                            responseFields[0].Value = commandResult.AsUInt32(8);
                            responseFields[1].Value = commandResult.AsUInt32(40);
                            responseFields[2].Value = commandResult.AsUInt32(72);
                            responseFields[3].Value = commandResult.AsUInt32(104, 24);
                            break;
                        case ResponseType.Response48Bits:
                        case ResponseType.Response48BitsWithBusy:
                            // our response does not contain 16 bits:
                            // * start bit
                            // * transmission bit
                            // * command index / reserved bits (6 bits)
                            // * CRC7 (7 bits)
                            // * end bit
                            if(commandResult.Length != 32)
                            {
                                this.Log(LogLevel.Warning, "Expected a response of length {0} bits (excluding control bits and CRC), but {1} received", 32, commandResult.Length);
                                return;
                            }
                            responseFields[0].Value = commandResult.AsUInt32();
                            break;
                        default:
                            this.Log(LogLevel.Warning, "Unexpected response type selected: {0}. Ignoring the command response.", responseTypeSelectField.Value);
                            return;
                    }

                    ProcessCommand(sdCard, commandIndex.Value);

                    irqManager.SetInterrupt(Interrupts.BufferWriteReady);
                    irqManager.SetInterrupt(Interrupts.BufferReadReady);
                    irqManager.SetInterrupt(Interrupts.CommandComplete);
                })
            ;

            Registers.Response0.Define(this)
                .WithValueField(0, 32, out responseFields[0], FieldMode.Read, name: "Response Register #0 (RESP0)")
            ;

            Registers.Response1.Define(this)
                .WithValueField(0, 32, out responseFields[1], FieldMode.Read, name: "Response Register #1 (RESP1)")
            ;

            Registers.Response2.Define(this)
                .WithValueField(0, 32, out responseFields[2], FieldMode.Read, name: "Response Register #2 (RESP2)")
            ;

            Registers.Response3.Define(this)
                .WithValueField(0, 32, out responseFields[3], FieldMode.Read, name: "Response Register #3 (RESP3)")
            ;

            Registers.DataBuffer.Define(this).WithValueField(0, 32, name: "Buffer Data Port (BDP)",
                valueProviderCallback: _ =>
                {
                    var sdCard = RegisteredPeripheral;
                    if(sdCard == null)
                    {
                        this.Log(LogLevel.Warning, "Tried to read data, but no SD card is currently attached");
                        return 0;
                    }
                    if(isDmaEnabled.Value)
                    {
                        this.Log(LogLevel.Warning, "Tried to read data in DMA mode from register that does not support it");
                        return 0;
                    }
                    if(!internalBuffer.Any())
                    {
                        return 0;
                    }
                    return ReadBuffer();
                },
                writeCallback: (_, value) =>
                {
                    var sdCard = RegisteredPeripheral;
                    if(sdCard == null)
                    {
                        this.Log(LogLevel.Warning, "Tried to write data, but no SD card is currently attached");
                        return;
                    }
                    if(isDmaEnabled.Value)
                    {
                        this.Log(LogLevel.Warning, "Tried to write data in DMA mode to register that does not support it");
                        return;
                    }
                    WriteBuffer(sdCard, BitConverter.GetBytes((uint)value));
                })
            ;

            Registers.PresentState.Define(this)
                .WithFlag(0, FieldMode.Read, name: "Command Inhibit CMD (CICMD)")
                .WithFlag(1, FieldMode.Read, name: "Command Inhibit DAT (CIDAT)") // as sending a command is instantienous those two bits will probably always be 0
                                                                                  // ...
                .WithFlag(10, FieldMode.Read, name: "Buffer Write Enable (BWE)", valueProviderCallback: _ => true)
                .WithFlag(11, FieldMode.Read, name: "Buffer Read Enable (BRE)", valueProviderCallback: _ => RegisteredPeripheral != null && internalBuffer.Any())
                .WithFlag(16, FieldMode.Read, name: "Card Inserted (CI)", valueProviderCallback: _ => RegisteredPeripheral != null)
                .WithFlag(17, FieldMode.Read, name: "Card State Stable (CSS)", valueProviderCallback: _ => true)
                .WithFlag(18, FieldMode.Read, name: "Card Detect Pin Level (CDSL)", valueProviderCallback: _ => RegisteredPeripheral != null)
            ;

            Registers.HostControl1.Define(this)
                .WithFlag(8, name: "SD Bus Power")
            ;

            Registers.HostControl2.Define(this)
                .WithFlag(1, FieldMode.Read, name: "Internal Clock Stable (ICS)", valueProviderCallback: _ => true)
                .WithTag("Software Reset For CMD Line (SRCMD)", 25, 1)
                .WithTag("Software Reset For DAT Line (SRDAT)", 26, 1)
            ;

            Registers.ErrorNormalInterruptStatus.Bind(this, irqManager.GetRegister<DoubleWordRegister>(
                valueProviderCallback: (irq, _) => irqManager.IsSet(irq) && IsBitSet((uint)intStatusEnable.Value, (byte)irq),
                writeCallback: (irq, prev, curr) => { if(curr) irqManager.ClearInterrupt(irq); } ))
            ;

            // Enables corresponding status bit in ErrorNormalInterruptStatus register regardless of 
            //  ErrorNormalSignalEnable bit.
            Registers.ErrorNormalStatusEnable.Define(this)
                .WithValueField(0, 32, out intStatusEnable, name: "SD Interrupts status enable register"  )
            ;

            // If set, the interrupt will signal the external interrupt controller.
            Registers.ErrorNormalSignalEnable.Bind(this, irqManager.GetRegister<DoubleWordRegister>(
                valueProviderCallback: (irq, _) => irqManager.IsEnabled(irq),
                writeCallback: (irq, _, curr) =>
                {
                    if(curr)
                    {
                        irqManager.EnableInterrupt(irq, curr);
                    }
                    else
                    {
                        irqManager.DisableInterrupt(irq);
                    }
                }))
            ;

            Registers.Capabilities.Define(this)
                // these fields must return non-zero values in order for u-boot to boot
                // .WithValueField(0, 6, FieldMode.Read, valueProviderCallback: _ => 4, name: "Timeout clock frequency (TCS)")
                .WithFlag(7, FieldMode.Read, valueProviderCallback: _ => true, name: "Timeout clock unit (TCU)")
                // .WithValueField(8, 8, FieldMode.Read, valueProviderCallback: _ => 1, name: "Base Clock Frequency For SD Clock (BCSDCLK)")
                .WithFlag(19, FieldMode.Read, valueProviderCallback: _ => true, name: "ADMA2 Support")
                .WithFlag(24, FieldMode.Read, valueProviderCallback: _ => true, name: "Voltage Support 3.3V (VS33)")
            ;

            Registers.ADMASystemAddress.Define(this)
                .WithValueField(0, 32, out admaSystemAddress, name: "ADMA System Address")
            ;
        }

        private void ProcessCommand(SDCard sdCard, SDCardCommand command)
        {
            switch(command)
            {
                case SDCardCommand.CheckSwitchableFunction:
                    internalBuffer.EnqueueRange(sdCard.ReadSwitchFunctionStatusRegister());
                    break;
                case SDCardCommand.SendInterfaceConditionCommand:
                    internalBuffer.EnqueueRange(sdCard.ReadExtendedCardSpecificDataRegister());
                    break;
                case SDCardCommand.ReadSingleBlock:
                case SDCardCommand.SendOperatingConditionRegister_ACMD41:
                case SDCardCommand.SendSDConfigurationRegister_ACMD51:
                    ReadCard(sdCard, (uint)blockSizeField.Value);
                    break;
                case SDCardCommand.ReadMultipleBlocks:
                    ReadCard(sdCard, (uint)(blockCountField.Value * blockSizeField.Value));
                    break;
                case SDCardCommand.WriteSingleBlock:
                    WriteCard(sdCard, (uint)blockSizeField.Value);
                    break;
                case SDCardCommand.WriteMultipleBlocks:
                    WriteCard(sdCard, (uint)(blockCountField.Value * blockSizeField.Value));
                    break;
            }
        }


        private enum ADMAAction
        {
            NOP = 0b000,
            RSV = 0b010,
            Transfer = 0b100,
            Link = 0b110
        }

        private void DoAdma(SDCard sdCard, bool isSDRead)
        {
            ulong dmaDescriptor;
            bool valid, isEnd, doInt;
            ADMAAction action;
            uint sysAddr;
            ushort length, totalLength = 0;
            
            while(true)
            {
                dmaDescriptor = sysbus.ReadQuadWord(admaSystemAddress.Value);
                valid = IsBitSet(dmaDescriptor, 1);
                isEnd = IsBitSet(dmaDescriptor, 2);
                doInt = IsBitSet(dmaDescriptor, 3);
                action = (ADMAAction)GetValue(dmaDescriptor, 3, 3);
                sysAddr = (uint)GetValue(dmaDescriptor, 32, 32);
                length = (ushort)GetValue(dmaDescriptor, 16, 16);
                if(0 == length) length = ushort.MaxValue;

                if(!valid)
                {
                    this.Log(LogLevel.Warning, "Invalid ADMA descriptor: 0x{X:8}: 0x{X:8}", admaSystemAddress.Value, dmaDescriptor);
                    irqManager.SetInterrupt(Interrupts.ADMAError);
                    // When this interrupt fires, the DMA engine is supposed to pause the transfer in it's current state such that
                    // the host device can fix up the error and resume the transfer. The current design does not allow for this.
                    // Workaround: Don't write shit software. (There might be a valid reason to craft a bad descriptor, idk)
                    return;
                }

                // Inc desc pointer here so that if ADMAError int happens, the Host Driver will read the correct value.
                admaSystemAddress.Value += 64;

                switch (action)
                {   
                    case ADMAAction.NOP:
                    case ADMAAction.RSV:
                        // Nothing to do
                        break;

                    case ADMAAction.Link:
                        // Link to another descriptor table.
                        admaSystemAddress.Value = sysAddr;
                        break;

                    case ADMAAction.Transfer:
                        if(isSDRead)
                        {
                            var data = sdCard.ReadData(length);
                            sysbus.WriteBytes(data, sysAddr);
                        }
                        else
                        {
                            var data = sysbus.ReadBytes(sysAddr, length);
                            sdCard.WriteData(data);
                        }

                        totalLength += length;
                        break;

                    default:
                        this.Log(LogLevel.Warning, "Bad ADMA descriptor action: {0}", action);
                        irqManager.SetInterrupt(Interrupts.ADMAError);
                        return;
                }
                
                if(isEnd)
                {
                    if(isblockCountEnabled.Value && (totalLength != (ushort)(blockCountField.Value * blockSizeField.Value)))
                        this.Log(LogLevel.Warning, "Sum of ADMA descriptor lengths not equal to Block Count * Block Size! {0} != {1}", totalLength, blockCountField.Value * blockSizeField.Value);
                    
                    // NOTE: It is optional to assert the DMAInterrupt on the last descriptor before asserting the TransferComplete interrupt.
                    break;
                }
                else if(doInt)
                {
                    // NOTE this will do int for NOP action. I can't tell what the intended behavior is.
                    irqManager.SetInterrupt(Interrupts.DMAInterrupt);
                }
            }
        }

        private void ReadCard(SDCard sdCard, uint size)
        {
            if(isDmaEnabled.Value)
            {
                DoAdma(sdCard, true);
                Machine.LocalTimeSource.ExecuteInNearestSyncedState(_ =>
                {
                    this.InfoLog("SD INT Transfer Complete (read)");
                    irqManager.SetInterrupt(Interrupts.TransferComplete);
                });
            }
            else
            {
                internalBuffer.EnqueueRange(sdCard.ReadData(size));
            }
        }

        private void WriteCard(SDCard sdCard, uint size)
        {
            if(isDmaEnabled.Value)
            {
                DoAdma(sdCard, false);
            }
            else
            {
                var bytes = new byte[size];

                if(internalBuffer.Count < size)
                {
                    this.Log(LogLevel.Warning, "Could not write {0} bytes to SD card, writing {1} bytes instead.", size, internalBuffer.Count);
                    size = (uint)internalBuffer.Count;
                }
                bytes = internalBuffer.DequeueRange((int)size);
                sdCard.WriteData(bytes);
            }

            Machine.LocalTimeSource.ExecuteInNearestSyncedState(_ =>
            {
                this.InfoLog("SD INT Transfer Complete (write)");
                irqManager.SetInterrupt(Interrupts.TransferComplete);
            });
        }

        private uint ReadBuffer()
        {
            var internalBytes = internalBuffer.DequeueRange(4);
            bytesRead += (uint)internalBytes.Length;
            irqManager.SetInterrupt(Interrupts.BufferReadReady);
            if(bytesRead == (blockCountField.Value * blockSizeField.Value)|| !internalBuffer.Any())
            {
                irqManager.SetInterrupt(Interrupts.TransferComplete);
                bytesRead = 0;
                // If we have read the exact amount of data we wanted, we can clear the buffer from any leftovers.
                internalBuffer.Clear();
            }
            return internalBytes.ToUInt32Smart();
        }

        private void WriteBuffer(SDCard sdCard, byte[] data)
        {
            var limit = (uint)(blockCountField.Value * blockSizeField.Value);
            internalBuffer.EnqueueRange(data);
            if(internalBuffer.Count < limit)
            {
                return;
            }
            sdCard.WriteData(internalBuffer.DequeueAll());
            irqManager.SetInterrupt(Interrupts.TransferComplete);
        }

        [IrqProvider]
        public GPIO IRQ { get; private set; }

        public long Size => 0x100;

        public DoubleWordRegisterCollection RegistersCollection { get; }

        // Register field variables
        private IFlagRegisterField isDmaEnabled;
        private IFlagRegisterField isblockCountEnabled;
        private IValueRegisterField blockSizeField;
        private IValueRegisterField blockCountField;
        private IValueRegisterField commandArgumentField;
        private IValueRegisterField admaSystemAddress;
        private IValueRegisterField intStatusEnable;
        private IEnumRegisterField<SDCardCommand> commandIndex;
        private IEnumRegisterField<ResponseType> responseTypeSelectField;

        private uint bytesRead;
        private Queue<byte> internalBuffer;

        private readonly IBusController sysbus;
        private readonly InterruptManager<Interrupts> irqManager;

        // private UInt64 dmaDescriptor;
        
        // The zynq has an SD controller that is compliant with the
        //  SD Host Controller Specification v2.0.
        private enum Registers
        {
            // [0x00 - 0x0F] - SD Command Generation
            // SDMASystemAddress = 0x00,
            BlockSizeBlockCount = 0x04,
            Argument = 0x08,
            CommandTransferMode = 0xC,

            // [0x10 - 0x1F] - Response
            Response0 = 0x10,
            Response1 = 0x14,
            Response2 = 0x18,
            Response3 = 0x1C,

            // [0x20-0x23] - Buffer Data Port
            DataBuffer = 0x20,

            // [0x24-0x2F] - Host Control 1 and Others
            PresentState = 0x24,
            HostControl1 = 0x28, // Driver enables DMA with DMA_SEL reg here.
            HostControl2 = 0x2C,

            // [0x30-0x3D] - Interrupt Controls
            ErrorNormalInterruptStatus = 0x30,
            ErrorNormalStatusEnable = 0x34, // enables irq status bits in ErrorNormalInterruptStatus register.
            ErrorNormalSignalEnable = 0x38, // enables irq to signal the external interrupt controller.
            // AutoCMD12ErrorStatus = 0x3C,

            // [0x40-0x4F] - Capabilities
            Capabilities = 0x40,
            // MaxCurrentCapabilities = 0x48,

            // [0x50-0x53] - Force Event
            // ForceEventAutoCMD12ErrorInterruptError = 0x50,

            // [0x54-0x5F] - ADMA2
            // ADMAErrorStatus = 0x54,
            ADMASystemAddress = 0x58,

            // [0x60-0x67] - ZYNQ specific features
            // BootTimoutControl = 0x60,
            // DebugSelection = 0x64,
            
            // [0xF0-0xFF] - Common Area
            // SPIInterruptSupport = 0xF0,
            // SlotInterruptStatus_HostControlVersion = 0xFC
        }

        private enum ResponseType
        {
            NoResponse = 0,
            Response136Bits = 1,
            Response48Bits = 2,
            Response48BitsWithBusy = 3
        }

        private enum Interrupts
        {
            CommandComplete = 0,
            TransferComplete = 1,
            BlockGapEvent = 2,
            DMAInterrupt = 3,
            BufferWriteReady = 4,
            BufferReadReady = 5,
            CardIsertion = 6,
            CardRemoval = 7,
            CardInterrupt = 8,
            BootAckRcv = 9,
            BootTerminateInerrupt = 10,
            // [11:14] Reserved
            ErrorInterrupt = 15,
            CommandTimeoutError = 16,
            CommandCRCError = 17,
            CommandEndBitError = 18,
            CommandIndexError = 19,
            DataTimeoutError = 20,
            DataCRCError = 21,
            DataEndBitError = 22,
            CurrentLimitError = 23,
            AutoCMDError = 24,
            ADMAError = 25,
            // [26:27] Reserved
            TargetResponseError = 28,
            CeataErrorStatus = 29,
            // [30:31] Reserved
        }

        private enum SDCardCommand
        {
            CheckSwitchableFunction = 6,
            SendInterfaceConditionCommand = 8,
            ReadSingleBlock = 17,
            ReadMultipleBlocks = 18,
            WriteSingleBlock = 24,
            WriteMultipleBlocks = 25,

            // These application commands behave like CMD17 - ReadSingleBlock.
            SendOperatingConditionRegister_ACMD41 = 41,
            SendSDConfigurationRegister_ACMD51 = 51,
        }
    }
}

