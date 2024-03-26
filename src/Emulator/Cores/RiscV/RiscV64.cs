//
// Copyright (c) 2010-2024 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Timers;
using Endianess = ELFSharp.ELF.Endianess;

namespace Antmicro.Renode.Peripherals.CPU
{
    public partial class RiscV64 : BaseRiscV
    {
        public RiscV64(IMachine machine, string cpuType, IRiscVTimeProvider timeProvider = null, uint hartId = 0, PrivilegeArchitecture privilegeArchitecture = PrivilegeArchitecture.Priv1_11, Endianess endianness = Endianess.LittleEndian, ulong? nmiVectorAddress = null, uint? nmiVectorLength = null, bool allowUnalignedAccesses = false, InterruptMode interruptMode = InterruptMode.Auto)
                : base(timeProvider, hartId, cpuType, machine, privilegeArchitecture, endianness, CpuBitness.Bits64, nmiVectorAddress, nmiVectorLength, allowUnalignedAccesses, interruptMode)
        {
        }

        public override string Architecture { get { return "riscv64"; } }

        public override string GDBArchitecture { get { return "riscv:rv64"; } }

        public override RegisterValue VLEN => VLENB * 8u;

        protected override byte MostSignificantBit => 63;

        private ulong BeforePCWrite(ulong value)
        {
            PCWritten();
            return value;
        }
    }
}
