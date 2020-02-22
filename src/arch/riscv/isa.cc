/*
 * Copyright (c) 2016 RISC-V Foundation
 * Copyright (c) 2016 The University of Virginia
 * Copyright (c) 2020 Barkhausen Institut
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "arch/riscv/isa.hh"

#include <ctime>
#include <set>
#include <sstream>

#include "arch/riscv/interrupts.hh"
#include "arch/riscv/pagetable.hh"
#include "arch/riscv/registers.hh"
#include "base/bitfield.hh"
#include "cpu/base.hh"
#include "debug/Checkpoint.hh"
#include "debug/RiscvMisc.hh"
#include "params/RiscvISA.hh"
#include "sim/core.hh"
#include "sim/pseudo_inst.hh"

namespace RiscvISA
{

const char *MiscRegNames[] = {
    "PRV",
    "ISA",
    "VENDORID",
    "ARCHID",
    "IMPID",
    "HARTID",
    "STATUS",
    "IP",
    "IE",
    "CYCLE",
    "TIME",
    "INSTRET",
    "HPMCOUNTER03",
    "HPMCOUNTER04",
    "HPMCOUNTER05",
    "HPMCOUNTER06",
    "HPMCOUNTER07",
    "HPMCOUNTER08",
    "HPMCOUNTER09",
    "HPMCOUNTER10",
    "HPMCOUNTER11",
    "HPMCOUNTER12",
    "HPMCOUNTER13",
    "HPMCOUNTER14",
    "HPMCOUNTER15",
    "HPMCOUNTER16",
    "HPMCOUNTER17",
    "HPMCOUNTER18",
    "HPMCOUNTER19",
    "HPMCOUNTER20",
    "HPMCOUNTER21",
    "HPMCOUNTER22",
    "HPMCOUNTER23",
    "HPMCOUNTER24",
    "HPMCOUNTER25",
    "HPMCOUNTER26",
    "HPMCOUNTER27",
    "HPMCOUNTER28",
    "HPMCOUNTER29",
    "HPMCOUNTER30",
    "HPMCOUNTER31",
    "HPMEVENT03",
    "HPMEVENT04",
    "HPMEVENT05",
    "HPMEVENT06",
    "HPMEVENT07",
    "HPMEVENT08",
    "HPMEVENT09",
    "HPMEVENT10",
    "HPMEVENT11",
    "HPMEVENT12",
    "HPMEVENT13",
    "HPMEVENT14",
    "HPMEVENT15",
    "HPMEVENT16",
    "HPMEVENT17",
    "HPMEVENT18",
    "HPMEVENT19",
    "HPMEVENT20",
    "HPMEVENT21",
    "HPMEVENT22",
    "HPMEVENT23",
    "HPMEVENT24",
    "HPMEVENT25",
    "HPMEVENT26",
    "HPMEVENT27",
    "HPMEVENT28",
    "HPMEVENT29",
    "HPMEVENT30",
    "HPMEVENT31",
    "TSELECT",
    "TDATA1",
    "TDATA2",
    "TDATA3",
    "DCSR",
    "DPC",
    "DSCRATCH",

    "MEDELEG",
    "MIDELEG",
    "MTVEC",
    "MCOUNTEREN",
    "MSCRATCH",
    "MEPC",
    "MCAUSE",
    "MTVAL",
    "PMPCFG0",
    // pmpcfg1 rv32 only
    "PMPCFG2",
    // pmpcfg3 rv32 only
    "PMPADDR00",
    "PMPADDR01",
    "PMPADDR02",
    "PMPADDR03",
    "PMPADDR04",
    "PMPADDR05",
    "PMPADDR06",
    "PMPADDR07",
    "PMPADDR08",
    "PMPADDR09",
    "PMPADDR10",
    "PMPADDR11",
    "PMPADDR12",
    "PMPADDR13",
    "PMPADDR14",
    "PMPADDR15",

    "SEDELEG",
    "SIDELEG",
    "STVEC",
    "SCOUNTEREN",
    "SSCRATCH",
    "SEPC",
    "SCAUSE",
    "STVAL",
    "SATP",

    "UTVEC",
    "USCRATCH",
    "UEPC",
    "UCAUSE",
    "UTVAL",
    "FFLAGS",
    "FRM",
};

ISA::ISA(Params *p) : BaseISA(p)
{
    static_assert(
        sizeof(MiscRegNames) / sizeof(MiscRegNames[0]) == NumMiscRegs,
        "MiscRegNames not in sync with NumMiscRegs"
    );

    miscRegFile.resize(NumMiscRegs);
    clear();
}

const RiscvISAParams *
ISA::params() const
{
    return dynamic_cast<const Params *>(_params);
}

void ISA::clear()
{
    std::fill(miscRegFile.begin(), miscRegFile.end(), 0);

    miscRegFile[MISCREG_PRV] = PRV_M;
    miscRegFile[MISCREG_ISA] = (2ULL << MXL_OFFSET) | 0x14112D;
    miscRegFile[MISCREG_VENDORID] = 0;
    miscRegFile[MISCREG_ARCHID] = 0;
    miscRegFile[MISCREG_IMPID] = 0;
    miscRegFile[MISCREG_STATUS] = (2ULL << UXL_OFFSET) | (2ULL << SXL_OFFSET) |
                                  (1ULL << FS_OFFSET);
    miscRegFile[MISCREG_MCOUNTEREN] = 0x7;
    miscRegFile[MISCREG_SCOUNTEREN] = 0x7;
    // don't set it to zero; software may try to determine the supported
    // triggers, starting at zero. simply set a different value here.
    miscRegFile[MISCREG_TSELECT] = 1;
}

bool
ISA::hpmCounterEnabled(int misc_reg) const
{
    int hpmcounter = misc_reg - MISCREG_CYCLE;
    if (hpmcounter < 0 || hpmcounter > 31)
        panic("Illegal HPM counter %d\n", hpmcounter);
    int counteren;
    switch (readMiscRegNoEffect(MISCREG_PRV)) {
      case PRV_M:
        return true;
      case PRV_S:
        counteren = MISCREG_MCOUNTEREN;
        break;
      case PRV_U:
        counteren = MISCREG_SCOUNTEREN;
        break;
      default:
        panic("Unknown privilege level %d\n", miscRegFile[MISCREG_PRV]);
        return false;
    }
    return (miscRegFile[counteren] & (1ULL << (hpmcounter))) > 0;
}

RegVal
ISA::readMiscRegNoEffect(int misc_reg) const
{
    if (misc_reg > NumMiscRegs || misc_reg < 0) {
        // Illegal CSR
        panic("Illegal CSR index %#x\n", misc_reg);
        return -1;
    }
    DPRINTF(RiscvMisc, "Reading MiscReg %s (%d): %#llx.\n",
            MiscRegNames[misc_reg], misc_reg, miscRegFile[misc_reg]);
    return miscRegFile[misc_reg];
}

RegVal
ISA::readMiscReg(int misc_reg, ThreadContext *tc)
{
    switch (misc_reg) {
      case MISCREG_HARTID:
        return tc->contextId();
      case MISCREG_CYCLE:
        if (hpmCounterEnabled(MISCREG_CYCLE)) {
            DPRINTF(RiscvMisc, "Cycle counter at: %llu.\n",
                    tc->getCpuPtr()->curCycle());
            return tc->getCpuPtr()->curCycle();
        } else {
            warn("Cycle counter disabled.\n");
            return 0;
        }
      case MISCREG_TIME:
        if (hpmCounterEnabled(MISCREG_TIME)) {
            DPRINTF(RiscvMisc, "Wall-clock counter at: %llu.\n",
                    std::time(nullptr));
            return std::time(nullptr);
        } else {
            warn("Wall clock disabled.\n");
            return 0;
        }
      case MISCREG_INSTRET:
        if (hpmCounterEnabled(MISCREG_INSTRET)) {
            DPRINTF(RiscvMisc, "Instruction counter at: %llu.\n",
                    tc->getCpuPtr()->totalInsts());
            return tc->getCpuPtr()->totalInsts();
        } else {
            warn("Instruction counter disabled.\n");
            return 0;
        }
      case MISCREG_IP:
        {
            auto ic = dynamic_cast<RiscvISA::Interrupts *>(
                    tc->getCpuPtr()->getInterruptController(tc->threadId()));
            return ic->readIP();
        }
      case MISCREG_IE:
        {
            auto ic = dynamic_cast<RiscvISA::Interrupts *>(
                    tc->getCpuPtr()->getInterruptController(tc->threadId()));
            return ic->readIE();
        }
      case MISCREG_SEPC:
      case MISCREG_MEPC:
        {
            auto misa = readMiscRegNoEffect(MISCREG_ISA);
            RegVal val = readMiscRegNoEffect(misc_reg);
            // epc[0] is always 0
            val &= ~static_cast<Addr>(0x1);
            // if compressed instructions are disabled, epc[1] is set to 0
            if ((misa & ISA_EXT_C_MASK) == 0)
                val &= ~static_cast<Addr>(0x2);
            return val;
        }
      default:
        // Try reading HPM counters
        // As a placeholder, all HPM counters are just cycle counters
        if (misc_reg >= MISCREG_HPMCOUNTER03 &&
                misc_reg <= MISCREG_HPMCOUNTER31) {
            if (hpmCounterEnabled(misc_reg)) {
                DPRINTF(RiscvMisc, "HPM counter %d: %llu.\n",
                        misc_reg - MISCREG_CYCLE, tc->getCpuPtr()->curCycle());
                return tc->getCpuPtr()->curCycle();
            } else {
                warn("HPM counter %d disabled.\n", misc_reg - MISCREG_CYCLE);
                return 0;
            }
        }
        return readMiscRegNoEffect(misc_reg);
    }
}

void
ISA::setMiscRegNoEffect(int misc_reg, RegVal val)
{
    if (misc_reg > NumMiscRegs || misc_reg < 0) {
        // Illegal CSR
        panic("Illegal CSR index %#x\n", misc_reg);
    }
    DPRINTF(RiscvMisc, "Setting MiscReg %s (%d) to %#x.\n",
            MiscRegNames[misc_reg], misc_reg, val);
    miscRegFile[misc_reg] = val;
}

void
ISA::setMiscReg(int misc_reg, RegVal val, ThreadContext *tc)
{
    if (misc_reg >= MISCREG_CYCLE && misc_reg <= MISCREG_HPMCOUNTER31) {
        // Ignore writes to HPM counters for now
        warn("Ignoring write to %s.\n", CSRData.at(misc_reg).name);
    } else {
        switch (misc_reg) {
          case MISCREG_IP:
            {
                auto ic = dynamic_cast<RiscvISA::Interrupts *>(
                    tc->getCpuPtr()->getInterruptController(tc->threadId()));
                ic->setIP(val);
            }
            break;
          case MISCREG_IE:
            {
                auto ic = dynamic_cast<RiscvISA::Interrupts *>(
                    tc->getCpuPtr()->getInterruptController(tc->threadId()));
                ic->setIE(val);
            }
            break;
          case MISCREG_SATP:
            {
                // we only support bare and Sv39 mode; setting a different mode
                // shall have no effect (see page 64 in priv ISA manual)
                SATP cur_val = readMiscRegNoEffect(misc_reg);
                SATP new_val = val;
                if (new_val.mode != AddrXlateMode::BARE &&
                    new_val.mode != AddrXlateMode::SV39)
                    new_val.mode = cur_val.mode;
                setMiscRegNoEffect(misc_reg, new_val);
            }
            break;
          case MISCREG_TSELECT:
            {
                // we don't support debugging, so always set a different value
                // than written
                setMiscRegNoEffect(misc_reg, val + 1);
            }
            break;
          case MISCREG_ISA:
            {
                auto cur_val = readMiscRegNoEffect(misc_reg);
                // only allow to disable compressed instructions
                // if the following instruction is 4-byte aligned
                if ((val & ISA_EXT_C_MASK) == 0 &&
                    (tc->pcState().npc() & 0x3) != 0)
                    val |= cur_val & ISA_EXT_C_MASK;
                setMiscRegNoEffect(misc_reg, val);
            }
            break;
          case MISCREG_STATUS:
            {
                // these bits are hard-wired
                RegVal cur = readMiscRegNoEffect(misc_reg);
                val &= ~(STATUS_SXL_MASK | STATUS_UXL_MASK);
                val |= cur & (STATUS_SXL_MASK | STATUS_UXL_MASK);
                setMiscRegNoEffect(misc_reg, val);
            }
            break;
          default:
            setMiscRegNoEffect(misc_reg, val);
        }
    }
}

void
ISA::serialize(CheckpointOut &cp) const
{
    DPRINTF(Checkpoint, "Serializing Riscv Misc Registers\n");
    SERIALIZE_CONTAINER(miscRegFile);
}

void
ISA::unserialize(CheckpointIn &cp)
{
    DPRINTF(Checkpoint, "Unserializing Riscv Misc Registers\n");
    UNSERIALIZE_CONTAINER(miscRegFile);
}

}

RiscvISA::ISA *
RiscvISAParams::create()
{
    return new RiscvISA::ISA(this);
}
