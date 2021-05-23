#include <iostream>
#include <iomanip>
#include <string.h>
#include <assert.h>
#include "types.h"
#include "util.h"
#include "archdef.h"
#include "mem.h"
#include "decode.h"
#include "core.h"
#include "debug.h"
#include "instr.h"

using namespace vortex;

Core::Core(const ArchDef &arch, Decoder &decoder, MemoryUnit &mem, Word id)
    : id_(id)
    , arch_(arch)
    , decoder_(decoder)
    , mem_(mem)
    , shared_mem_(1, SMEM_SIZE)
    , inst_in_schedule_("schedule")
    , inst_in_fetch_("fetch")
    , inst_in_decode_("decode")
    , inst_in_read_("read")
    , inst_in_execute_("execute")
    , inst_in_writeback_("writeback")
    , ports_()
    , schedule_module_(*this, ports_)
    , fetch_module_(*this, ports_)
    , decode_module_(*this, ports_)
    , read_module_(*this, ports_)
    , execute_module_(*this, ports_)
    , writeback_module_(*this, ports_) {

  csrs_.resize(arch_.num_csrs(), 0);

  fcsrs_.resize(arch_.num_warps(), 0);

  barriers_.resize(arch_.num_barriers(), 0);

  warps_.resize(arch_.num_warps());
  for (int i = 0; i < arch_.num_warps(); ++i) {
    warps_[i] = std::make_shared<Warp>(this, i);
  }

  this->clear();
}

void Core::clear() {
  for (auto& csr : csrs_) {
    csr = 0;
  }

  for (auto& fcsr : fcsrs_) {
    fcsr = 0;
  }

  for (auto& barrier : barriers_) {
    barrier.reset();
  }
  
  for (auto warp : warps_) {
    warp->clear();
  }

  steps_  = 0;
  insts_  = 0;
  loads_  = 0;
  stores_ = 0;

  warps_[0]->setTmask(0, true);
}

void Core::step() {
  D(3, "CYCLE: " << steps_);
  // schedule_module_.clock_schedule(steps_);
  schedule_module_.clock_schedule(steps_);
  fetch_module_.clock_fetch(steps_);
  decode_module_.clock_decode(steps_);
  read_module_.clock_read(steps_);
  execute_module_.clock_execute(steps_);
  writeback_module_.clock_writeback(steps_);

  steps_++;
}

Word Core::get_csr(Addr addr, int tid, int wid) {
  if (addr == CSR_FFLAGS) {
    return fcsrs_.at(wid) & 0x1F;
  } else if (addr == CSR_FRM) {
    return (fcsrs_.at(wid) >> 5);
  } else if (addr == CSR_FCSR) {
    return fcsrs_.at(wid);
  } else if (addr == CSR_WTID) {
    // Warp threadID
    return tid;
  } else if (addr == CSR_LTID) {
    // Core threadID
    return tid + (wid * arch_.num_threads());
  } else if (addr == CSR_GTID) {
    // Processor threadID
    return tid + (wid * arch_.num_threads()) + 
              (arch_.num_threads() * arch_.num_warps() * id_);
  } else if (addr == CSR_LWID) {
    // Core warpID
    return wid;
  } else if (addr == CSR_GWID) {
    // Processor warpID        
    return wid + (arch_.num_warps() * id_);
  } else if (addr == CSR_GCID) {
    // Processor coreID
    return id_;
  } else if (addr == CSR_NT) {
    // Number of threads per warp
    return arch_.num_threads();
  } else if (addr == CSR_NW) {
    // Number of warps per core
    return arch_.num_warps();
  } else if (addr == CSR_NC) {
    // Number of cores
    return arch_.num_cores();
  } else if (addr == CSR_INSTRET) {
    // NumInsts
    return insts_;
  } else if (addr == CSR_INSTRET_H) {
    // NumInsts
    return (Word)(insts_ >> 32);
  } else if (addr == CSR_CYCLE) {
    // NumCycles
    return (Word)steps_;
  } else if (addr == CSR_CYCLE_H) {
    // NumCycles
    return (Word)(steps_ >> 32);
  } else {
    return csrs_.at(addr);
  }
}

void Core::set_csr(Addr addr, Word value, int /*tid*/, int wid) {
  if (addr == CSR_FFLAGS) {
    fcsrs_.at(wid) = (fcsrs_.at(wid) & ~0x1F) | (value & 0x1F);
  } else if (addr == CSR_FRM) {
    fcsrs_.at(wid) = (fcsrs_.at(wid) & ~0xE0) | (value << 5);
  } else if (addr == CSR_FCSR) {
    fcsrs_.at(wid) = value & 0xff;
  } else {
    csrs_.at(addr) = value;
  }
}

void Core::barrier(int bar_id, int count, int warp_id) {
  auto& barrier = barriers_.at(bar_id);
  barrier.set(warp_id);
  if (barrier.count() < (size_t)count)    
    return;
  for (int i = 0; i < arch_.num_warps(); ++i) {
    if (barrier.test(i)) {
      warps_.at(i)->activate();
    }
  }
  barrier.reset();
}

Word Core::icache_fetch(Addr addr) {
  Word data;
  mem_.read(addr, &data, sizeof(Word), 0);
  return data;
}

Word Core::dcache_read(Addr addr, Size size) {
  ++loads_;
  Word data = 0;
#ifdef SM_ENABLE
  if ((addr >= (SHARED_MEM_BASE_ADDR - SMEM_SIZE))
   && ((addr + 3) < SHARED_MEM_BASE_ADDR)) {
     shared_mem_.read(addr & (SMEM_SIZE-1), &data, size);
     return data;
  }
#endif
  mem_.read(addr, &data, size, 0);
  return data;
}

void Core::dcache_write(Addr addr, Word data, Size size) {
  ++stores_;
#ifdef SM_ENABLE
  if ((addr >= (SHARED_MEM_BASE_ADDR - SMEM_SIZE))
   && ((addr + 3) < SHARED_MEM_BASE_ADDR)) {
     shared_mem_.write(addr & (SMEM_SIZE-1), &data, size);
     return;
  }
#endif
  mem_.write(addr, &data, size, 0);
}

bool Core::running() const {
  return schedule_module_.is_active((size_t)num_steps())
      || fetch_module_.is_active((size_t)num_steps())
      || decode_module_.is_active((size_t)num_steps())
      || read_module_.is_active((size_t)num_steps())
      || execute_module_.is_active((size_t)num_steps())
      || writeback_module_.is_active((size_t)num_steps());
}

void Core::printStats() const {
  std::cout << "Steps : " << steps_ << std::endl
            << "Insts : " << insts_ << std::endl
            << "Loads : " << loads_ << std::endl
            << "Stores: " << stores_ << std::endl;
}
