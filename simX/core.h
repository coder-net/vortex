#pragma once

#include <string>
#include <vector>
#include <list>
#include <stack>
#include <unordered_map>
#include <memory>
#include <set>

#include "debug.h"
#include "types.h"
#include "archdef.h"
#include "decode.h"
#include "mem.h"
#include "warp.h"
#include "pipeline.h"
#include "port.h"
#include "module.h"

namespace vortex {


class Core {
public:
  Core(const ArchDef &arch, Decoder &decoder, MemoryUnit &mem, Word id);

  void clear();

  bool running() const;

  void step();

  void step_old();

  void printStats() const;

  Word id() const {
    return id_;
  }

  Warp& warp(int i) {
    return *warps_.at(i);
  }

  Decoder& decoder() {
    return decoder_;
  }

  const ArchDef& arch() const {
    return arch_;
  }

  unsigned long num_insts() const {
    return insts_;
  }

  unsigned long num_steps() const {
    return steps_;
  } 

  Word getIRegValue(int reg) const {
    return warps_[0]->getIRegValue(reg);
  }

  Word get_csr(Addr addr, int tid, int wid);
  
  void set_csr(Addr addr, Word value, int tid, int wid);

  void barrier(int bar_id, int count, int warp_id);

  Word icache_fetch(Addr);

  Word dcache_read(Addr, Size);

  void dcache_write(Addr, Word, Size);

// private:

  void schedule();
  void fetch();
  void decode();
  void read();
  void execute();
  void writeback();
  
  
  std::vector<RegMask> in_use_iregs_;
  std::vector<RegMask> in_use_fregs_;
  RegMask in_use_vregs_;
  WarpMask stalled_warps_;
  WarpMask executing_queue_warps_;
  std::vector<std::shared_ptr<Warp>> warps_;  
  std::vector<WarpMask> barriers_;  
  std::vector<Word> csrs_;
  std::vector<Byte> fcsrs_;

  Word id_;
  const ArchDef &arch_;
  Decoder &decoder_;
  MemoryUnit &mem_;
#ifdef SM_ENABLE
  RAM shared_mem_;
#endif

  Pipeline inst_in_schedule_;
  Pipeline inst_in_fetch_;
  Pipeline inst_in_decode_;
  Pipeline inst_in_read_;
  Pipeline inst_in_execute_;
  Pipeline inst_in_writeback_;

  uint64_t steps_;
  uint64_t insts_;
  uint64_t loads_;
  uint64_t stores_;

  PortsStorage ports_;
  // modules
  ScheduleModule schedule_module_;
  FetchModule fetch_module_;
  DecodeModule decode_module_;
  ReadModule read_module_;
  ExecuteModule execute_module_;
  WritebaskModule writeback_module_;
};

} // namespace vortex