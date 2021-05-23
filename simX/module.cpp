#include "module.h"

#include <sstream>

#include "core.h"
#include "instr.h"

namespace vortex {

Module::Module(const std::string& name) : name_(name)
{}

const std::string& Module::name() const {
  return name_;
}

std::string Module::debug_info() const {
  std::stringstream ss;
  ss << name() << " executing now";
  return ss.str();
}


ScheduleModule::ScheduleModule(Core& core, PortsStorage& ps)
        : Module("SCHEDULE_MODULE")
        , core_(core)
        , wp_schedule_2_fetch_(ps.WPSchedule2FetchWID_)
        , rp_fetch_2_schedule_stall_(ps.RPFetch2ScheduleStall_)
        , rp_execute_2_schedule_executed_wid_(ps.RPExecute2ScheduleExecutedWID)
        , rp_execute_2_schedule_stalled_wid_(ps.RPExecute2ScheduleStalledWID)
        , rp_writeback_2_schedule_unstalled_wid_(ps.RPWriteback2ScheduleUnstalledWID)
        , last_schedule_wid_(0)
{
  reset();
}

void ScheduleModule::clock_schedule(const size_t cycle) {
  D(3, debug_info());

  // check module stalling
  {
    bool is_stall = false;
    rp_fetch_2_schedule_stall_->read(&is_stall, cycle);
    if (is_stall) {
      D(3, "Module: " << name() << " is stalled");
      return;
    }
  }
  // setting states from other modules
  {
    int wid;
    if (rp_writeback_2_schedule_unstalled_wid_->read(&wid, cycle)) {
      stalled_warps_[wid] = 0;
      D(3, "Read unstalled WID: " << wid);
    }
    if (rp_execute_2_schedule_stalled_wid_->read(&wid, cycle)) {
      stalled_warps_[wid] = 1;
      D(3, "Read stalled WID: " << wid);
    }
    if (rp_execute_2_schedule_executed_wid_->read(&wid, cycle)) {
      executing_warps_[wid] = 0;
      D(3, "Read executed WID: " << wid);
    }
  }

  bool foundSchedule = false;
  int scheduled_warp = last_schedule_wid_;

  for (int wid = 0; wid < core_.arch().num_warps(); ++wid) {
    // round robin scheduling
    scheduled_warp = (scheduled_warp + 1) % core_.arch().num_warps();
    bool is_active = core_.warp(scheduled_warp).active();
    bool stalled = stalled_warps_[scheduled_warp];
    bool in_queue = executing_warps_[scheduled_warp];
    if (is_active && !stalled && !in_queue) {
      executing_warps_[scheduled_warp] = 1;
      foundSchedule = true;
      break;
    }
  }

  if (!foundSchedule) {
    D(3, "Schedule warp not found");
  } else {
    last_schedule_wid_ = scheduled_warp;
    wp_schedule_2_fetch_->write(scheduled_warp, cycle);
    D(3, "Schedule: WID=" << scheduled_warp);
  }
}

bool ScheduleModule::is_active(const size_t cycle) const {
  return !rp_execute_2_schedule_executed_wid_->is_empty(cycle)
      || !rp_execute_2_schedule_stalled_wid_->is_empty(cycle)
      || !rp_writeback_2_schedule_unstalled_wid_->is_empty(cycle)
      || !rp_fetch_2_schedule_stall_->is_empty(cycle);
}

void ScheduleModule::reset() {
  stalled_warps_.reset();
  executing_warps_.reset();
}


FetchModule::FetchModule(Core& core, PortsStorage& ps)
  : Module("FETCH_MODULE")
  , core_(core)
  , rp_schedule_2_fetch_wid_(ps.RPSchedule2FetchWID_)
  , wp_fetch_2_schedule_stall_(ps.WPFetch2ScheduleStall_)
  , rp_decode_2_fetch_stall_(ps.RPDecode2FetchStall)
  , wp_fetch_2_decode_word_(ps.WPFetch2DecodeWord)
{}

void FetchModule::clock_fetch(const size_t cycle) {
  D(3, debug_info());

  bool is_stall = false;
  rp_decode_2_fetch_stall_->read(&is_stall, cycle);
  if (is_stall) {
    D(3, "Module: " << name() << " is stalled");
    wp_fetch_2_schedule_stall_->write(true, cycle);
    return;
  }

  int wid;
  if (!rp_schedule_2_fetch_wid_->read(&wid, cycle)) {
    D(3, name() << ", not WID from schedule for " << cycle << " cycle");
    return; // exit
  } else {
    auto fetched = core_.icache_fetch(core_.warp(wid).getPC());
    wp_fetch_2_decode_word_->write(std::make_pair(wid, fetched), cycle);
    D(3, name() << ", WID: " << wid << " from schedule for " << cycle << " cycle");
  }
}

bool FetchModule::is_active(const size_t cycle) const {
  return !rp_schedule_2_fetch_wid_->is_empty(cycle)
      || !rp_decode_2_fetch_stall_->is_empty(cycle);
}

DecodeModule::DecodeModule(Core &core, PortsStorage &ps)
  : Module("DECODE_MODULE")
  , core_(core)
  , rp_fetch_2_decode_word_(ps.RPFetch2DecodeWord)
  , wp_decode_2_fetch_stall_(ps.WPDecode2FetchStall)
  , rp_read_2_decode_stall_(ps.RPRead2DecodeStall)
  , wp_decode_2_read_instr_(ps.WPDecode2ReadInstr)
{}

void DecodeModule::clock_decode(const size_t cycle) {
  D(3, debug_info());

  bool is_stall = false;
  rp_read_2_decode_stall_->read(&is_stall, cycle);
  if (is_stall) {
    D(3, "Module: " << name() << " is stalled");
    wp_decode_2_fetch_stall_->write(true, cycle);
    return;
  }

  std::pair<int, Word> fetched;
  if (!rp_fetch_2_decode_word_->read(&fetched, cycle)) {
    D(3, name() << ", not fetched info for " << cycle << " cycle");
    return; // exit
  } else {
    auto instr = core_.decoder().decode(fetched.second);
    D(3, "inst decoded");
    wp_decode_2_read_instr_->write(std::make_pair(fetched.first, std::move(instr)), cycle);
    D(3, name() << ", decoded instr for WID: " << fetched.first << " for " << cycle << " cycle");
  }
}

bool DecodeModule::is_active(const size_t cycle) const {
  return !rp_fetch_2_decode_word_->is_empty(cycle)
      || !rp_read_2_decode_stall_->is_empty(cycle);
}

ReadModule::ReadModule(Core &core, PortsStorage &ps)
  : Module("READ_MODULE")
  , core_(core)
  , rp_decode_2_read_instr_(ps.RPDecode2ReadInstr)
  , wp_read_2_decode_stall_(ps.WPRead2DecodeStall)
  , rp_execute_2_read_stall_(ps.RPExecute2ReadStall)
  , wp_read_2_execute_instr_(ps.WPRead2ExecuteInstr)
  , rp_writeback_2_read_reg_(ps.RPWriteback2ReadReg)
  , stalled_(false)
{
  in_use_iregs_.resize(core_.arch().num_warps(), 0);
  in_use_fregs_.resize(core_.arch().num_warps(), 0);
  in_use_vregs_.reset();
}

void ReadModule::clock_read(const size_t cycle) {
  D(3, debug_info());
  {
    bool is_stall = false;
    rp_execute_2_read_stall_->read(&is_stall, cycle);
    if (is_stall) {
      D(3, "Module: " << name() << " is stalled");
      wp_read_2_decode_stall_->write(true, cycle);
      return;
    }
  }
  {
    ReleasedMemRegInfo info;
    if (rp_writeback_2_read_reg_->read(&info, cycle)) {
      switch (info.reg_type) {
        case RegTypes::INTEGER:
          in_use_iregs_[info.wid][info.reg_num] = 0;
          break;
        case RegTypes::FLOAT:
          in_use_fregs_[info.wid][info.reg_num] = 0;
          break;
        case RegTypes::VECTOR:
          in_use_vregs_[info.reg_num] = 0;
          break;
        default:
          break;
      }
    }
  }

  std::pair<int, std::shared_ptr<Instr>> instr_info;
  if (!rp_decode_2_read_instr_->read(&instr_info, cycle)) {
    D(3, name() << ", not fetched info for " << cycle << " cycle");
    if (stalled_) {
      D(3, name() << ", registers not ready! Stall");
      wp_read_2_decode_stall_->write(true, cycle);
    }
    return; // exit
  } else {
    auto wid = instr_info.first;
    auto& instr = instr_info.second;

    auto regs_valid = core_.warp(wid).read(*instr, in_use_iregs_[wid], in_use_fregs_[wid], in_use_vregs_);

    // Check, if register for writeback will be used
//    bool in_use_regs = (p.used_iregs & in_use_iregs_[instr.first]) != 0
//                     || (p.used_fregs & in_use_fregs_[instr.first]) != 0
//                     || (p.used_vregs & in_use_vregs_) != 0;

    if (!regs_valid) {
      D(3, name() << ", registers not ready! Stall");
      stalled_ = true;
      wp_read_2_decode_stall_->write(true, cycle);
      return;
    }

    stalled_ = false;

    switch (instr->getRDType()) {
      case RegTypes::INTEGER:
        if (instr->getRDest())
          in_use_iregs_[wid][instr->getRDest()] = 1;
        break;
      case RegTypes::FLOAT:
        in_use_fregs_[wid][instr->getRDest()] = 1;
        break;
      case RegTypes::VECTOR:
        in_use_vregs_[instr->getRDest()] = 1;
        break;
      default:
        break;
    }

    wp_read_2_execute_instr_->write(std::make_pair(wid, instr), cycle);
    D(3, name() << " send instr to execute");
  }
}

bool ReadModule::is_active(const size_t cycle) const {
  return !rp_decode_2_read_instr_->is_empty(cycle)
      || !rp_writeback_2_read_reg_->is_empty(cycle)
      || !rp_execute_2_read_stall_->is_empty(cycle)
      || !rp_writeback_2_read_reg_->is_empty(cycle);
}


ExecuteModule::ExecuteModule(Core &core, PortsStorage &ps)
  : Module("EXECUTE_MODULE")
  , core_(core)
  , rp_read_2_execute_instr_(ps.RPRead2ExecuteInstr)
  , wp_execute_2_read_stall_(ps.WPExecute2ReadStall)
  , rp_writeback_2_execute_stall_(ps.RPWriteback2ExecuteStall)
  , wp_execute_2_writeback_result_(ps.WPExecute2WritebackResult)
  , wp_execute_2_schedule_executed_wid_(ps.WPExecute2ScheduleExecutedWID)
  , wp_execute_2_schedule_stalled_wid_(ps.WPExecute2ScheduleStalledWID)
{}

void ExecuteModule::clock_execute(const size_t cycle) {
  D(3, debug_info());

  {
    bool is_stall = false;
    rp_writeback_2_execute_stall_->read(&is_stall, cycle);
    if (is_stall) {
      D(3, "Module: " << name() << " is stalled");
      wp_execute_2_read_stall_->write(true, cycle);
      return;
    }
  }

  std::pair<int, std::shared_ptr<Instr>> instr_info;
  if (!rp_read_2_execute_instr_->read(&instr_info, cycle)) {
    D(3, name() << ", not fetched info for " << cycle << " cycle");
    return; // exit
  } else {
    auto wid = instr_info.first;
    auto& instr = instr_info.second;

    auto active_threads_b = core_.warp(wid).getActiveThreads();
    auto stall_warp = !core_.warp(wid).execute(*instr);
    auto active_threads_a = core_.warp(wid).getActiveThreads();

    wp_execute_2_schedule_executed_wid_->write(wid, cycle);

    core_.inc_insts(active_threads_a);

    if (active_threads_b != active_threads_a) {
      D(3, "** warp #" << wid << " active threads changed from " << active_threads_b << " to " << active_threads_a);
    }

    if (stall_warp) {
      wp_execute_2_schedule_stalled_wid_->write(wid, cycle);
    }

    wp_execute_2_writeback_result_->write(WritebackInfo(wid, instr, stall_warp), cycle);
  }
}

bool ExecuteModule::is_active(const size_t cycle) const {
  return !rp_read_2_execute_instr_->is_empty(cycle)
      || !rp_writeback_2_execute_stall_->is_empty(cycle);
}


WritebaskModule::WritebaskModule(Core &core, PortsStorage &ps)
  : Module("WRITEBACK_MODULE")
  , core_(core)
  , rp_execute_2_writeback_result_(ps.RPExecute2WritebackResult)
  , wp_writeback_2_execute_stall_(ps.WPWriteback2ExecuteStall)
  , wp_writeback_2_schedule_unstalled_wid_(ps.WPWriteback2ScheduleUnstalledWID)
  , wp_writeback_2_read_reg_(ps.WPWriteback2ReadReg)
{}

void WritebaskModule::clock_writeback(const size_t cycle) {
  D(3, debug_info());

  WritebackInfo info;
  if (!rp_execute_2_writeback_result_->read(&info, cycle)) {
    D(3, name() << ", not fetched info for " << cycle << " cycle");
    return; // exit
  } else {

    core_.warp(info.wid).writeback(*info.instr);

    wp_writeback_2_read_reg_->write(ReleasedMemRegInfo(
          info.wid, info.instr->getRDest(), info.instr->getRDType()
    ), cycle);

    if (info.stall_warp) {
      wp_writeback_2_schedule_unstalled_wid_->write(info.wid, cycle);
    }
  }
}

bool WritebaskModule::is_active(const size_t cycle) const {
  return !rp_execute_2_writeback_result_->is_empty(cycle);
}

} // namespace vortex