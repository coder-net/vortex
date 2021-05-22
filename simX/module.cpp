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

  for (size_t wid = 0; wid < core_.warps_.size(); ++wid) {
    // round robin scheduling
    scheduled_warp = (scheduled_warp + 1) % core_.warps_.size();
    bool is_active = core_.warps_[scheduled_warp]->active();
    bool stalled = stalled_warps_[scheduled_warp];
    bool in_queue = executing_warps_[scheduled_warp];
    if (is_active && !stalled && !in_queue) {
      executing_warps_[scheduled_warp] = 1;
      core_.executing_queue_warps_[scheduled_warp] = 1;
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

//  assert(core_.inst_in_schedule_.wid == last_schedule_wid_ && "Not equal last schedules WID");
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
  , wp_execute_2_schedule_executed_wid_(ps.WPExecute2ScheduleExecutedWID)
  , wp_execute_2_schedule_stalled_wid_(ps.WPExecute2ScheduleStalledWID)
  , wp_writeback_2_schedule_unstalled_wid_(ps.WPWriteback2ScheduleUnstalledWID)
{}

void DecodeModule::clock_decode(const size_t cycle) {
  core_.writeback();
//  if (core_.inst_in_writeback_.stall_warp) {
//    wp_writeback_2_schedule_unstalled_wid_->write(core_.inst_in_writeback_.wid, cycle);
//    D(3, "Send unstalled WID: " << core_.inst_in_writeback_.wid << " to schedule");
//  }
  core_.execute();
//  if (core_.inst_in_execute_.stall_warp) {
//    wp_execute_2_schedule_stalled_wid_->write(core_.inst_in_execute_.wid, cycle);
//  }
  // wp_execute_2_schedule_executed_wid_->write(core_.inst_in_execute_.wid, cycle);
  core_.read();

  std::pair<int, Word> fetched;
  if (rp_fetch_2_decode_word_->read(&fetched, cycle)) {
    core_.inst_in_decode_.valid = true;
    core_.inst_in_decode_.wid = fetched.first;
    core_.inst_in_decode_.fetched = fetched.second;
  } else {
    core_.inst_in_decode_.valid = false;
  }

  core_.decode();

  if (core_.inst_in_decode_.stalled) {
    wp_decode_2_fetch_stall_->write(true, cycle);
    D(3, name() << " is stall, write for fetch module");
  }
}

bool DecodeModule::is_active(const size_t cycle) const {
  return !rp_fetch_2_decode_word_->is_empty(cycle);
}

} // namespace vortex