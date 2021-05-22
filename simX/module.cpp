#include "module.h"

#include <sstream>

#include "core.h"

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

  bool is_stall = false;
  rp_fetch_2_schedule_stall_->read(&is_stall, cycle);
  if (is_stall) {
//    assert(core_.inst_in_schedule_.stalled && "Module is stalled, but pipeline is not");
    D(3, "Module: " << name() << " is stalled");
    return;
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

  D(3, "stalled warps stable: " << core_.stalled_warps_ << ", test: " << stalled_warps_);
  assert(core_.stalled_warps_ == stalled_warps_ && "Not equal stalled warps");
  assert(core_.executing_queue_warps_ == executing_warps_ && "Not equal executing warps");
//  assert(!core_.inst_in_schedule_.stalled && "Module is not stalled, but pipeline is");
//  assert(core_.inst_in_schedule_.valid && "Pipeline is invalid");

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

//  if (foundSchedule) {
//    core_.executing_queue_warps_[scheduled_warp] = 0;
//  }
//
//  core_.schedule();
//
//  assert(core_.inst_in_fetch_.valid == foundSchedule && "Diff in FoundSchedule");

  if (!foundSchedule) {
    D(3, "Schedule warp not found");
  } else {
    last_schedule_wid_ = scheduled_warp;
    wp_schedule_2_fetch_->write(scheduled_warp, cycle);
    D(3, "Schedule: wid=" << scheduled_warp);
  }

//  assert(core_.inst_in_schedule_.wid == last_schedule_wid_ && "Not equal last schedules WID");
}

bool ScheduleModule::is_active(const size_t cycle) const {
  return !rp_execute_2_schedule_executed_wid_->is_empty(cycle)
      || !rp_execute_2_schedule_stalled_wid_->is_empty(cycle)
      || !rp_writeback_2_schedule_unstalled_wid_->is_empty(cycle);
      //|| !rp_fetch_2_schedule_stall_->is_empty(cycle);
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
        , wp_execute_2_schedule_executed_wid_(ps.WPExecute2ScheduleExecutedWID)
        , wp_execute_2_schedule_stalled_wid_(ps.WPExecute2ScheduleStalledWID)
        , wp_writeback_2_schedule_unstalled_wid_(ps.WPWriteback2ScheduleUnstalledWID)
{}

void FetchModule::clock_fetch(const size_t cycle) {
  D(3, debug_info());

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
  core_.decode();

  int wid;
  if (!rp_schedule_2_fetch_wid_->read(&wid, cycle)) {
    core_.inst_in_fetch_.valid = false;
    D(3, name() << ", not WID from schedule for " << cycle << " cycle");
  } else {
    core_.inst_in_fetch_.valid = true;
    core_.inst_in_fetch_.wid = wid;
    D(3, name() << ", WID: " << wid << " from schedule for " << cycle << " cycle");
  }

  core_.fetch();

  if (core_.inst_in_fetch_.stalled) {
    wp_fetch_2_schedule_stall_->write(true, cycle);
    D(3, name() << " is stall, write for schedule module");
  }
}

bool FetchModule::is_active(const size_t cycle) const {
  return !rp_schedule_2_fetch_wid_->is_empty(cycle);
}

} // namespace vortex