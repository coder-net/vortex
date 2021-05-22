#pragma once

#include <assert.h>
#include <queue>
#include <memory>
#include <vector>
#include <string>
#include <iterator>
#include <utility>

#include "pipeline.h"
#include "types.h"

namespace vortex {

template <typename T>
struct State {
  State(T&& v, size_t c)
    : value(std::move(v))
    , cycle(c)
  {}

  T value;
  size_t cycle;
};

class Port {
public:
  Port(const std::string& name, const size_t latency)
    : name_(name)
    , latency_(latency)
    , last_cycle_(0)
  {}

  const std::string& name() const {
    return name_;
  }

  size_t latency() const {
    return latency_;
  }

protected:
  size_t last_cycle() const {
    return last_cycle_;
  }

  void update_last_cycle(const size_t cycle) {
    assert(last_cycle_ <= cycle && "New cycle < last cycle");
    last_cycle_ = cycle;
  }

private:
  const std::string name_;
  const size_t latency_;
  size_t last_cycle_;
};

template <typename T>
class WritePort;

template <typename T>
class ReadPort : public Port {
public:
  ReadPort(const std::string& name, const size_t latency) : Port(name, latency)
  {}

  bool is_ready(const size_t cycle) {
    cleanup_stale_info(cycle);
    return !state_queue_.empty() && state_queue_.front().cycle == cycle;
  }

  bool is_empty(const size_t cycle) {
    cleanup_stale_info(cycle);
    return state_queue_.empty();
  }

  T read(const size_t cycle) {
    assert(is_ready(cycle) && "ReadPort does not ready yet");
    return pop_front();
  }

  bool read(T* value, const size_t cycle) {
    if (!is_ready(cycle)) {
      return false;
    }
    *value = pop_front();
    return true;
  }

protected:
  friend class WritePort<T>;

  void cleanup_stale_info(const size_t cycle) {
    update_last_cycle(cycle);
    while (!state_queue_.empty() && state_queue_.front().cycle < cycle) {
      state_queue_.pop();
    }
  }

  T pop_front() {
    T p(std::move(state_queue_.front().value));
    state_queue_.pop();
    return p;
  }

  void emplace_back(T&& value, const size_t cycle) {
    auto cycle_to_read = cycle + latency();
    cleanup_stale_info(cycle);
    state_queue_.push(State<T>(std::move(value), cycle_to_read));
  }

private:
  std::queue<State<T>> state_queue_;
};


template <typename T>
class WritePort : public Port {
public:
  WritePort(const std::string& name, const size_t latency) : Port(name, latency)
  {}

  void write(T&& value, const size_t cycle) {
    basic_write(std::forward<T>(value), cycle);
  }

  void write(const T& value, const size_t cycle) {
    basic_write(T(value), cycle);
  }

  void add_reader(const std::shared_ptr<ReadPort<T>>& rp) {
    destinations_.emplace_back(rp);
  }

protected:
  friend class ReadPort<T>;

  void basic_write(T&& value, const size_t cycle) {
    for (size_t idx = 1; idx < destinations_.size(); ++idx) {
      destinations_[idx]->emplace_back(T(value), cycle);
    }
    // TODO: check that desc > 0 ?
    destinations_[0]->emplace_back(std::move(value), cycle);
  }

private:
  std::vector<std::shared_ptr<ReadPort<T>>> destinations_;
};


template <typename T>
std::shared_ptr<ReadPort<T>> make_read_port(const std::string& name, const size_t latency) {
  return std::make_shared<ReadPort<T>>(name, latency);
}

template <typename T>
std::shared_ptr<WritePort<T>> make_write_port(const std::string& name, const size_t latency) {
  return std::make_shared<WritePort<T>>(name, latency);
}


template <typename T>
struct WarpInfo {
  WarpInfo(const size_t wid, T&& i)
          : WID(wid)
          , info(i)
  {}

  size_t WID;
  T info;
};

// declarations
class Instr;

struct PortsStorage {
  // schedule -> fetch
//  static constexpr char* WPSchedule2FetchData = "WP_SCHEDULE_2_FETCH_DATA";
//  static constexpr char* RPFetch2ScheduleStall = "RP_FETCH_2_SCHEDULE_STALL";

  static constexpr size_t StallLatency = 1;
  static constexpr size_t TestLatency = 1;

  PortsStorage()
    // schedule -> fetch
    : RPFetch2ScheduleStall_(make_read_port<bool>("RP_FETCH_2_SCHEDULE_STALL", StallLatency))
    , WPFetch2ScheduleStall_(make_write_port<bool>("WP_FETCH_2_SCHEDULE_STALL", StallLatency))
    , RPSchedule2FetchWID_(make_read_port<int>("RP_SCHEDULE_2_FETCH_WID", TestLatency))
    , WPSchedule2FetchWID_(make_write_port<int>("WP_SCHEDULE_2_FETCH_WID", TestLatency))
    // fetch -> decode
    , RPDecode2FetchStall(make_read_port<bool>("RP_DECODE_2_FETCH_STALL", StallLatency))
    , WPDecode2FetchStall(make_write_port<bool>("WP_DECODE_2_FETCH_STALL", StallLatency))
    , RPFetch2DecodeWord(make_read_port<std::pair<int, Word>>("RP_FETCH_2_DECODE_WORD", TestLatency))
    , WPFetch2DecodeWord(make_write_port<std::pair<int, Word>>("WP_FETCH_2_DECODE_WORD", TestLatency))
    // execute -> schedule
    , RPExecute2ScheduleExecutedWID(make_read_port<int>("RP_EXECUTE_2_SCHEDULE_EXECUTED_WID", TestLatency))
    , WPExecute2ScheduleExecutedWID(make_write_port<int>("WP_EXECUTE_2_SCHEDULE_EXECUTED_WID", TestLatency))
    , RPExecute2ScheduleStalledWID(make_read_port<int>("RP_EXECUTE_2_SCHEDULE_STALLED_WID", TestLatency))
    , WPExecute2ScheduleStalledWID(make_write_port<int>("WP_EXECUTE_2_SCHEDULE_STALLED_WID", TestLatency))
    // writeback -> schedule
    , RPWriteback2ScheduleUnstalledWID(make_read_port<int>("RP_WRITEBACK_2_SCHEDULE_UNSTALLED_WID", TestLatency))
    , WPWriteback2ScheduleUnstalledWID(make_write_port<int>("WP_WRITEBACK_2_SCHEDULE_UNSTALLED_WID", TestLatency))
  {
    WPFetch2ScheduleStall_->add_reader(RPFetch2ScheduleStall_);
    WPSchedule2FetchWID_->add_reader(RPSchedule2FetchWID_);
    WPDecode2FetchStall->add_reader(RPDecode2FetchStall);
    WPFetch2DecodeWord->add_reader(RPFetch2DecodeWord);

    WPExecute2ScheduleExecutedWID->add_reader(RPExecute2ScheduleExecutedWID);
    WPExecute2ScheduleStalledWID->add_reader(RPExecute2ScheduleStalledWID);

    WPWriteback2ScheduleUnstalledWID->add_reader(RPWriteback2ScheduleUnstalledWID);
  }

  // fetch -> schedule
  std::shared_ptr<ReadPort<bool>> RPFetch2ScheduleStall_;
  std::shared_ptr<WritePort<bool>> WPFetch2ScheduleStall_;

  // schedule -> fetch
  std::shared_ptr<ReadPort<int>> RPSchedule2FetchWID_;
  std::shared_ptr<WritePort<int>> WPSchedule2FetchWID_;

  // decode -> fetch
  std::shared_ptr<ReadPort<bool>> RPDecode2FetchStall;
  std::shared_ptr<WritePort<bool>> WPDecode2FetchStall;

  // fetch -> decode
  std::shared_ptr<ReadPort<std::pair<int, Word>>> RPFetch2DecodeWord;
  std::shared_ptr<WritePort<std::pair<int, Word>>> WPFetch2DecodeWord;

//  // read -> decode
//  std::shared_ptr<ReadPort<bool>> RPRead2DecodeStall;
//  std::shared_ptr<WritePort<bool>> WPRead2DecodeStall;
//
//  // decode -> read
//  std::shared_ptr<ReadPort<std::pair<int, Instr>>> RPDecode2ReadInstr;
//  std::shared_ptr<WritePort<std::pair<int, Instr>>> WPDecode2ReadInstr;

  // execute -> schedule
  std::shared_ptr<ReadPort<int>> RPExecute2ScheduleExecutedWID;
  std::shared_ptr<WritePort<int>> WPExecute2ScheduleExecutedWID;
  std::shared_ptr<ReadPort<int>> RPExecute2ScheduleStalledWID;
  std::shared_ptr<WritePort<int>> WPExecute2ScheduleStalledWID;

  // writeback -> schedule
  std::shared_ptr<ReadPort<int>> RPWriteback2ScheduleUnstalledWID;
  std::shared_ptr<WritePort<int>> WPWriteback2ScheduleUnstalledWID;

};

} // namespace vortex