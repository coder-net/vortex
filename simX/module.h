#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "port.h"
#include "debug.h"

namespace vortex {

class Core;
class Instr;

class Module {
public:
  Module(const std::string& name);

  const std::string& name() const;
  std::string debug_info() const;

  virtual bool is_active(const size_t cycle) const = 0;


private:
  const std::string name_;
};


class ScheduleModule : public Module {
public:

  ScheduleModule(Core& core, PortsStorage& ps);

  void clock_schedule(const size_t cycle);
  bool is_active(const size_t cycle) const override;
  void reset();

private:
  Core& core_;
  std::shared_ptr<WritePort<int>> wp_schedule_2_fetch_;
  std::shared_ptr<ReadPort<bool>> rp_fetch_2_schedule_stall_;
  std::shared_ptr<ReadPort<int>> rp_execute_2_schedule_executed_wid_;
  std::shared_ptr<ReadPort<int>> rp_execute_2_schedule_stalled_wid_;
  std::shared_ptr<ReadPort<int>> rp_writeback_2_schedule_unstalled_wid_;

  WarpMask stalled_warps_;
  WarpMask executing_warps_;
  int last_schedule_wid_;
};


class FetchModule : Module {
public:
  FetchModule(Core& core, PortsStorage& ps);

  void clock_fetch(const size_t cycle);
  bool is_active(const size_t cycle) const override;

private:
  Core& core_;
  std::shared_ptr<ReadPort<int>> rp_schedule_2_fetch_wid_;
  std::shared_ptr<WritePort<bool>> wp_fetch_2_schedule_stall_;
  std::shared_ptr<ReadPort<bool>> rp_decode_2_fetch_stall_;
  std::shared_ptr<WritePort<std::pair<int, Word>>> wp_fetch_2_decode_word_;
};

class DecodeModule : Module {
public:
  DecodeModule(Core& core, PortsStorage& ps);

  void clock_decode(const size_t cycle);
  bool is_active(const size_t cycle) const override;

private:
  Core& core_;
  std::shared_ptr<ReadPort<std::pair<int, Word>>> rp_fetch_2_decode_word_;
  std::shared_ptr<WritePort<bool>> wp_decode_2_fetch_stall_;

  std::shared_ptr<WritePort<int>> wp_execute_2_schedule_executed_wid_;
  std::shared_ptr<WritePort<int>> wp_execute_2_schedule_stalled_wid_;
  std::shared_ptr<WritePort<int>> wp_writeback_2_schedule_unstalled_wid_;
};

}