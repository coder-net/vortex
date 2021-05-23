#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <assert.h>

#include <sstream>

#include "util.h"
#include "instr.h"
#include "core.h"

using namespace vortex;

Warp::Warp(Core *core, Word id)
    : id_(id)
    , core_(core) {
  iRegFile_.resize(core_->arch().num_threads(), std::vector<Word>(core_->arch().num_regs(), 0));
  fRegFile_.resize(core_->arch().num_threads(), std::vector<Word>(core_->arch().num_regs(), 0));
  vRegFile_.resize(core_->arch().num_regs(), std::vector<Byte>(core_->arch().vsize(), 0));
  this->clear();
}

void Warp::clear() {
  D(3, "Warp::clear called");
  PC_ = STARTUP_ADDR;
  tmask_.reset();
  active_ = false;
}

int Warp::getNumThreads() const {
  return core_->arch().num_threads();
}

bool Warp::execute(Instr& instr) {
  assert(tmask_.any() && "Warp::execute");

  D(3, "Step: wid=" << id_ << ", PC=0x" << std::hex << PC_);

  auto result = this->executing(instr);

  // At Debug Level 3, print debug info after each instruction.
  D(4, "Register state:");
  for (int i = 0; i < core_->arch().num_regs(); ++i) {
    DPN(4, "  %r" << std::setfill('0') << std::setw(2) << std::dec << i << ':');
    for (int j = 0; j < core_->arch().num_threads(); ++j) {
      DPN(4, ' ' << std::setfill('0') << std::setw(8) << std::hex << iRegFile_[j][i] << std::setfill(' ') << ' ');
    }
    DPN(4, std::endl);
  }

  DPH(3, "Thread mask:");
  for (int i = 0; i < core_->arch().num_threads(); ++i)
    DPN(3, " " << tmask_[i]);
  DPN(3, "\n");

  return result;
}

bool Warp::read(Instr& instr, const RegMask& ireg_used, const RegMask& freg_used, const RegMask& vreg_used) const {
//  assert(pipeline->instr &&"Not instruction for read stage");
//
//  // Update pipeline
//  pipeline->valid = true;
//  pipeline->PC = getPC();
//  pipeline->used_iregs.reset();
//  pipeline->used_fregs.reset();
//  pipeline->used_vregs.reset();
//
//  auto& instr = *pipeline->instr;
  for (int tid = 0; tid < getNumThreads(); ++tid) {
    // copy src registers
    for (int i = 0; i < instr.getNRSrc(); ++i) {
      const int rst = instr.getRSType(i);
      const int rs = instr.getRSrc(i);
      if (i) DPN(3, ", ");
      switch (rst) {
        case RegTypes::INTEGER:
          if (ireg_used.test(rs)) {
            return false;
          }
          // pipeline->used_iregs[rs] = 1;
          instr.setRSData(iRegFile_.at(tid).at(rs), tid, i);
          DPN(3, "r" << std::dec << rs << "=0x" << std::hex << iRegFile_.at(tid).at(rs));
          break;
        case RegTypes::FLOAT:
          if (freg_used.test(rs)) {
            return false;
          }
          // pipeline->used_fregs[rs] = 1;
          instr.setRSData(fRegFile_.at(tid).at(rs), tid, i);
          DPN(3, "fr" << std::dec << rs << "=0x" << std::hex << fRegFile_.at(tid).at(rs));
          break;
        default:
          break;
      }
    }
    // copy dst registers
    const int rdt = instr.getRDType();
    const int rd = instr.getRDest();
    switch (rdt) {
      case RegTypes::INTEGER:
        if (ireg_used.test(rd)) {
          return false;
        }
        // pipeline->used_iregs[rd] = 1;
        instr.setRDData(iRegFile_.at(tid).at(rd), tid);
        break;
      case RegTypes::FLOAT:
        if (freg_used.test(rd)) {
          return false;
        }
        // pipeline->used_fregs[rd] = 1;
        instr.setRDData(fRegFile_.at(tid).at(rd), tid);
        break;
      default:
        break;
    }
  }
  // for vector instr values
  // copy src registers
  for (int i = 0; i < instr.getNRSrc(); ++i) {
    if (instr.getRSType(i) == RegTypes::VECTOR) {
      const int rs = instr.getRSrc(i);
      if (vreg_used.test(rs)) {
        return false;
      }
      // pipeline->used_vregs[rs] = 1;
      instr.setVRSData(vRegFile_.at(rs), i);
    }
  }
  // copy dst registers
  if (instr.getRDType() == RegTypes::VECTOR) {
    const int rd = instr.getRDest();
    if (vreg_used.test(rd)) {
      return false;
    }
    // pipeline->used_vregs[rd] = 1;
    instr.setVRDData(vRegFile_.at(rd));
  }
  return true;
}

void Warp::writeback(const Instr& instr) {
  int rdest  = instr.getRDest();
  int rdt = instr.getRDType();

  for (int tid = 0; tid < getNumThreads(); ++tid) {
//    if (!instr.isThreadUsed(tid)) {
//      continue;
//    }
    switch (rdt) {
      case RegTypes::INTEGER:
        if (rdest) {
          D(3, "[" << std::dec << tid << "] Dest Register: r" << rdest << "=0x" << std::hex << std::hex << instr.getRDData(tid));
          iRegFile_[tid][rdest] = instr.getRDData(tid);
        }
        break;
      case RegTypes::FLOAT:
        D(3, "[" << std::dec << tid << "] Dest Register: fr" << rdest << "=0x" << std::hex << std::hex << instr.getRDData(tid));
        fRegFile_[tid][rdest] = instr.getRDData(tid);
        break;
      default:
        break;
    }
  }
  // for vector instr values
  if (instr.getRDest() == RegTypes::VECTOR) {
    vRegFile_[rdest] = instr.getVRDData();
  }
}
