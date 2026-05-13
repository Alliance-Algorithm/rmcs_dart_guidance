#pragma once

#include "manager/core/runtime/action_sequence.hpp"

#include <string>

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// Task
//   任务是带描述信息的顺序动作组，本质上继承自 ActionSequence。
//   具体任务通常在构造函数内按业务流程串接多个 action，供上层状态机直接调度。
// ─────────────────────────────────────────────────────────────────────────────
class Task : public ActionSequence {
public:
    explicit Task(std::string name, std::string description = "")
        : ActionSequence(std::move(name))
        , description_(std::move(description)) {}

    const std::string& description() const { return description_; }

private:
    std::string description_;
};

} // namespace rmcs_dart_guidance::manager
