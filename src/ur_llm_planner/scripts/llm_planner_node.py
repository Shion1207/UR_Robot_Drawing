#!/usr/bin/env python3
"""LLM Task Planner node.

    User Command -> LLM (9Router) -> JSON Plan -> Plan Validator -> Skill Executor -> MoveIt 2

Nhiem vu ca nhan: student.name / student.id (config/student_config.yaml) -> P = XX mod 6 ->
vat nao phai nam o zone A/B/C. Bang gan nay duoc dua vao prompt de LLM xu ly cac lenh nhu
"Arrange all objects according to my student ID".

Nhan cau lenh tu:
  - ban phim (khi chay bang `ros2 run` trong terminal, interactive:=true)
  - topic /nl_command (std_msgs/String)
Ngoai ra, topic /plan_json (std_msgs/String) nhan truc tiep mot ke hoach JSON (bo qua LLM) de
kiem thu Plan Validator va Skill Executor.

LLM chi chon va sap xep skill. Moi ke hoach deu phai qua Plan Validator; neu co skill / object /
zone khong hop le thi TOAN BO ke hoach bi tu choi. Robot chi duoc dieu khien qua service
/execute_skill cua skill_server.
"""

import json
import os
import queue
import sys
import threading
from typing import Dict, List, Optional

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from llm_planner.llm_client import LLMClient, LLMError
from llm_planner.plan_validator import Step, extract_json, validate_plan
from llm_planner.prompt import build_messages, build_repair_message
from llm_planner.student import StudentTask, make_student_task
from ur_llm_planner.srv import ExecuteSkill

GREEN, RED, YELLOW, CYAN, BOLD, RESET = (
    "\033[92m", "\033[91m", "\033[93m", "\033[96m", "\033[1m", "\033[0m")


def format_raw_step(raw) -> str:
    """Hien thi mot buoc JSON (co the khong hop le) dang skill(arg, ...)."""
    if not isinstance(raw, dict):
        return json.dumps(raw, ensure_ascii=False)
    args = [str(v) for k, v in raw.items() if k != "skill"]
    return f"{raw.get('skill')}({', '.join(args)})"


class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__("llm_planner_node")

        self.object_names: List[str] = self.declare_parameter("object_names", [""]).value
        self.zone_names: List[str] = self.declare_parameter("zone_names", [""]).value
        self.object_names = [n for n in self.object_names if n]
        self.zone_names = [n for n in self.zone_names if n]
        if not self.object_names or not self.zone_names:
            raise RuntimeError("Chua khai bao object_names / zone_names (nap config/scene.yaml)")
        self.objects = {n: self.declare_parameter(f"objects.{n}.description", n).value
                        for n in self.object_names}
        self.zones = {n: self.declare_parameter(f"zones.{n}.description", n).value
                      for n in self.zone_names}

        # MSSV: cho phep ca chuoi lan so (vd: -p student.id:=23020123 duoc doc thanh int)
        any_type = ParameterDescriptor(dynamic_typing=True)
        student_name = self.declare_parameter("student.name", "", any_type).value
        student_id = self.declare_parameter("student.id", "", any_type).value
        self.student: Optional[StudentTask] = None
        if str(student_id).strip():
            self.student = make_student_task(student_name, student_id)
            missing = [x for kv in self.student.targets.items() for x in kv
                       if x not in self.object_names + self.zone_names]
            if missing:
                raise RuntimeError(f"Nhiem vu ca nhan dung vat/vung khong co trong scene: {missing}")

        base_url = self.declare_parameter("llm.base_url", "http://localhost:20128/v1").value
        model = self.declare_parameter("llm.model", "").value
        api_key = self.declare_parameter("llm.api_key", "").value or \
            os.environ.get("NINEROUTER_API_KEY", "") or os.environ.get("OPENAI_API_KEY", "")
        timeout = self.declare_parameter("llm.timeout", 60.0).value
        temperature = self.declare_parameter("llm.temperature", 0.0).value
        # Model "reasoning" (vd: gemini-3.8-flash) dung phan lon max_tokens de "nghi" truoc khi
        # tra loi (xem "reasoning_tokens" trong usage) -> can max_tokens lon de khong bi cat cau
        # tra loi giua chung (finish_reason = "length" -> JSON khong hop le).
        max_tokens = self.declare_parameter("llm.max_tokens", 4000).value
        self.max_repair = self.declare_parameter("llm.max_repair_attempts", 2).value
        self.verbose = self.declare_parameter("verbose", False).value
        self.execute = self.declare_parameter("execute", True).value
        self.interactive = self.declare_parameter("interactive", True).value
        self.skill_timeout = self.declare_parameter("skill_timeout", 120.0).value

        self.llm = LLMClient(base_url, model, api_key, timeout, temperature, max_tokens)
        self.get_logger().info(f"LLM: {base_url} | model: {model}")

        self.scene_state: Optional[dict] = None
        self.jobs: "queue.Queue[tuple]" = queue.Queue()

        cb = MutuallyExclusiveCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, "nl_command",
                                 lambda m: self.jobs.put(("command", m.data)), 10,
                                 callback_group=cb)
        self.create_subscription(String, "plan_json",
                                 lambda m: self.jobs.put(("plan", m.data)), 10,
                                 callback_group=cb)
        self.create_subscription(String, "scene_state", self._on_scene_state, latched,
                                 callback_group=cb)
        self.plan_pub = self.create_publisher(String, "llm_planner/plan", 10)
        self.result_pub = self.create_publisher(String, "llm_planner/result", 10)
        self.skill_client = self.create_client(ExecuteSkill, "execute_skill",
                                               callback_group=MutuallyExclusiveCallbackGroup())

        threading.Thread(target=self._worker, daemon=True).start()

    # ------------------------------------------------------------------ callbacks
    def _on_scene_state(self, msg: String):
        try:
            self.scene_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("scene_state khong phai JSON hop le")

    def _worker(self):
        while rclpy.ok():
            kind, text = self.jobs.get()
            try:
                if kind == "command":
                    self.handle_command(text)
                else:
                    self.handle_plan_text(text, command="(plan_json)")
            except Exception as e:  # noqa: BLE001 - khong de worker chet
                self.get_logger().error(f"Loi khong mong doi: {e}")
            finally:
                self.jobs.task_done()

    # ------------------------------------------------------------------ pipeline
    def _held_and_locations(self):
        """Vat dang cam + vi tri (vung) cua tung vat, lay tu /scene_state."""
        state = self.scene_state or {}
        held = state.get("held_object")
        objects = state.get("objects")
        if not isinstance(objects, dict):
            return held, None
        locations: Dict[str, Optional[str]] = {
            name: (info or {}).get("in_zone") for name, info in objects.items()}
        return held, locations

    def print_banner(self):
        if self.student is None:
            print(f"{YELLOW}Chua khai bao student.id - khong co nhiem vu ca nhan{RESET}")
            return
        t = self.student
        print(f"{BOLD}STUDENT:{RESET} {t.name} | ID: {t.student_id} | {t.formula()}")
        print(f"{BOLD}PERSONAL TASK:{RESET} " + " | ".join(t.assignment_lines()))

    def handle_command(self, command: str):
        command = command.strip()
        if not command:
            return
        print(f"\n{BOLD}USER COMMAND:{RESET}\n{command}")
        held, locations = self._held_and_locations()
        messages = build_messages(command, self.objects, self.zones, self.scene_state,
                                  self.student)

        data = None
        result_errors: List[str] = []
        for attempt in range(self.max_repair + 1):
            try:
                reply = self.llm.chat(messages)
            except LLMError as e:
                self._report(command, "LLM_ERROR", errors=[str(e)])
                return
            if self.verbose:
                print(f"\n{CYAN}LLM RAW RESPONSE:{RESET}\n{reply.strip()}")
            try:
                data = extract_json(reply)
            except ValueError as e:
                data = None
                result_errors = [str(e)]
                print(f"\n{BOLD}LLM PLAN:{RESET} (khong doc duoc)\n{reply.strip()}")
            if data is not None:
                self._print_raw_plan(data, attempt)
                result = validate_plan(data, self.object_names, self.zone_names, held, locations)
                if result.ok:
                    self._run_plan(command, result.steps)
                    return
                result_errors = result.errors
                if result.reason and not data.get("plan"):
                    # LLM chu dong tu choi -> khong can hoi lai
                    self._report(command, "REJECTED", plan=data, errors=[result.reason])
                    return
            print(f"{RED}VALIDATOR: plan bi tu choi{RESET}")
            for err in result_errors:
                print(f"   - {err}")
            if attempt < self.max_repair:
                print(f"{YELLOW}Gui loi cho LLM de sua ke hoach "
                      f"({attempt + 1}/{self.max_repair})...{RESET}")
                messages += [{"role": "assistant", "content": reply},
                             build_repair_message(result_errors)]
        self._report(command, "REJECTED", plan=data, errors=result_errors)

    def _print_raw_plan(self, data: dict, attempt: int):
        title = "LLM PLAN:" if attempt == 0 else f"LLM PLAN (sua lan {attempt}):"
        print(f"\n{BOLD}{title}{RESET}")
        plan = data.get("plan")
        if isinstance(plan, list) and plan:
            for raw in plan:
                print(f"    {format_raw_step(raw)}")
        else:
            print(f"    (rong){' - ' + data['reason'] if data.get('reason') else ''}")

    def handle_plan_text(self, text: str, command: str):
        print(f"\n{BOLD}USER COMMAND:{RESET}\n{command}")
        held, locations = self._held_and_locations()
        try:
            data = extract_json(text)
        except ValueError as e:
            self._report(command, "REJECTED", errors=[str(e)])
            return
        self._print_raw_plan(data, 0)
        result = validate_plan(data, self.object_names, self.zone_names, held, locations)
        if not result.ok:
            print(f"{RED}VALIDATOR: plan bi tu choi{RESET}")
            for err in result.errors:
                print(f"   - {err}")
            self._report(command, "REJECTED", plan=data, errors=result.errors)
            return
        self._run_plan(command, result.steps)

    def _run_plan(self, command: str, steps: List[Step]):
        plan = {"plan": [s.to_dict() for s in steps]}
        self.plan_pub.publish(String(data=json.dumps(plan)))
        print(f"{GREEN}VALIDATOR: plan hop le ({len(steps)} buoc){RESET}")
        print(f"\n{BOLD}EXECUTION:{RESET}")
        if not self.execute:
            print("    (execute:=false - chi kiem tra ke hoach, khong dieu khien robot)")
            self._report(command, "VALIDATED", plan=plan)
            return

        if not self.skill_client.wait_for_service(timeout_sec=5.0):
            self._report(command, "FAILED", plan=plan,
                         errors=["Service /execute_skill chua san sang (skill_server chua chay?)"])
            return

        labels = [str(s) for s in steps]
        width = max(len(label) for label in labels) + 3
        step_results = []
        for i, (step, label) in enumerate(zip(steps, labels), start=1):
            print(f"{label} {'.' * (width - len(label))} ", end="", flush=True)
            status, message = self._call_skill(step)
            step_results.append({"step": step.to_dict(), "status": status, "message": message})
            if status == "SUCCESS":
                print(f"{GREEN}SUCCESS{RESET}")
                continue
            print(f"{RED}{status}{RESET}\n    -> {message}")
            for rest in labels[i:]:
                print(f"{rest} {'.' * (width - len(rest))} {YELLOW}SKIPPED{RESET}")
            self._report(command, "FAILED", plan=plan, steps=step_results,
                         errors=[f"Buoc {i} {step} that bai: {status}"])
            return
        self._report(command, "SUCCESS", plan=plan, steps=step_results)

    def _call_skill(self, step: Step):
        req = ExecuteSkill.Request(skill=step.skill, object=step.object, zone=step.zone)
        future = self.skill_client.call_async(req)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=self.skill_timeout):
            return "FAILED", f"Het thoi gian cho skill ({self.skill_timeout}s)"
        res = future.result()
        if res is None:
            return "FAILED", "Khong nhan duoc phan hoi tu skill_server"
        return res.status, res.message

    def _report(self, command, status, plan=None, steps=None, errors=None):
        result = {"command": command, "status": status, "plan": plan,
                  "steps": steps or [], "errors": errors or []}
        self.result_pub.publish(String(data=json.dumps(result, ensure_ascii=False)))
        color = GREEN if status in ("SUCCESS", "VALIDATED") else RED
        print(f"\n{BOLD}{color}TASK {status}{RESET}" +
              (f": {'; '.join(errors)}" if errors else ""))


def default_param_args() -> List[str]:
    """Nap config/scene.yaml + config/llm.yaml cua package lam gia tri mac dinh.

    Tham so truyen tren dong lenh (--ros-args -p ... / --params-file ...) dat SAU nen se ghi de.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("ur_llm_planner")
    except Exception:  # noqa: BLE001
        return []
    args = ["--ros-args"]
    for name in ("scene.yaml", "llm.yaml", "student_config.yaml"):
        path = os.path.join(share, "config", name)
        if os.path.exists(path):
            args += ["--params-file", path]
    return args if len(args) > 1 else []


def main():
    # In log ngay ca khi stdout duoc chuyen huong (launch file, file log)
    sys.stdout.reconfigure(line_buffering=True)
    rclpy.init(args=sys.argv[:1] + default_param_args() + sys.argv[1:])
    try:
        node = LLMPlannerNode()
    except (RuntimeError, ValueError) as e:
        print(f"{RED}{e}{RESET}")
        rclpy.shutdown()
        sys.exit(1)

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.print_banner()
    try:
        if node.interactive and sys.stdin.isatty():
            print(f"{CYAN}Nhap cau lenh cho robot (vd: 'Put the red cube in zone B.', "
                  f"'Arrange all objects according to my student ID.'). "
                  f"Go 'exit' de thoat.{RESET}")
            while rclpy.ok():
                try:
                    line = input("\n>>> ")
                except EOFError:
                    break
                if line.strip().lower() in ("exit", "quit", "q"):
                    break
                node.jobs.put(("command", line))
                node.jobs.join()
        else:
            node.get_logger().info("Dang nghe lenh tren topic /nl_command")
            spin_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
