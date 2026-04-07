import fun_base
import subprocess
import re
import psutil
import time
import json


class MonitorImpl(fun_base.BaseInterface):
    def __init__(self, package_dir: str):
        super().__init__(package_dir)
        self.command_ = ["sh", "-c", "top -b -d 1 -o %CPU -n 1"]
        self.cpu_config_ = self.config_.get("cpu_threshold", 70)

    def Start(self, *args, **kwargs):
        return

    def Stop(self, *args, **kwargs):
        print("[INFO] Stopping CPU monitor...", flush=True)
        return

    def __CheckResult(self, stdout: str) -> str:
        lines = stdout.splitlines()
        cpu_busy = 0

        for line in lines:
            if line.startswith("%Cpu") or "Cpu(s)" in line:
                match = re.search(r"([\d.]+)\s+id", line)
                if match:
                    cpu_busy = 100 - float(match.group(1))
                break

        if cpu_busy > self.cpu_config_:
            result = "\n".join(lines[:20])
            # print(f"[WARNING] CPU busy={cpu_busy}%, high load detected:\n{result}")
            return result
        else:
            return ""

    def CallOnceOld(self, *args, **kwargs) -> str:
        result = subprocess.run(
            self.command_,
            capture_output=True,
            text=True,
            timeout=3,
        )
        return self.__CheckResult(result.stdout)

    def Snapshot(self) -> list:
        processes = []
        for proc in psutil.process_iter(
            [
                "pid",
                "name",
                "cmdline",
                "cpu_percent",
            ]
        ):
            try:
                processes.append(proc.info)
            except (psutil.NoSuchProcess, psutil.AccessDenied, AttributeError):
                pass
        return processes

    def CallOnce(self, *args, **kwargs) -> str:
        try:
            cpu_times = psutil.cpu_times_percent()
            print(f"CPU Times: {cpu_times} {self.cpu_config_}")
            # cpu_stats = psutil.cpu_stats()
            # print(f"CPU Stats: {cpu_stats}")
            if 100 - cpu_times.idle > self.cpu_config_:
                processes = self.Snapshot()
                processes.sort(key=lambda x: x["cpu_percent"] or 0, reverse=True)
                output = {
                    "top_cpu": processes[:5],
                    "cpu_times": {
                        "user": cpu_times.user,
                        "system": cpu_times.system,
                        "idle": cpu_times.idle,
                        "iowait": getattr(cpu_times, "iowait", 0.0),
                        "irq": getattr(cpu_times, "irq", 0.0),
                        "softirq": getattr(cpu_times, "softirq", 0.0),
                    },
                }
                return json.dumps(output, indent=4)
            else:
                return ""
        except psutil.Error as e:
            print(f"psutil 错误: {type(e).__name__} - {e}")
            return ""
        except (ValueError, TypeError) as e:
            print(f"参数错误: {e}")
            return ""


if __name__ == "__main__":
    ttt = MonitorImpl(package_dir="")
    ttt.CallOnce()
