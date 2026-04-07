import fun_base
import subprocess
import re
import psutil
import json


class MonitorImpl(fun_base.BaseInterface):
    def __init__(self, package_dir: str):
        super().__init__(package_dir)
        self.command_ = ["sh", "-c", "top -b -d 1 -o %MEM -n 1"]
        self.mem_config_ = self.config_.get("mem_threshold", 70)

    def Start(self, *args, **kwargs):
        return

    def Stop(self, *args, **kwargs):
        print("[INFO] Stopping memory monitor...", flush=True)
        return

    def __CheckResult(self, stdout: str) -> str:
        lines = stdout.splitlines()
        mem_usage = 0

        for line in lines:
            if "MiB Mem" in line:
                match = re.search(r"([\d.]+)\s+total.*?([\d.]+)\s+used", line)
                if match:
                    total = float(match.group(1))
                    used = float(match.group(2))
                    mem_usage = (used / total) * 100.0
                break

        if mem_usage > self.mem_config_:
            result = "\n".join(lines[:20])
            print(
                f"[WARNING] Memory usage={mem_usage:.1f}%, high memory detected:\n{result}"
            )
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
                "memory_percent",
            ]
        ):
            try:
                processes.append(proc.info)
            except (psutil.NoSuchProcess, psutil.AccessDenied, AttributeError):
                pass
        return processes

    def CallOnce(self, *args, **kwargs) -> str:
        try:
            mem_percent = psutil.virtual_memory().percent
            if mem_percent > self.mem_config_:
                processes = self.Snapshot()
                processes.sort(key=lambda x: x["memory_percent"] or 0, reverse=True)
                output = {"top_memory": processes[:5]}
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
