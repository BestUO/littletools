import psutil
import fun_base
import subprocess
import json
import time


class MonitorImpl(fun_base.BaseInterface):
    def __init__(self, package_dir: str):
        super().__init__(package_dir)
        self.io_config_ = self.config_.get("wa_threshold", 30)
        self.command_ = [
            "sh",
            "-c",
            "pidstat -d 1 1 | awk '/^[0-9]/ && !/UID/ && $5>=0 {print}' | sort -k5 -rn",
        ]
        # self.stop_event_ = threading.Event()
        self.last_snapshot_ = {}

    def Start(self, *args, **kwargs):
        # self.thread_ = threading.Thread(target=self.__Run)
        # self.thread_.start()
        pass

    def Stop(self, *args, **kwargs):
        print("[INFO] Stopping IO monitor...", flush=True)
        # self.stop_event_.set()
        # self.thread_.join()
        pass

    def __CheckResult(self, stdout: str) -> list:
        output = {
            "top_io": [line.strip() for line in stdout.splitlines() if line.strip()],
        }
        return output

    def Snapshot(self) -> dict:
        processes = {}
        for proc in psutil.process_iter(["pid", "name", "cmdline"]):
            try:
                io = proc.io_counters()
                processes[proc.pid] = {
                    "pid": proc.info["pid"],
                    "name": proc.info["name"],
                    "cmdline": proc.info["cmdline"],
                    "rb": io.read_bytes,
                    "wb": io.write_bytes,
                }
            except (psutil.NoSuchProcess, psutil.AccessDenied, AttributeError):
                pass
        return processes

    def CallOnceOld(self, *args, **kwargs) -> str:
        cpu_times = psutil.cpu_times_percent(interval=1)
        iowait = getattr(cpu_times, "iowait", 0.0)

        if iowait <= self.io_config_:
            return ""

        result = subprocess.run(
            self.command_,
            capture_output=True,
            text=True,
            timeout=3,
        )
        output = {
            "io": self.__CheckResult(result.stdout),
        }
        print(json.dumps(output, indent=4))
        return json.dumps(output, indent=4)

    def CallOnce(self, *args, **kwargs) -> str:
        try:
            cpu_times = psutil.cpu_times_percent()
            iowait = getattr(cpu_times, "iowait", 0.0)
            snapshot = self.Snapshot()
            deltas = {}
            for pid, info in snapshot.items():
                if pid in self.last_snapshot_:
                    prev_info = self.last_snapshot_[pid]
                    deltas[pid] = {
                        "pid": pid,
                        "name": info["name"],
                        "cmdline": info["cmdline"],
                        "rb_delta": (info["rb"] - prev_info["rb"])
                        / 1024,  # Convert to KB
                        "wb_delta": (info["wb"] - prev_info["wb"])
                        / 1024,  # Convert to KB
                    }
                else:
                    deltas[pid] = {
                        "pid": pid,
                        "name": snapshot[pid]["name"],
                        "cmdline": info["cmdline"],
                        "rb_delta": snapshot[pid]["rb"] / 1024,  # Convert to KB
                        "wb_delta": snapshot[pid]["wb"] / 1024,  # Convert to KB
                    }
            self.last_snapshot_ = snapshot

            if iowait <= self.io_config_:
                return ""
            else:
                sorted_deltas = sorted(
                    deltas.values(),
                    key=lambda d: max(d["rb_delta"], d["wb_delta"]),
                    reverse=True,
                )
                output = {"top_io": sorted_deltas[:5]}
                return json.dumps(output, indent=4)
        except psutil.Error as e:
            print(f"psutil 错误: {type(e).__name__} - {e}")
            return ""
        except (ValueError, TypeError) as e:
            print(f"参数错误: {e}")
            return ""

    def __Run(self):
        while not self.stop_event_.is_set():
            self.stop_event_.wait(1)


if __name__ == "__main__":
    ttt = MonitorImpl(package_dir="")
    print(ttt.CallOnce())
    time.sleep(1)
    print(ttt.CallOnce())
