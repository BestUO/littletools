import fun_base
import subprocess
import re
import json
import psutil


class MonitorImpl(fun_base.BaseInterface):
    def __init__(self, package_dir: str):
        super().__init__(package_dir)
        self.command_ = ["sh", "-c", "sensors"]
        self.temperature_config_ = self.config_.get("temperature_threshold", 70)

    def Start(self, *args, **kwargs):
        return

    def Stop(self, *args, **kwargs):
        print("[INFO] Stopping temperature monitor...", flush=True)
        return

    def __GetCPUFReq(self) -> str:
        result = subprocess.run(
            [
                "sh",
                "-c",
                f"echo {self.password_}| sudo -S cat /sys/devices/system/cpu/cpu*/cpufreq/cpuinfo_cur_freq",
            ],
            capture_output=True,
            text=True,
            timeout=3,
        )
        freq_dict = {}
        for idx, line in enumerate(result.stdout.splitlines()):
            line = line.strip()
            if line.isdigit():
                freq_ghz = int(line) / 1_000_000
                freq_dict[f"cpu{idx}"] = f"{freq_ghz:.3f}GHz"
        return freq_dict

    def __CheckResult(self, stdout: str) -> str:
        lines = stdout.splitlines()
        temps = {}
        current_zone = None
        overheating = False

        for line in lines:
            line = line.strip()
            if line.endswith("-virtual-0"):
                current_zone = line.replace("-virtual-0", "")
            elif current_zone:
                match = re.search(r"temp1:\s+\+([\d.]+)°C", line)
                if match:
                    temp_celsius = float(match.group(1))
                    temps[current_zone] = temp_celsius
                    if temp_celsius > self.temperature_config_:
                        overheating = True
                    current_zone = None
        if overheating:
            output = {
                # "frequency": self.__GetCPUFReq(),
                "temperature": {zone: f"{temp:.2f}" for zone, temp in temps.items()},
            }
            return json.dumps(output, indent=4)
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

    def CallOnce(self, *args, **kwargs) -> str:
        try:
            temps = psutil.sensors_temperatures()
            temps = {
                zone: f"{entries[0][1]:.2f}"
                for zone, entries in temps.items()
                if entries
            }
            any_above = any(
                float(temp) > self.temperature_config_ for temp in temps.values()
            )
            if any_above:
                output = {"temperature": temps}
                print(json.dumps(output, indent=4))
                return json.dumps(output, indent=4)
            else:
                return ""
        except psutil.Error as e:
            print(f"psutil 错误: {type(e).__name__} - {e}")
            return ""
        except (ValueError, TypeError) as e:
            print(f"参数错误: {e}")
            return ""


import json
import psutil

if __name__ == "__main__":
    ttt = MonitorImpl(package_dir="")
    ttt.CallOnce()
