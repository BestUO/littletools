import fun_base
import threading
import time
from pathlib import Path
import shutil


class MonitorImpl(fun_base.BaseInterface):
    def __init__(self, package_dir: str):
        super().__init__(package_dir)
        self.clean_disk_ = self.config_.get(
            "clean_disk",
            {
                "interval_day": 1,
                "disk_space_left_G": 40,
                "paths": [
                    {
                        "path": "",
                        "dir_size_G": 10,
                        "file_size_M": 1,
                        "keep_day": 7,
                    }
                ],
            },
        )
        self.stop_event_ = threading.Event()

    def Start(self, *args, **kwargs):
        self.thread_ = threading.Thread(target=self.__Run)
        self.thread_.start()

    def Stop(self, *args, **kwargs):
        print("[INFO] Stopping disk monitor...", flush=True)
        self.stop_event_.set()
        self.thread_.join()

    def CallOnce(self, *args, **kwargs) -> str:
        return ""

    def __DiskSpaceLeft(self) -> int:
        stat = shutil.disk_usage("/")
        return stat.free / (1024**3)

    def __DirSizeG(self, path: Path) -> float:
        total = sum(f.stat().st_size for f in path.rglob("*") if f.is_file())
        return total / (1024**3)

    def __CleanPath(self, path_cfg: dict):
        p = Path(path_cfg["path"]).expanduser().resolve()
        if not p.exists() or not p.is_dir():
            print(
                f"[WARN] Path does not exist or is not a directory, skipping: {p}",
                flush=True,
            )
            return

        dir_size_G = path_cfg.get("dir_size_G", 10)
        file_size_M = path_cfg.get("file_size_M", 1)
        keep_day = path_cfg.get("keep_day", 7)

        current_size = self.__DirSizeG(p)
        print(
            f"[INFO] check directory {p}  "
            f"current usage={current_size:.2f}G  threshold={dir_size_G}G",
            flush=True,
        )

        if current_size <= dir_size_G:
            print(
                f"[INFO] Directory does not exceed the threshold, skipping cleanup.",
                flush=True,
            )
            return

        now = time.time()
        keep_seconds = keep_day * 24 * 3600
        file_size_B = file_size_M * 1024 * 1024

        deleted_count = 0
        deleted_size = 0.0

        for f in sorted(
            p.rglob("*"), key=lambda x: x.stat().st_mtime if x.is_file() else 0
        ):
            if not f.is_file():
                continue
            try:
                stat = f.stat()
                age_seconds = now - stat.st_mtime
                size_bytes = stat.st_size

                if age_seconds > keep_seconds and size_bytes > file_size_B:
                    deleted_size += size_bytes
                    f.unlink()
                    deleted_count += 1
                    print(
                        f"[DELETE] {f}  "
                        f"size={size_bytes / (1024**2):.2f}M  "
                        f"passed={age_seconds / 86400:.1f}天",
                        flush=True,
                    )
            except Exception as e:
                print(f"[ERROR] delete {f} fail : {e}", flush=True)

        for d in sorted(p.rglob("*"), key=lambda x: len(x.parts), reverse=True):
            if d.is_dir():
                try:
                    d.rmdir()
                except OSError:
                    pass

        print(
            f"[INFO] finished: delete {deleted_count} files,"
            f"release {deleted_size / (1024**3):.2f}G",
            flush=True,
        )

    def __Run(self):
        while not self.stop_event_.is_set():
            print(f"[{time.ctime()}] begin disk space check", flush=True)

            disk_left = self.__DiskSpaceLeft()
            threshold = self.clean_disk_["disk_space_left_G"]
            print(
                f"[INFO] available disk space={disk_left:.2f}G  threshold={threshold}G",
                flush=True,
            )

            if disk_left < threshold:
                print(
                    "[INFO] Insufficient free space, starting to traverse and clean up paths...",
                    flush=True,
                )
                for path_cfg in self.clean_disk_["paths"]:
                    if path_cfg["path"]:
                        self.__CleanPath(path_cfg)
            else:
                print(
                    "[INFO] Sufficient free space available, no cleanup required",
                    flush=True,
                )

            self.stop_event_.wait(timeout=self.clean_disk_["interval_day"] * 24 * 3600)


if __name__ == "__main__":
    ttt = MonitorImpl(package_dir="")
    ttt.Start()
    time.sleep(1000)
