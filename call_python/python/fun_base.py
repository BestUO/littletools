from abc import ABC, abstractmethod
import json


class BaseInterface(ABC):
    def __init__(self, package_dir: str):
        self.password_ = "123"
        self.config_ = json.loads("{}")
        config_path = package_dir + "/config/monitor_config.json"
        try:
            with open(
                config_path,
                "r",
                encoding="utf-8",
            ) as f:
                self.config_ = json.load(f)
        except FileNotFoundError:
            print(f"{config_path} 不存在")
        except json.JSONDecodeError as e:
            print(f"JSON 解析错误: {e}")

    @abstractmethod
    def Start(self, *args, **kwargs):
        pass

    @abstractmethod
    def Stop(self, *args, **kwargs):
        pass

    @abstractmethod
    def CallOnce(self, *args, **kwargs) -> str:
        pass
