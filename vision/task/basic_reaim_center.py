from .basic_center import BasicCenterTask
from .constants import BASIC_REAIM_LIMIT_MS


class BasicReaimCenterTask(BasicCenterTask):
    timeout_ms = BASIC_REAIM_LIMIT_MS
