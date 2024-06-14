from typing import Callable

GetLocalDataCallbackType = Callable[[dict], None]
GetOtherDataCallbackType = Callable[[list[dict]], None]