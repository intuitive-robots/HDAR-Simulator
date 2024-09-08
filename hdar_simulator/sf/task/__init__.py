from typing import Literal

from ..sf_simulator import SFSimulator
from .BoxPushing.BoxPushingSimulator import BoxPushingSimulator
from .BoxPickandPlace.BoxPickandPlaceSimulator import BoxPickandPlaceSimulator as BoxPickandPlace


def sf_task_factory(
        typ: Literal["TemporalCorrelatedAgent"], **kwargs
        ) -> SFSimulator:
    return eval(typ + "(**kwargs)")
