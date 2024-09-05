from typing import Literal

from ..SFSimulator import SFSimulator
from .BoxPushing.BoxPushingSimulator import BoxPushingSimulator as BoxPushing
from .BoxPickandPlace.BoxPickandPlaceSimulator import BoxPickandPlaceSimulator as BoxPickandPlace


def sf_task_factory(
        typ: Literal["TemporalCorrelatedAgent"], **kwargs
        ) -> SFSimulator:
    return eval(typ + "(**kwargs)")
