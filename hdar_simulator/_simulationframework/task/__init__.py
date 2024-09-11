from typing import Literal

from ..sf_simulator import SFSimulator
from .BoxPushing.BoxPushing import BoxPushing

def sf_task_factory(
        typ: Literal["TemporalCorrelatedAgent"], **kwargs
        ) -> SFSimulator:
    return eval(typ + "(**kwargs)")
