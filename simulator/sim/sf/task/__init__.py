from typing import Literal

from ..SFSimulator import SFSimulator
from .BoxPushing.BoxPushingSimulator import BoxPushingSimulator
from .BoxPushing.BoxPushingSimulator import BoxPushingSimulator as BoxPushing

from .BoxPickandPlace.BoxPickandPlaceSimulator import BoxPickandPlaceSimulator
from .BoxPickandPlace.BoxPickandPlaceSimulator import BoxPickandPlaceSimulator as PickandPlaceBox
from .DrawerOpening.OpenDrawerSimulator import OpenDrawerSimulator
from .DrawerOpening.OpenDrawerSimulator import OpenDrawerSimulator as DrawerOpening
from .DoorOpening.OpenDoorSimulator import OpenDoorSimulator
from .DoorOpening.OpenDoorSimulator import OpenDoorSimulator as OpenDoor
from .BoxPushingwithFeedback.BoxPushSimulator import BoxPushSimulator
from .BoxPushingwithFeedback.BoxPushSimulator import BoxPushSimulator as BoxPush
from .BoxAssemble.BoxAssembleSimulator import BoxAssembleSimulator
from .BoxAssemble.BoxAssembleSimulator import BoxAssembleSimulator as BoxAssemble


from .BimanualAssemble.BimanualAssmbleSimulator import BimanualAssembleSimulator
from .BimanualAssemble.BimanualAssmbleSimulator import BimanualAssembleSimulator as AssembleBimanual
from .BimanualPushing.BigBoxPush import BimanualPushingSimulator
from .BimanualPushing.BigBoxPush import BimanualPushingSimulator as PushBimanual
from .BimanualHolding.BimanualHoldSimulator import BimanualHoldingSimulator
from .BimanualHolding.BimanualHoldSimulator import BimanualHoldingSimulator as HoldBimanual
from .BimanualPutIteminDraw.PutIteminDrawSimulator import BimanualPutIteminDrawSimulator
from .BimanualPutIteminDraw.PutIteminDrawSimulator import BimanualPutIteminDrawSimulator as PutIteminDraw


def sf_task_factory(
        typ: Literal["TemporalCorrelatedAgent"], **kwargs
        ) -> SFSimulator:
    return eval(typ + "(**kwargs)")
