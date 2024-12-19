from typing import Literal

from ..sf_simulator import SFSimulator
from .BoxPushing.BoxPushing import BoxPushing,BoxPushingVibration
#single arm task
from .PushCube.PushCube import PushCube,PushCubeVibration
from .AssembleBox.AssembleBox import AssembleBox,AssembleBoxVibration
from .PickandPlaceBox.PickandPlaceBox import PickandPlaceBox,PickandPlaceBoxVibration
from .OpenDoor.OpenDoor import OpenDoor,OpenDoorVibration
from .OpenDrawer.OpenDrawer import OpenDrawer,OpenDrawerVibration
#dual arm task
from .HoldBoard.HoldBoard import HoldBoard,HoldBoardVibration
from .PushPuzzle.PushPuzzle import PushPuzzle,PushPuzzleVibration
from .TwoArmAssemble.TwoArmAssemble import TwoArmAssemble,TwoArmAssembleVibration
from .PutIteminDrawer.PutIteminDrawer import PutIteminDrawer,PutIteminDrawerVibration
from .HandoverBox.HandoverBox import HandoverBox,HandoverBoxVibration


def sf_task_factory(
        typ: Literal["TemporalCorrelatedAgent"], **kwargs
        ) -> SFSimulator:
    return eval(typ + "(**kwargs)")
