from .TaskManager import TaskManager
from .CubeStackingManager import CubeStackingManager
from .CupStackingManager import CupStackingManager
from .PracticalManipulation import PracticalManipulation
from .WarmUp import WarmUp
from .BoxStackingManager import BoxStackingManager

TaskManager._repository = {
    "cube_stacking": CubeStackingManager,
    "box_stacking": BoxStackingManager,
    "cup_stacking": CupStackingManager,
    "practical_manipulation": PracticalManipulation,
    "warm_up": WarmUp,
}
