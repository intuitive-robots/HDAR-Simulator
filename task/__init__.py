from .TaskManager import TaskManager
from .CubeStackingManager import CubeStackingManager
from .PracticalManipulation import PracticalManipulationManager
from .UserStudyManager import UserStudyManager

TaskManager._repository = {
    "cube_stacking": CubeStackingManager,
    "practical_manipulation": PracticalManipulationManager,
    "user_study": UserStudyManager,
}
