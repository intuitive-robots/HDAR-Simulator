from .cube_stacking import CubeStackingManager
from .practical_manipulation import PracticalManipulationManager
from .user_study import UserStudyManager
from .task_manager import get_manager
from . import task_manager

task_manager._repository = {
    "cube_stacking": CubeStackingManager,
    "practical_manipulation": PracticalManipulationManager,
    "user_study": UserStudyManager,
}
