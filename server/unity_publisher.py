from SimPublisher.simpub.sim.sf_publisher import SFPublisher
from alr_sim.core.Scene import Scene
from typing import List, Dict

class UnityPublisher(SFPublisher):
    def __init__(
        self,
        scene: Scene,
        host: str = "127.0.0.1",
        no_rendered_objects: List[str] = None,
        no_tracked_objects: List[str] = None,
    )-> None:
        self.scene = scene
        