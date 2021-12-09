# Copyright 2021 Norwegian University of Science and Technology.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from pythreejs import (
    DirectionalLight,
    PointLight,
    AmbientLight,
    PerspectiveCamera,
    Scene,
    OrbitControls,
    Renderer,
)


class Viewer:
    def __init__(
        self,
        width=768,
        height=512,
        near=0.01,
        far=100,
        background="#111111",
        antialias=True,
    ):

        key_light = DirectionalLight(color="white", position=[3, 3, 3], intensity=0.5)

        c = PerspectiveCamera(40, width / height, near, far)
        c.position = [2, 2, 2]
        c.up = [0, 0, 1]

        c.add(key_light)

        pl = PointLight(color="white", intensity=0.1, position=[10, 10, 10])

        self._scene = Scene()
        self._scene.background = background
        self._scene.add(AmbientLight())
        self._scene.add(pl)

        self.renderer = Renderer(
            camera=c,
            scene=self._scene,
            antialias=antialias,
            controls=[OrbitControls(controlling=c)],
            height=height,
            width=width,
        )

    def add(self, obj):
        self._scene.add(obj)
