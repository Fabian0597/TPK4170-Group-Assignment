from pythreejs import Group
from viewer import Viewer
from models import ColladaMesh, Grid, Line, Trihedron, Ball

class Scene:
    def __init__(self, viewer, trihedron, axes, links, target):
        self.viewer = viewer
        self.trihedron = trihedron
        self.axes = axes
        self.links = links
        self.target = target

def create_scene():
    viewer = Viewer(width=512, height=512, background="#ffffff")
    viewer.add(Grid(color="#aaaaaa"))

    # Add the robot model to the scene
    robot = ColladaMesh("robot_model.dae", scale=1e-3*10)
    num_links = len(robot.children)
    num_joints = num_links - 1
    robot.matrixAutoUpdate = False
    robot.matrix = [1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1] # Put the robot rightside up
    viewer.add(robot)

    # Add the trihedrons to the scene
    trihedron = [Trihedron(size=0.2, linewidth=2) for _ in range(num_links)]
    for t in trihedron:
        t.matrixAutoUpdate = False
        robot.add(t)

    # Add the screw axes to the scene
    axes = [Line([0, 0, -0.2], [0, 0, 0.2], "#44025c", linewidth=2) for _ in range(num_joints)]
    for a in axes:
        a.matrixAutoUpdate = False
        robot.add(a)

    # Add the target to grab
    target = Group()
    target.matrixAutoUpdate = False
    target.add(Ball("#999999"))
    target.add(Trihedron(size=0.1, linewidth=2))
    robot.add(target)
        
    # Get the links models
    links = robot.children
    for l in links:
        l.matrixAutoUpdate = False
        
    return Scene(viewer, trihedron, axes, links, target)