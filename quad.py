from mpi4py import MPI
import pybullet as p
import pybullet_data
import numpy as np

# Initialize MPI for parallel execution
comm = MPI.COMM_WORLD
rank = comm.Get_rank()
size = comm.Get_size()

# Start PyBullet in HEADLESS mode (No GUI, No GPU, No Textures)
physics_client = p.connect(p.DIRECT)

# Set physics engine parameters
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Load a basic plane as the terrain
terrain = p.loadURDF("plane.urdf")

# Load Quadruped Robot Model (Replace with your URDF)
robot = p.loadURDF("quadruped.urdf", basePosition=[0, 0, 0.2])

# Generate procedural rough terrain using Perlin noise (no images)
def generate_rough_terrain(size=10, scale=0.1):
    """Creates a simple heightfield for desert-like terrain without using images."""
    heightfield_data = np.random.uniform(-0.5, 0.5, size * size).tolist()
    terrain_shape = p.createCollisionShape(
        shapeType=p.GEOM_HEIGHTFIELD,
        meshScale=[size, size, scale],
        heightfieldTextureScaling=128,  # Purely for scaling, doesn't load textures
        heightfieldData=heightfield_data,
        numHeightfieldRows=size,
        numHeightfieldColumns=size,
    )
    terrain_id = p.createMultiBody(0, terrain_shape)
    return terrain_id

terrain_id = generate_rough_terrain()

# Simulate simple locomotion (trotting motion)
joint_indices = [0, 1, 2, 3]  # Modify based on your robot's URDF

for step in range(5000):  # Increased steps for more simulation time
    for j in joint_indices:
        p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, targetPosition=np.sin(0.1 * step))

    p.stepSimulation()

    # Reduce logging to prevent slowdowns
    if step % 1000 == 0 and rank == 0:
        print(f"Simulation Step: {step}")

p.disconnect()