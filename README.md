# Car Parking Simulation

A Python project for simulating and planning car parking maneuvers using various search algorithms.

## Table of Contents
- [Project Structure](#project-structure)
- [System Design](#system-design)
  - [Geometry Foundation](#geometry-foundation)
  - [Object Representations](#object-representations)
  - [Collision Detection System](#collision-detection-system)
  - [Search Algorithm Framework](#search-algorithm-framework)
  - [Algorithm Implementations](#algorithm-implementations)
- [Usage](#usage)
- [Key Components](#key-components)
- [Dependencies](#dependencies)
- [Results](#results)
- [AI-Generated Content](#ai-generated-content)

<details>
<summary><strong>📁 Project Structure</strong></summary>

```
car_project/
├── src/
│   ├── __init__.py          # Main package exports
│   ├── core/                # Core data structures and utilities
│   │   ├── __init__.py
│   │   ├── action.py        # Action definitions (straight, rotate)
│   │   ├── car.py           # Car geometry and kinematics
│   │   ├── direction.py     # Direction enum (LEFT, RIGHT)
│   │   ├── node.py          # Search node for algorithms
│   │   ├── parking_space.py # Parking space definition
│   │   ├── path.py          # Path representation (straight/arc segments)
│   │   └── point.py         # 2D point geometry
│   ├── algorithms/          # Parking algorithms
│   │   ├── __init__.py
│   │   ├── bfs_base.py      # Base class for parking search
│   │   ├── bfs_parking.py   # BFS parking planner
│   │   ├── bidir_bfs_parking.py # Bidirectional BFS parking
│   │   ├── heuristic.py     # Heuristic functions
│   │   └── parallel_parking.py # Parallel parking algorithm
│   ├── tests/               # Test files
│   │   ├── __init__.py
│   │   ├── bfs_test.py      # BFS algorithm tests
│   │   ├── bfs_reference_test.py # Reference BFS tests
│   │   ├── heuristic_bidir_test.py # Bidirectional heuristic tests
│   │   └── heuristic_test.py # Heuristic function tests
│   └── utils/               # Utility functions
│       ├── __init__.py
│       └── plot_utils.py    # Plotting utilities
├── car_env/                 # Python virtual environment
└── README.md               # This file
```
</details>

<details>
<summary><strong>🔧 System Design</strong></summary>

### Geometry Foundation

The system is built upon a geometric foundation that abstracts 2D space and motion:

**Point Class**: The fundamental abstraction representing a 2D vector with geometric operations:
- `angle()`: Returns the angle of the vector from the origin
- `rotate(angle)`: Rotates the point around the origin by a given angle
- `norm()`: Returns the Euclidean norm (magnitude) of the vector
- Standard vector operations: addition, subtraction, scalar multiplication

**PathPart Abstraction**: An abstract base class representing geometric path segments that can be:
- **StraightPart**: A straight line segment between two points
- **ArcPart**: A circular arc parameterized by center point, radius, start angle, and sweep angle

All PathPart implementations provide:
- `intersects(other)`: Determines if two path segments intersect
- `draw(ax)`: Renders the path segment on a matplotlib Axes object

**Path Class**: A composite structure containing an ordered sequence of PathPart objects, representing complete trajectories.

### Object Representations

#### Car System

**CarSpecs**: Immutable physical specifications of a vehicle:
- `length`: Total vehicle length in meters
- `width`: Vehicle width in meters  
- `rear_axle`: Distance from rear to rear axle in meters
- `minimum_turning_radius`: Minimum radius the vehicle can turn in meters

**CarState**: Complete pose description of a car at any moment:
- `specs`: Reference to the CarSpecs object
- `orientation`: Current heading angle in radians
- `center`: Position of the car's geometric center as a Point

The car is abstractly represented as a rectangle with four corner points, computed from the center, orientation, and specifications.

**Motion Primitives**: CarState provides two fundamental motion operations:
- `straight(distance)`: Moves the car forward/backward by a specified distance
- `rotate(angle, radius, direction)`: Rotates the car around a specified radius in the given direction

Both methods return a new CarState and optionally update a Path object if provided. More abstractly, an Action object can be applied using `apply_action(action)`, which encapsulates either straight or rotate operations.

#### Parking Space Representation

Parking spaces are modeled as collections of line segments forming boundaries:

**ParkingSpace Components**:
- `start`: Reference point for the parking space
- `width`: Width of the parking space
- `length`: Length of the parking space
- `baseline_left/right`: Extension distances beyond the parking space on left/right sides
- `road_width`: Width of the road adjacent to the parking space

The parking space consists of multiple curb segments:
- Bottom curb (parking space boundary)
- Side curbs (left and right boundaries)
- Top curb (road boundary)
- Extended baseline segments

*[Image placeholder: Parking space diagram showing curb segments and dimensions]*

### Collision Detection System

The system implements a two-tier collision detection approach:

**Static Collision Detection**: Checks if the car's rectangular representation intersects with any parking space curb segments in its current position.

**Dynamic Collision Detection**: Traces the complete paths taken by all four corners of the car during a motion primitive and checks for intersections with curb segments along the entire trajectory.

**Collision Handling**: When applying actions, if a ParkingSpace is provided and either static or dynamic collision is detected, a `CollisionException` is raised, preventing the invalid motion.

**Limitation**: The current implementation only checks corner trajectories. Edge segments between corners are not explicitly checked for collisions, which could theoretically miss some edge-curb intersections.

### Search Algorithm Framework

#### State Space Discretization

Although the physical world is continuous, the search operates on a discretized state space:
- Car positions are quantized to a grid resolution
- Orientations are quantized to discrete angle steps
- States are hashed for efficient storage and lookup in hash tables

#### Action Parameterization

**Rotation Actions**: Parameterized by:
- `radius`: Turning radius (using logarithmic scale for efficient coverage)
- `direction`: LEFT or RIGHT
- `angle`: Rotation magnitude in radians

**Straight Actions**: Parameterized by:
- `magnitude`: Distance to travel (positive/negative for forward/backward)

#### Node Expansion Strategy

**Pruning Mechanisms**:
1. **Collision-based pruning**: If a motion intersects curbs, further expansion in that direction is terminated
2. **Distance-based pruning**: States beyond a threshold distance (1.5x the initial distance to goal) are pruned to focus search on relevant areas

**State Management**: 
- Visited states are stored in a hash set for O(1) lookup
- Frontier states are maintained in appropriate data structures (queue for BFS, priority queue for A*)

### Algorithm Implementations

#### Base Framework

**`_ParkingSearchBase`**: Abstract base class implementing the core search infrastructure:
- State hashing and discretization
- Goal state checking
- Path reconstruction
- Common search utilities

#### Breadth-First Search (BFS)

**Configuration Parameters**:
- `step_resolution`: Distance step size for straight motions
- `step_angle_resolution`: Angle step size for rotations  
- `max_straight`: Maximum straight-line distance
- `max_radius`: Maximum turning radius
- `max_radius_count`: Number of radius values to try
- `depth`: Maximum search depth limit
- `max_turn_sweep`: Maximum rotation angle per action
- `max_dist_ratio`: Distance pruning threshold (1.5x initial distance)

**Algorithm**: Standard BFS with depth limiting and collision-aware expansion.

**Example Usage**:
```python
from src import CarSpecs, CarState, ParkingSpace, BFSParking, ExpansionConfig, Point

specs = CarSpecs(length=5, width=2, rear_axle=1, minimum_turning_radius=7)
start = CarState(specs, 0.0, Point(-10, 2))
goal = CarState(specs, 0.0, Point(0, -2))
parking = ParkingSpace(Point(-5, 0), width=4, length=10)

config = ExpansionConfig(
    step_resolution=2.0,
    step_angle_resolution=np.deg2rad(10),
    max_straight=8.0,
    depth=4
)

planner = BFSParking(start, [goal], parking, expansion_config=config)
path, actions = planner.search()
```

#### Heuristic BFS (A* Algorithm)

**Priority Queue Strategy**: Nodes are expanded based on heuristic priority rather than FIFO order, significantly improving search efficiency.

**Heuristic Functions Tested**:

1. **Goal Distance**: Simple Euclidean distance to goal center
2. **Line Distance**: Perpendicular distance from car's midline to goal center, weighted with rear axle distance
3. **Curb Distance**: Minimum distance from car's long sides to target curb point
4. **Top Curb Distance**: Distance from car sides to the road boundary (entry point)
5. **Hybrid Heuristic**: Combines line distance (for far distances) with curb distance (for close distances), using a threshold of 5 meters

**Performance Impact**: Heuristic-guided search dramatically reduces expansion count by focusing on promising directions.

**Example**:
```python
from src.algorithms.heuristic import distance_from_curb

config = ExpansionConfig(
    step_resolution=2.0,
    depth=4,
    heuristic=distance_from_curb  # Use curb-based heuristic
)
```

#### Bidirectional BFS

**Core Insight**: Exiting a parking space is computationally easier than entering it, making bidirectional search highly effective.

**Implementation**: 
- **Forward Search**: From start state toward goal (often with no heuristic)
- **Reverse Search**: From goal state toward start (using top curb distance heuristic)
- **Meet-in-the-Middle**: Searches meet at an intermediate state, then paths are reconstructed

**Configuration Strategy**:
- Forward expansion: Lower depth (2-3), no heuristic for broad exploration
- Reverse expansion: Higher depth (3-4), top curb distance heuristic for focused search

**Example**:
```python
from src import BidirBFSParking

# Forward config: broad exploration
cfg_start = ExpansionConfig(depth=2, heuristic=None)

# Reverse config: focused search  
cfg_goal = ExpansionConfig(depth=3, heuristic=distance_from_top_curb)

planner = BidirBFSParking(
    start_state=start,
    goal_states=[goal], 
    parking_space=parking,
    config_start=cfg_start,
    config_goal=cfg_goal
)
```

</details>

<details>
<summary><strong>🚀 Usage</strong></summary>

### Basic Import

```python
from src import CarSpecs, CarState, ParkingSpace, BFSParking, ExpansionConfig, Point
import numpy as np

# Create car specifications
specs = CarSpecs(length=4.5, width=1.8, rear_axle=1.0, minimum_turning_radius=5.0)

# Create car state
start_state = CarState(specs=specs, orientation=0.0, center=Point(0, 0))
goal_state = CarState(specs=specs, orientation=0.0, center=Point(10, 0))

# Create parking space
parking = ParkingSpace(start=Point(10, 5), width=2.5, length=5.0)

# Configure the planner
expansion_config = ExpansionConfig(
    step_resolution=2.0,
    step_angle_resolution=np.deg2rad(10),
    max_radius=100.0,
    max_radius_count=5,
    max_straight=8.0,
    depth=4,
)

# Create planner and search
planner = BFSParking(
    start_state=start_state,
    goal_states=[goal_state],
    parking_space=parking,
    expansion_config=expansion_config,
    grid_resolution=2.0,
    angle_resolution=np.deg2rad(10),
)
result = planner.search()
if result:
    path, actions = result
```

### Running Tests

```bash
# Run a specific test (recommended for this structure):
python -m src.tests.bfs_test
```

</details>

<details>
<summary><strong>🔑 Key Components</strong></summary>

### Core Modules
- **Car**: Handles car geometry, kinematics, and collision detection
- **ParkingSpace**: Defines parking space boundaries and constraints
- **Path**: Represents car trajectories as sequences of straight and arc segments
- **Action**: Defines motion primitives (straight movement, rotation)

### Algorithms
- **BFSParking**: Breadth-first search parking planner
- **BidirBFSParking**: Bidirectional BFS for improved performance
- **ParallelParking**: Specialized algorithm for parallel parking scenarios

### Heuristics
- **distance_from_curb**: Distance-based heuristic for parking accuracy
- **line_distance**: Line-based heuristic for path planning
- **line_then_curb**: Hybrid heuristic combining multiple approaches

</details>

<details>
<summary><strong>📦 Dependencies</strong></summary>

- numpy
- matplotlib

</details>

<details>
<summary><strong>📊 Results</strong></summary>

*[Results section to be added later]*

</details>

<details>
<summary><strong>⚠️ AI-Generated Content</strong></summary>

⚠️ **DISCLAIMER**: The following sections are bluntly AI-generated. 
I think it's still worth a read if you are interested.

### Performance Optimizations

**Road Boundary Addition**: Including the top curb segment (road boundary) significantly improves search performance by providing better spatial constraints.

**Grid Resolution Impact**: 
- Too coarse grids can cause reconstruction failures in bidirectional search
- Fine grids improve accuracy but increase computational cost
- Optimal resolution balances accuracy with performance

**Heuristic Selection**: Different heuristics perform better for different scenarios:
- Line distance works well for long-range navigation
- Curb distance excels for precise parking maneuvers
- Hybrid approaches provide robust performance across scenarios

## Design Choices and Rationale

### Geometric Abstraction Decisions

**PathPart Hierarchy**:
- **Choice**: Used abstract base class with StraightPart and ArcPart implementations
- **Rationale**: This design allows for extensibility (future path types like splines) while providing a unified interface for intersection testing and visualization. The abstraction enables polymorphic behavior without complex type checking.

**Rectangle Car Representation**:
- **Choice**: Represented cars as rectangles rather than more complex shapes
- **Rationale**: Balances computational efficiency with sufficient accuracy for parking scenarios. While real cars have curved edges, the rectangular approximation captures the essential geometric constraints for collision detection and path planning.

### State Management Architecture

**Immutable CarSpecs**:
- **Choice**: Made CarSpecs immutable using `@dataclass(frozen=True)`
- **Rationale**: Vehicle specifications don't change during operation, and immutability prevents accidental modifications and enables safe sharing across multiple CarState instances.

**Functional State Updates**:
- **Choice**: Motion primitives return new CarState instances rather than modifying existing ones
- **Rationale**: Enables easy state history tracking, simplifies debugging, and prevents side effects that could corrupt search state. This functional approach aligns well with the search algorithm's need to explore multiple state trajectories.

**Action Abstraction**:
- **Choice**: Created Action class to encapsulate motion primitives
- **Rationale**: Provides a uniform interface for both straight and rotate operations, enables action reversal for bidirectional search, and makes the search algorithm independent of specific motion implementations.

### Collision Detection Strategy

**Two-Tier Collision Detection**:
- **Choice**: Implemented both static and dynamic collision checking
- **Rationale**: Static detection catches immediate collisions, while dynamic detection prevents invalid trajectories. This dual approach ensures safety without excessive computational overhead.

**Corner-Only Trajectory Checking**:
- **Choice**: Only check corner trajectories for dynamic collision detection
- **Rationale**: While theoretically incomplete, this approach captures the vast majority of collision scenarios with significantly reduced computational complexity. The trade-off between completeness and performance was deemed acceptable for this application.

**Exception-Based Collision Handling**:
- **Choice**: Raise CollisionException rather than returning boolean collision status
- **Rationale**: Forces explicit handling of collision scenarios and prevents silent failures. This design choice makes collision handling more robust and debuggable.

### Search Algorithm Design

**Discretization Strategy**:
- **Choice**: Discretize continuous state space using grid resolution and angle quantization
- **Rationale**: Enables finite search spaces and efficient state hashing. While this introduces approximation error, the resolution can be tuned to balance accuracy with computational feasibility.

**Logarithmic Radius Sampling**:
- **Choice**: Use geometric progression for turning radius values
- **Rationale**: Provides better coverage of the radius space with fewer samples. Small radius changes have larger effects on trajectory than large radius changes, making logarithmic sampling more efficient than linear sampling.

**Hash-Based State Storage**:
- **Choice**: Use hash tables for visited state tracking
- **Rationale**: Provides O(1) average case lookup and insertion, essential for handling large search spaces efficiently. The discretization enables reliable hashing of continuous state values.

### Algorithm Selection Rationale

**BFS as Foundation**:
- **Choice**: Started with breadth-first search as the base algorithm
- **Rationale**: BFS guarantees optimality (shortest path) and provides a clear baseline for performance comparison. Its simplicity makes it easier to debug and extend.

**Heuristic Integration**:
- **Choice**: Added heuristic guidance while maintaining depth limiting
- **Rationale**: Heuristics dramatically improve search efficiency without sacrificing the safety guarantees of depth limiting. The combination provides the benefits of A* with additional safety constraints.

**Bidirectional Search**:
- **Choice**: Implemented bidirectional BFS as the final approach
- **Rationale**: Leverages the asymmetric difficulty of entering vs. exiting parking spaces. The "meet-in-the-middle" approach reduces the effective search depth and often finds solutions faster than unidirectional search.

### Configuration Management

**ExpansionConfig Pattern**:
- **Choice**: Centralized configuration in ExpansionConfig dataclass
- **Rationale**: Provides a clean interface for algorithm parameterization, enables easy experimentation with different parameter combinations, and separates configuration from algorithm logic.

**Separate Forward/Reverse Configs**:
- **Choice**: Allow different configurations for bidirectional search directions
- **Rationale**: Enables optimization of each search direction based on its specific characteristics. Forward search benefits from broad exploration, while reverse search benefits from focused heuristics.

### Performance Optimization Decisions

**Road Boundary Inclusion**:
- **Choice**: Include top curb segment in parking space representation
- **Rationale**: Provides additional spatial constraints that guide search more effectively. This simple addition significantly improves search performance without major architectural changes.

**Distance-Based Pruning**:
- **Choice**: Prune states beyond 1.5x initial goal distance
- **Rationale**: Focuses search on relevant areas while maintaining a reasonable safety margin. This heuristic pruning reduces search space without sacrificing solution quality in most cases.

**Grid Resolution Tuning**:
- **Choice**: Make grid resolution configurable rather than fixed
- **Rationale**: Different scenarios require different accuracy levels. Fine grids improve solution quality but increase computation time, while coarse grids are faster but may miss solutions in complex scenarios.

</details>