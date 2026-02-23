# SLAM with Line Features

a real-time slam implementation using line segment extraction from lidar data. I extract line features from 2d laser scans, track them as landmarks, and visualize everything as the robot moves around.

<img width="2499" height="1377" alt="image" src="https://github.com/user-attachments/assets/ce35f9e2-e287-453e-a099-888e6ceb4ac4" />
<img width="2499" height="1377" alt="image" src="https://github.com/user-attachments/assets/ce35f9e2-e287-453e-a099-888e6ceb4ac4" />

*robot exploring the environment with line features being extracted in real-time*

## what it does

- extracts line segments from lidar point clouds using seeded region growing
- prevents bridging across doors/windows with gap detection
- tracks landmarks over time with data association
- lets you drive a robot around with keyboard controls
- visualizes everything: point cloud, line segments, landmarks, robot path

The current implememntation uses a topdown cros0section of an indoor environment shown below.
<img width="1536" height="1024" alt="image" src="https://github.com/user-attachments/assets/10caf801-ae4d-46b5-8b96-510a7579c055" />
<img width="1536" height="1024" alt="image" src="https://github.com/user-attachments/assets/10caf801-ae4d-46b5-8b96-510a7579c055" />
![Uploading image.png…]()
![Uploading image.png…]()


## quick start

```bash
# install dependencies
pip install -r requirements.txt

# run it
python main.py
```

when it starts, click anywhere to position your robot. then use wasd to drive around.

![Positioning Mode](screenshots/positioning.png)
*click to place the robot before starting*

## controls

**movement:**
- `W` / `↑` - forward
- `S` / `↓` - backward
- `A` / `←` - rotate left
- `D` / `→` - rotate right
- `Q` - forward + left
- `E` - forward + right

**visualization:**
- `1` - point cloud only
- `2` - seed segments
- `3` - final line segments
- `4` - everything (default)
- `5` - toggle path on/off

**other:**
- `P` - reposition robot
- `R` - reset path

## basic features

### line extraction

I use a seeded region growing algorithm that:
1. finds seed segments (small groups of collinear points)
2. grows them by adding nearby points that fit the line
3. checks for gaps to avoid bridging across openings
4. validates segments to filter out bad ones

the key improvement here is gap detection - I check three things before connecting points:
- spatial gap (are they too far apart?)
- angular gap (sudden direction change?)
- range discontinuity (depth jump indicating an opening?)

this prevents the classic problem where line extractors connect points across doors and windows.

![Line Extraction](screenshots/line_extraction.png)
*line segments extracted from lidar scan - notice how they stop at doorways*

### landmark tracking

extracted line segments become landmarks. I:
- associate new observations with existing landmarks
- track how many times I've seen each landmark
- only trust "strong" landmarks (seen multiple times)
- remove old landmarks that haven't been seen recently

landmarks that persist across multiple frames are shown in cyan.

## Legends
- green dots - raw lidar points
- yellow lines - seed segments (initial line fits)
- red lines - final extracted line segments
- cyan lines - strong landmarks (seen multiple times)
- magenta - robot's path
- blue circle - robot (yellow line shows heading)

![Full Visualization](screenshots/full_viz.png)
*all visualization layers enabled*

## configuration

edit `config.py` to tune things - here are the important ones:

**lidar settings:**
- `LIDAR_BEAMS` - number of laser beams (420 default, try 720 for denser clouds)
- `LIDAR_NOISE_SIGMA` - sensor noise `[distance, angle]` (increase for testing robustness)

**line extraction:**
- `LINE_EXTRACTION_EPSILON` - how tight points must fit the line
- `LINE_EXTRACTION_G_MAX` - max gap betIen consecutive points
- `LINE_EXTRACTION_L_MIN` - minimum line length to keep
- `LINE_EXTRACTION_ANGLE_GAP_MAX` - max angular jump (prevents bridging)
- `LINE_EXTRACTION_DISTANCE_JUMP_RATIO` - max depth change ratio (prevents bridging)

**robot:**
- `MAX_VELOCITY` - forward/backward speed
- `MAX_ANGULAR_VELOCITY` - rotation speed


## maps

I include a few test maps in `world/maps/`:
- `indoor1.png` - office-like environment
- `cave1.png`, `cave2.png`, `cave3.png` - cave environments

you can add your own maps - just use black for walls and white for free space.

## performance

runs at 60 fps with:
- 420 lidar beams
- real-time line extraction
- collision detection
- landmark tracking
- full visualization

if it's slow, try reducing `LIDAR_BEAMS` in config.

## known issues

- robot can get stuck in tight corners (collision detection is conservative)
- path grows forever (could add max length)
- no loop closure yet
- ekf-slam is just a stub for now

## what's next

things I could add:
- [ ] full ekf-slam implementation
- [ ] uncertainty visualization (covariance ellipses)
- [ ] loop closure detection
- [ ] map saving/loading
- [ ] multiple robots
- [ ] path planning

## credits and references for my implementation

line extraction algorithm from:
**"A line segment extraction algorithm using laser data based on seeded region growing"**
by Xiao Zhang, Guokun Lai, Xianqiang Yang (2018)
[Read the paper](https://www.researchgate.net/publication/323122919_A_line_segment_extraction_algorithm_using_laser_data_based_on_seeded_region_growing)

I implemented algorithms 1-3 from the paper with additional gap detection to prevent bridging across openings.

built with pygame and numpy.

## license

do whatever you want with it. 😁️️
