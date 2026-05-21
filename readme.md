
# 🧠 Project: Simulated Autonomous Mobile Robot with AI (Webots + PyTorch)

--------------------------------------------------------------------------------

## Launch a simulation

1. **Evaluate a Controller**
   - python main.py --eval --`controller_name`
   - python main.py --eval --`controller_name_1` --`controller_name_2`

2. **Train an RL model**
   - python main.py --train --episodes `number_of_episodes`

--------------------------------------------------------------------------------

## Main Objective

Develop a **simulated autonomous robot** in **Webots**, controlled by an **AI model (MLP, LSTM, or GRU)** trained using **simulated sensor data**. The entire project is software-based and runs on a **low-end computer without a GPU**.

--------------------

### Project Steps (big picture)

1. **Install and get started with Webots**
   - Free 3D robotics simulator
   - Configure for lightweight use (no GPU required)
   - Control an existing robot (e.g., e-puck, TurtleBot) with Python

2. **Basic robot control**
   - 2D movements (ignoring the height that is the `z` axis)
   - Move robot using speed and rotation commands
   - Read from simulated sensors: distance, collision, position

3. **Data collection**
   - Actions taken
   - Sensors-based data
   - Kinematic-based data

4. **AI training (PyTorch on CPU)**
   - Train a model:
     - **MLP, SAC-based (RL)**: input → action
   - Validate on a test set

5. **Deploy AI in Webots**
   - Load the trained model
   - Use it in real-time to control the robot inside the simulation

6. **Evaluation & Comparison**
   - Metrics: distance traveled, collisions, goal completion, ...
   - Compare:
     - Basic (rule-based) control
     - Learned AI behavior
     - Random behavior (optional)

--------------------

### Technologies Used

| Tool / Language     | Purpose                                       |
|---------------------|-----------------------------------------------|
| Webots              | Robotics simulation                           |
| Python              | Main programming language (rapid prototyping) |
| PyTorch (CPU only)  | Train neural networks (RL-based)              |
| NumPy / Matplotlib  | Data processing and visualization             |

--------------------

### Constraints

- **No GPU** → must run on **CPU only**
- **No physical hardware**: simulation only
- Must use **free and open-source tools**
- Long-term goal: get closer to **embedded system constraints** (real-time, low memory)

--------------------

### Language Strategy: Python ➜ C++ Transition (optional)

> To better reflect embedded systems practices, I will consider a **progressive transition to C++**:

- Start with **Python** for easier development, testing, and AI training
- Structure code to allow rewriting performance-critical parts in **C++**
- Reuse Webots but switch to a **C++ controller** instead of Python
- Export the trained model (e.g., to ONNX) and use a **C++ runtime** (like ONNX Runtime or a lightweight embedded inference library)

--------------------

### Folders Architecture

```
robot-ai-simulation/
│
├── docs/                   'Documentation, architecture diagrams, ...'
│
├── logs/                   'Notes, experiment logs, debug info'
│   └── log_n.md                 'Log number "n" on the work done'
│
├── ai/                          'All AI-related code'
│   ├── replay_buffer.py             'ReplayBuffer class'
│   ├── actor.py                     'Actor MLP'
│   ├── critic.py                    'Critic MLP'
│   └── sac_agent.py                 'SAC training logic'
│
├── robots/e_puck           'All (e-puck) Webot-related files'
│   ├── controllers/             'C/C++/Python scripts that make the Robot & World work'
|   |   ├── basic/                   'Basic C/C++/Python controller'
|   |   |   └── basic.py                 'Braitenberg controller'
|   |   ├── sac/                     'Basic C/C++/Python controller'
|   |   |   └── sac.py                   'Braitenberg controller'
|   |   ├── supervisor/              'C/C++/Python supervisor'
|   |   |   └── supervisor.py            'A specific controller that manages the world's layout & the Webots camera'
|   |   └── eval/                    'All content linked to the evaluation of the controllers'
|   |       ├── basic_n.json             'Content saved from a simulation of the `basic` controller'
|   |       └── evaluator.py             'Controllers evaluation script'
│   ├── proto/                   'Robot configuration file + some other useful folders'
│   └── worlds/                  'The Worlds (simulation) configuration (".wbt" files)'
│
├── .gitignore              'Files ignored by Git'
│
├── main.py                 'Entry-point'
│
├── readme.md               'Project Description'
│
├── requirements.txt        'Python dependencies'
│
└── setup.md                'Infos on setuping `Webots` & the Project environment'
```

--------------------

### Version Control: Using GitHub Effectively

- **Commit regularly** after each meaningful step:
  - “Set up Webots environment”
  - “First working robot motion”
  - “Logged sensor data successfully”
  - “Trained MLP model”
- This gives:
  - Clear project history
  - Safe backup in case of system failure
  - Professional appearance for your GitHub profile
  - Easier debugging and rollbacks
- I will write a small dev log (`log.md`) and updating the `README.md` progressively

**Branching strategy (optional):**
- `main`: always stable
- `dev-*` branches for experimentation


--------------------------------------------------------------------------------

## Learning Goals

- Master **robotic simulation**
- Apply **RL** to robotics
- Use **AI training frameworks** like PyTorch on CPU
- Use **GitHub efficiently** with meaningful commits
- Transition progressively from **Python to C++** (optional)

--------------------------------------------------------------------------------